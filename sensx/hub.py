"""SensXHub -- streaming driver for a TouchTronix Hub PCB with up to five sensors.

The hub board multiplexes up to five SensX sensors on a single serial link.
Every sensor has its own frame header:

    ======  ==============
    Sensor  Header
    ======  ==============
    A       ``0xAA 0xAA``
    B       ``0xBB 0xBB``
    C       ``0xCC 0xCC``
    D       ``0xDD 0xDD``
    E       ``0xEE 0xEE``
    ======  ==============

Hub frame format (NO CRC -- differs from single-sensor SensX)::

    [header: 2 bytes][payload: rows*cols*2 bytes, 12-bit big-endian]

Frames from the sensors follow each other back-to-back on the wire, in no
fixed channel order. Each letter belongs to a connector on the hub, not to a
sensor; an empty connector sends nothing and the other letters stay the same.

Blocking read (five SensX25 (5x5) sensors, 921 600 baud)::

    from sensx import SensXHub, CHANNEL_OF

    hub = SensXHub("COM7", baud_rate=921600, rows_a=5, cols_a=5, channels="ABCDE")
    while True:
        header, frame = hub.read_frame()
        print(CHANNEL_OF[header], frame.max())

Callback::

    from sensx import SensXHub
    import time

    hub = SensXHub("COM7", baud_rate=921600, rows_a=5, cols_a=5, channels="ABCDE")
    hub.on_frame = lambda channel, frame, ts: print(channel, frame.max())
    hub.start()
    time.sleep(10)
    hub.close()

Context manager::

    from sensx import SensXHub
    import time

    with SensXHub("COM7", baud_rate=921600, rows_a=5, cols_a=5, channels="ABCDE") as hub:
        hub.on_frame = lambda channel, frame, ts: print(channel, frame.max())
        hub.start()
        time.sleep(10)

Output mode (SensX25)::

    hub.set_output_mode("adc")       # or "pressure"
    print(hub.get_output_mode())     # "adc"

SensX25 sensors output either pressure (factory setting, kPa, 0-534) or
ADC counts (0-3945: the 12-bit ADC reading minus a fixed offset of 150, so
an unloaded sensor reads 0). The setting is stored in the sensors and kept
through power-off. Through the hub one command sets every sensor at once.
"""

from __future__ import annotations

import logging
import threading
import time
from collections import deque
from contextlib import contextmanager
from typing import Callable, Deque, Dict, Iterator, List, Optional, Set, Tuple

import numpy as np
import serial

from sensx import _modbus


logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Protocol constants
# ---------------------------------------------------------------------------
HEADER_A = b"\xaa\xaa"
HEADER_B = b"\xbb\xbb"
HEADER_C = b"\xcc\xcc"
HEADER_D = b"\xdd\xdd"
HEADER_E = b"\xee\xee"
HEADERS = (HEADER_A, HEADER_B, HEADER_C, HEADER_D, HEADER_E)
CHANNELS = "ABCDE"
#: Channel letter for each header, e.g. ``CHANNEL_OF[HEADER_C] == "C"``.
CHANNEL_OF: Dict[bytes, str] = dict(zip(HEADERS, CHANNELS))
CHANNEL_HEADER: Dict[str, bytes] = dict(zip(CHANNELS, HEADERS))
HEADER_LEN = 2
BYTES_PER_PIXEL = 2

#: Output modes accepted by :meth:`SensXHub.set_output_mode`.
OUTPUT_MODES: Dict[str, int] = {"adc": 0, "pressure": 1}
_MODE_NAMES = {value: name for name, value in OUTPUT_MODES.items()}

#: All sensors answer on Modbus address 1.
_MODBUS_ADDRESS = 1
#: Seconds to listen for replies after each settings command.
_REPLY_WINDOW_S = 0.2
#: The sensors ignore commands for a moment after a write; wait this long
#: after any write before sending the next settings command.
_SETTLE_S = 0.3
#: Reads of the output mode before giving up on hearing from every sensor.
#: Behind the hub some reads miss a reply, sometimes several in a row.
_MAX_MODE_READS = 20
#: Reads with no reply at all before deciding the sensors don't answer.
_MAX_SILENT_READS = 4

#: Frames held for ``read_frame()`` when the caller reads slower than the hub
#: sends; beyond this the oldest frames are dropped.
_MAX_PENDING_FRAMES = 1024


class SensXHub:
    """Driver for a TouchTronix Hub PCB hosting up to **five** SensX sensors.

    Parameters
    ----------
    port : str
        Serial port path (e.g. ``"/dev/ttyUSB0"`` or ``"COM7"``).
    baud_rate : int
        Baud rate (default 1 500 000). Use 921 600 for SensX25 (5x5) sensors.
    rows_a, cols_a : int
        Grid dimensions of sensor A (default 16 x 12).
    rows_b, cols_b : int
        Grid dimensions of sensor B (defaults to same as sensor A).
    serial_timeout : float
        Serial read timeout in seconds.
    read_chunk : int
        Bytes to read per ``serial.read()`` call.
    channels : str
        Hub channels with a sensor connected, e.g. ``"ABCDE"`` or ``"ACE"``
        (default ``"AB"``). Frames from other channels are ignored. Sensors
        C, D and E use the same grid as sensor A.
    """

    def __init__(
        self,
        port: str,
        baud_rate: int = 1_500_000,
        rows_a: int = 16,
        cols_a: int = 12,
        rows_b: Optional[int] = None,
        cols_b: Optional[int] = None,
        serial_timeout: float = 0.005,
        read_chunk: int = 4096,
        channels: str = "AB",
    ) -> None:
        wanted = channels.upper()
        if not wanted or set(wanted) - set(CHANNELS) or len(set(wanted)) != len(wanted):
            raise ValueError(
                f"channels must be distinct letters from {CHANNELS!r}, got {channels!r}"
            )

        self.port = port
        self.baud_rate = baud_rate
        self._channels = "".join(c for c in CHANNELS if c in wanted)

        # Sensor geometry (B defaults to A; C-E always follow A)
        self.rows_a = rows_a
        self.cols_a = cols_a
        self.rows_b = rows_b if rows_b is not None else rows_a
        self.cols_b = cols_b if cols_b is not None else cols_a
        self._shape: Dict[bytes, Tuple[int, int]] = {
            h: (rows_a, cols_a) for h in HEADERS
        }
        self._shape[HEADER_B] = (self.rows_b, self.cols_b)
        self._frame_size: Dict[bytes, int] = {
            h: HEADER_LEN + rows * cols * BYTES_PER_PIXEL
            for h, (rows, cols) in self._shape.items()
        }
        # All five headers are recognised for alignment, but only frames
        # from the configured channels are delivered.
        self._active = frozenset(CHANNEL_HEADER[c] for c in self._channels)

        self._read_chunk = read_chunk

        # Serial port (opened immediately so wiring errors surface early)
        self._ser = serial.Serial(port, baud_rate, timeout=serial_timeout)

        # Send init command (same as single-sensor driver)
        init_cmd = bytes.fromhex("01 06 60 10 00 01 57 CF")
        self._ser.write(init_cmd)
        self._ser.flush()
        self._last_write = time.perf_counter()

        # Latest frame and timestamp per sensor (thread-safe)
        self._lock = threading.Lock()
        self._frames: Dict[bytes, np.ndarray] = {
            h: np.zeros(shape, dtype=np.uint16) for h, shape in self._shape.items()
        }
        self._timestamps: Dict[bytes, float] = {h: 0.0 for h in HEADERS}

        # Blocking-read state: unparsed bytes and parsed frames not yet returned
        self._buf = bytearray()
        self._pending: Deque[Tuple[bytes, np.ndarray, float]] = deque(
            maxlen=_MAX_PENDING_FRAMES
        )

        # Background reader
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

        # User callbacks
        self.on_frame: Optional[Callable[[str, np.ndarray, float], None]] = None
        self.on_frame_a: Optional[Callable[[np.ndarray, float], None]] = None
        self.on_frame_b: Optional[Callable[[np.ndarray, float], None]] = None

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def channels(self) -> str:
        """Configured channel letters, e.g. ``"ABCDE"`` for five sensors."""
        return self._channels

    @property
    def latest_frame_a(self) -> np.ndarray:
        """Return a copy of the most recent sensor-A frame (thread-safe)."""
        with self._lock:
            return self._frames[HEADER_A].copy()

    @property
    def latest_frame_b(self) -> np.ndarray:
        """Return a copy of the most recent sensor-B frame (thread-safe)."""
        with self._lock:
            return self._frames[HEADER_B].copy()

    @property
    def latest_timestamp_a(self) -> float:
        """Timestamp (``time.perf_counter()``) of the most recent sensor-A frame."""
        return self.get_timestamp("A")

    @property
    def latest_timestamp_b(self) -> float:
        """Timestamp (``time.perf_counter()``) of the most recent sensor-B frame."""
        return self.get_timestamp("B")

    @property
    def is_running(self) -> bool:
        """Whether the background reader thread is alive."""
        return self._thread is not None and self._thread.is_alive()

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> None:
        """Start the background reader thread."""
        if self.is_running:
            return
        self._stop_event.clear()
        if not self._ser.is_open:
            self._ser.open()
        self._ser.reset_input_buffer()
        self._thread = threading.Thread(
            target=self._reader_loop, name="SensXHub-Reader", daemon=True
        )
        self._thread.start()

    def stop(self, join: bool = True) -> None:
        """Stop the background reader thread."""
        self._stop_event.set()
        if join and self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=2.0)

    def close(self) -> None:
        """Stop streaming and close the serial port."""
        self.stop()
        if self._ser.is_open:
            self._ser.close()

    # ------------------------------------------------------------------
    # Context manager
    # ------------------------------------------------------------------

    def __enter__(self) -> "SensXHub":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # ------------------------------------------------------------------
    # Frame parsing (no CRC on hub)
    # ------------------------------------------------------------------

    def _parse_frame(self, header: bytes, raw: bytes) -> np.ndarray:
        """Parse raw frame bytes into a numpy array (no CRC check)."""
        rows, cols = self._shape[header]
        payload = raw[HEADER_LEN:]
        frame = np.frombuffer(payload, dtype=">u2").reshape(rows, cols)
        return (frame & 0x0FFF).astype(np.uint16)

    def _find_next_header(
        self, buf: bytearray, start: int = 0
    ) -> Tuple[Optional[int], Optional[bytes]]:
        """Find the earliest occurrence of any sensor header.

        Returns ``(index, header_bytes)`` or ``(None, None)`` if none is found.
        """
        best_idx: Optional[int] = None
        best_header: Optional[bytes] = None
        for header in HEADERS:
            idx = buf.find(header, start)
            if idx != -1 and (best_idx is None or idx < best_idx):
                best_idx, best_header = idx, header
        return best_idx, best_header

    def _is_valid_header(self, buf: bytearray, idx: int) -> bool:
        """Check if bytes at position idx form a valid frame header."""
        if idx + HEADER_LEN > len(buf):
            return False
        return bytes(buf[idx : idx + HEADER_LEN]) in self._frame_size

    def _extract_frames(self, buf: bytearray) -> List[Tuple[bytes, np.ndarray]]:
        """Remove every complete frame from *buf* and return them in order.

        A frame is accepted only when the next frame's header follows it
        immediately, so header-like bytes inside a payload, or other bytes on
        the line (such as replies to settings commands), are skipped instead
        of producing a misaligned frame. An incomplete frame at the end stays
        in *buf* until more data arrives.
        """
        frames: List[Tuple[bytes, np.ndarray]] = []
        while True:
            idx, header = self._find_next_header(buf)
            if idx is None or header is None:
                # Keep the last byte: it may be the first half of a header.
                del buf[:-1]
                return frames
            if idx > 0:
                del buf[:idx]

            fsize = self._frame_size[header]

            # Need enough data for current frame + next header validation
            if len(buf) < fsize + HEADER_LEN:
                return frames

            if not self._is_valid_header(buf, fsize):
                # False header found inside payload - skip it
                del buf[:1]
                continue

            # Confirmed proper frame boundary
            raw = bytes(buf[:fsize])
            del buf[:fsize]
            if header in self._active:
                frames.append((header, self._parse_frame(header, raw)))

    # ------------------------------------------------------------------
    # Frame storage and callbacks
    # ------------------------------------------------------------------

    def _store_frame(self, header: bytes, frame: np.ndarray, ts: float) -> None:
        """Store the latest frame and timestamp for a sensor (thread-safe)."""
        with self._lock:
            self._frames[header][:] = frame
            self._timestamps[header] = ts

    def _dispatch(self, header: bytes, frame: np.ndarray, ts: float) -> None:
        """Store a frame and run the user callbacks for it."""
        self._store_frame(header, frame, ts)
        channel = CHANNEL_OF[header]
        if header == HEADER_A:
            per_sensor = self.on_frame_a
        elif header == HEADER_B:
            per_sensor = self.on_frame_b
        else:
            per_sensor = None
        if per_sensor is not None:
            try:
                per_sensor(frame, ts)
            except Exception:
                logger.exception("Error in on_frame_%s callback", channel.lower())
        if self.on_frame is not None:
            try:
                self.on_frame(channel, frame, ts)
            except Exception:
                logger.exception("Error in on_frame callback")

    # ------------------------------------------------------------------
    # Synchronous (blocking) reads
    # ------------------------------------------------------------------

    def read_frame(self) -> Tuple[bytes, np.ndarray]:
        """Block until one complete frame (from any sensor) is received.

        Returns ``(header, frame)`` where *header* is one of
        :data:`HEADER_A` ... :data:`HEADER_E` (``CHANNEL_OF[header]`` gives the
        letter) and *frame* is a ``numpy.ndarray``.

        Frames are returned in arrival order. Every frame in each chunk read
        from the port is kept, so none are lost while the caller keeps up;
        a caller that falls far behind loses the oldest frames first. Do
        **not** mix this with ``start()``; use one pattern or the other.
        """
        while not self._pending:
            chunk = self._ser.read(self._read_chunk)
            if not chunk:
                # No data available (timeout) -- brief yield to prevent busy spin
                time.sleep(0.001)
                continue
            self._buf += chunk
            ts = time.perf_counter()
            for header, frame in self._extract_frames(self._buf):
                self._pending.append((header, frame, ts))

        header, frame, ts = self._pending.popleft()
        self._store_frame(header, frame, ts)
        return header, frame

    def read_frame_a(self) -> np.ndarray:
        """Block until the next sensor-A frame arrives."""
        while True:
            header, frame = self.read_frame()
            if header == HEADER_A:
                return frame

    def read_frame_b(self) -> np.ndarray:
        """Block until the next sensor-B frame arrives."""
        while True:
            header, frame = self.read_frame()
            if header == HEADER_B:
                return frame

    def read_frames(
        self, timeout: Optional[float] = None
    ) -> Tuple[Optional[np.ndarray], ...]:
        """Block until one frame from each configured channel has been received.

        Returns one frame per channel in :attr:`channels` order:
        ``(frame_a, frame_b)`` by default, ``(frame_a, ..., frame_e)`` for
        ``channels="ABCDE"``. Any entry may be ``None`` if *timeout* expires
        first; pass a timeout if a configured connector may be empty.
        """
        deadline = None if timeout is None else time.monotonic() + timeout
        wanted = [CHANNEL_HEADER[c] for c in self._channels]
        got: Dict[bytes, np.ndarray] = {}

        while len(got) < len(wanted):
            if deadline is not None and time.monotonic() > deadline:
                break
            header, frame = self.read_frame()
            got.setdefault(header, frame)

        return tuple(got.get(h) for h in wanted)

    def get_frame(self, channel: str) -> Optional[np.ndarray]:
        """Return the latest frame from a sensor (non-blocking).

        *channel* is ``"A"`` ... ``"E"``. Returns a copy of the most recent
        frame, or None if no frame received yet. Thread-safe - can be called
        from any thread while hub is running.
        """
        header = self._header_for(channel)
        with self._lock:
            if self._timestamps[header] == 0:
                return None
            return self._frames[header].copy()

    def get_timestamp(self, channel: str) -> float:
        """Timestamp (``time.perf_counter()``) of a sensor's most recent frame.

        Returns 0.0 if no frame has been received yet.
        """
        header = self._header_for(channel)
        with self._lock:
            return self._timestamps[header]

    def get_frame_a(self) -> Optional[np.ndarray]:
        """Return the latest frame from sensor A (non-blocking).

        Returns a copy of the most recent frame, or None if no frame received yet.
        Thread-safe - can be called from any thread while hub is running.
        """
        return self.get_frame("A")

    def get_frame_b(self) -> Optional[np.ndarray]:
        """Return the latest frame from sensor B (non-blocking).

        Returns a copy of the most recent frame, or None if no frame received yet.
        Thread-safe - can be called from any thread while hub is running.
        """
        return self.get_frame("B")

    def _header_for(self, channel: str) -> bytes:
        letter = channel.upper()
        if len(letter) != 1 or letter not in CHANNELS:
            raise ValueError(f"channel must be one of {', '.join(CHANNELS)}; got {channel!r}")
        return CHANNEL_HEADER[letter]

    # ------------------------------------------------------------------
    # Output mode (SensX25)
    # ------------------------------------------------------------------

    def get_output_mode(self) -> str:
        """Return the sensors' output mode: ``"adc"`` or ``"pressure"``.

        Streaming pauses for a moment while the sensors are asked. Raises
        RuntimeError if no sensor answers or if the sensors are not all in
        the same mode (fix that with :meth:`set_output_mode`).
        """
        with self._paused():
            values, complete, live = self._read_output_mode()

        if not live:
            raise RuntimeError(
                f"No frames from channels {self._channels}; check the port, "
                "baud rate and connections."
            )
        if not values:
            raise RuntimeError(
                "No reply to the output-mode query. Output-mode switching is "
                "supported on SensX25 (5x5) sensors."
            )
        unknown = sorted(set(values) - set(_MODE_NAMES))
        if unknown:
            raise RuntimeError(f"Unexpected output-mode value(s) from the sensors: {unknown}")
        if len(set(values)) > 1:
            raise RuntimeError(
                "The sensors are not all in the same output mode "
                f"(replies: {[_MODE_NAMES[v] for v in values]}). "
                "Use set_output_mode() to set them all."
            )
        self._warn_silent(live)
        if not complete:
            logger.warning(
                "Output mode heard from fewer than the %d sensors sending data.",
                len(live),
            )
        return _MODE_NAMES[values[0]]

    def set_output_mode(self, mode: str, attempts: int = 3) -> None:
        """Set the output mode of every sensor on the hub.

        *mode* is ``"pressure"`` (factory setting, kPa) or ``"adc"`` (ADC
        counts). The hub passes the command to all sensors at once; sensors
        cannot be set individually through the hub. The setting is stored in
        the sensors and kept through power-off.

        The new mode is read back from every sensor that is sending data on
        the configured channels before returning; configured channels that
        send nothing are logged as a warning. If the sensors are already in
        *mode*, nothing is written. Streaming pauses (usually about a second,
        at most several seconds) and resumes on its own; frames received
        during the change are discarded. Raises RuntimeError if the change
        cannot be confirmed.
        """
        key = mode.lower()
        if key not in OUTPUT_MODES:
            raise ValueError(f"mode must be one of {sorted(OUTPUT_MODES)}; got {mode!r}")
        target = OUTPUT_MODES[key]
        write = _modbus.request(
            _MODBUS_ADDRESS, _modbus.FUNC_WRITE, _modbus.REG_OUTPUT_MODE, target
        )

        with self._paused():
            # Only write to sensors that report a valid output mode.
            values, complete, live = self._read_output_mode()
            if not live:
                raise RuntimeError(
                    f"No frames from channels {self._channels}; nothing was changed. "
                    "Check the port, baud rate and connections."
                )
            if not values:
                raise RuntimeError(
                    "No reply to the output-mode query; nothing was changed. "
                    "Output-mode switching is supported on SensX25 (5x5) sensors."
                )
            unknown = sorted(set(values) - set(_MODE_NAMES))
            if unknown:
                raise RuntimeError(
                    f"Unexpected output-mode value(s) {unknown}; nothing was changed."
                )
            self._warn_silent(live)
            if complete and set(values) == {target}:
                return

            for _ in range(attempts):
                self._transact(write)
                values, complete, live = self._read_output_mode()
                if complete and set(values) == {target}:
                    return
                if not complete and set(values) <= {target}:
                    break  # a sensor never answers; writing again won't help

        raise RuntimeError(
            f"Could not confirm output mode {key!r} on all {len(live)} sensors "
            f"sending data (channels {''.join(sorted(live))}; last replies: "
            f"{[_MODE_NAMES.get(v, v) for v in values]}). A connected sensor "
            "is not answering."
        )

    def _read_output_mode(self) -> Tuple[List[int], bool, Set[str]]:
        """Ask the sensors for their output mode.

        Every connected sensor answers, but behind the hub a reply is now
        and then lost on the way back, so the question is repeated until a
        single read returns an answer from every sensor that is sending
        frames on the configured channels.

        Returns all values heard, whether such a complete read happened, and
        the configured channels seen sending frames.
        """
        query = _modbus.request(
            _MODBUS_ADDRESS, _modbus.FUNC_READ, _modbus.REG_OUTPUT_MODE, 1
        )
        heard: List[int] = []
        live: Set[str] = set()
        for attempt in range(1, _MAX_MODE_READS + 1):
            rx = self._transact(query)
            live |= {CHANNEL_OF[h] for h, _ in self._extract_frames(bytearray(rx))}
            values = _modbus.find_read_replies(rx, _MODBUS_ADDRESS)
            heard += values
            if live and len(values) >= len(live):
                return heard, True, live
            if not heard and attempt >= _MAX_SILENT_READS:
                break  # nothing answers at all
        return heard, False, live

    def _warn_silent(self, live: Set[str]) -> None:
        """Log configured channels that sent no frames (empty connectors)."""
        silent = [c for c in self._channels if c not in live]
        if silent:
            logger.warning(
                "No data on channel(s) %s; is a sensor connected there?",
                ", ".join(silent),
            )

    def _transact(self, request: bytes) -> bytes:
        """Send a settings command and return everything received after it."""
        wait = self._last_write + _SETTLE_S - time.perf_counter()
        if wait > 0:
            time.sleep(wait)
        self._ser.reset_input_buffer()
        self._ser.write(request)
        self._ser.flush()
        if request[1] == _modbus.FUNC_WRITE:
            self._last_write = time.perf_counter()
        rx = bytearray()
        deadline = time.perf_counter() + _REPLY_WINDOW_S
        while time.perf_counter() < deadline:
            rx += self._ser.read(self._read_chunk)
        return bytes(rx)

    @contextmanager
    def _paused(self) -> Iterator[None]:
        """Stop the reader thread (if running) around a settings command."""
        if self._thread is not None and threading.current_thread() is self._thread:
            raise RuntimeError("Sensor settings cannot be changed from a frame callback.")
        was_running = self.is_running
        if was_running:
            self.stop()
            if self.is_running:
                raise RuntimeError("Reader thread did not stop; settings not changed.")
        try:
            yield
        finally:
            # Frames received before and during the command are stale.
            self._buf.clear()
            self._pending.clear()
            self._ser.reset_input_buffer()
            if was_running:
                self.start()

    # ------------------------------------------------------------------
    # Background reader loop
    # ------------------------------------------------------------------

    def _reader_loop(self) -> None:
        buf = bytearray()

        while not self._stop_event.is_set():
            try:
                chunk = self._ser.read(self._read_chunk)
            except serial.SerialException:
                break
            if not chunk:
                # No data available -- brief yield to prevent busy spin
                time.sleep(0.001)
                continue

            buf += chunk
            ts = time.perf_counter()

            # Parse as many complete frames as possible
            for header, frame in self._extract_frames(buf):
                self._dispatch(header, frame, ts)
