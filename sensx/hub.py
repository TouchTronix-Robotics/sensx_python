"""SensXHub -- streaming driver for a TouchTronix Hub PCB with up to five sensors.

The hub board multiplexes up to five SensX sensors on a single serial link.
Every sensor has its own frame header, ``HEADER_A`` (``0xAA 0xAA``) to
``HEADER_E`` (``0xEE 0xEE``), one per hub connector. An empty connector sends
nothing, so the sensors that send frames are the ones connected.

Hub frame format (NO CRC -- differs from single-sensor SensX)::

    [header: 2 bytes][payload: rows*cols*2 bytes, 12-bit big-endian]

Frames from the sensors follow each other back-to-back on the wire, in no
fixed order.

Blocking read (five SensX25 (5x5) sensors, 921 600 baud)::

    from sensx import SensXHub

    hub = SensXHub("COM7", baud_rate=921600, rows=5, cols=5)
    while True:
        header, frame = hub.read_frame()     # header: HEADER_A ... HEADER_E
        print(header.hex(), frame.max())

Callback::

    from sensx import SensXHub
    import time

    with SensXHub("COM7", baud_rate=921600, rows=5, cols=5) as hub:
        hub.on_frame = lambda header, frame, ts: print(header.hex(), frame.max())
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
from typing import Callable, Deque, Dict, Iterator, List, Optional, Tuple

import numpy as np
import serial


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
HEADER_LEN = 2
BYTES_PER_PIXEL = 2

# Output mode (SensX25): Modbus register 0x6005 at address 1, value = index
# in _MODES (0 = ADC counts, 1 = pressure).
_MODES = ("adc", "pressure")
_MODE_REGISTER = 0x6005
#: Seconds to listen for replies after a settings command.
_REPLY_WINDOW_S = 0.2
#: The sensors ignore a command sent right after a write; wait this long.
_SETTLE_S = 0.3
#: The hub loses some replies, sometimes several reads in a row.
_MAX_MODE_READS = 20

#: Frames held for ``read_frame()`` when the caller reads slower than the hub
#: sends; beyond this the oldest frames are dropped.
_MAX_PENDING_FRAMES = 1024
#: Serial read timeout (s) and bytes per read.
_SERIAL_TIMEOUT_S = 0.005
_READ_CHUNK = 4096


def _crc16(data: bytes) -> bytes:
    """Modbus CRC-16 (poly 0xA001, init 0xFFFF), low byte first."""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x01:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return bytes([crc & 0xFF, crc >> 8])


def _mode_request(function: int, value: int) -> bytes:
    """Modbus request for the output-mode register: read (0x03) or write (0x06)."""
    body = bytes([1, function, _MODE_REGISTER >> 8, _MODE_REGISTER & 0xFF, 0, value])
    return body + _crc16(body)


def _mode_replies(rx: bytes) -> List[int]:
    """Values of every valid read reply ``01 03 02 hi lo crc crc`` in *rx*.

    The replies arrive inside the frame stream, one per sensor.
    """
    values: List[int] = []
    i = rx.find(b"\x01\x03\x02")
    while 0 <= i <= len(rx) - 7:
        if _crc16(rx[i : i + 5]) == rx[i + 5 : i + 7]:
            values.append((rx[i + 3] << 8) | rx[i + 4])
        i = rx.find(b"\x01\x03\x02", i + 1)
    return values


class SensXHub:
    """Driver for a TouchTronix Hub PCB hosting up to **five** SensX sensors.

    Parameters
    ----------
    port : str
        Serial port path (e.g. ``"/dev/ttyUSB0"`` or ``"COM7"``).
    baud_rate : int
        Baud rate of the hub: 921 600 for SensX25 (5x5) sensors.
    rows, cols : int
        Grid dimensions of the sensors. All sensors on a hub are the same type.
    """

    def __init__(self, port: str, baud_rate: int, rows: int, cols: int) -> None:
        self.rows = rows
        self.cols = cols
        self._frame_size = HEADER_LEN + rows * cols * BYTES_PER_PIXEL

        # Serial port (opened immediately so wiring errors surface early).
        # The hub streams on its own from power-on; nothing needs to be sent.
        self._ser = serial.Serial(port, baud_rate, timeout=_SERIAL_TIMEOUT_S)
        self._last_write = 0.0

        # Latest frame and timestamp, pre-allocated for all five sensors
        self._lock = threading.Lock()
        self._frames: Dict[bytes, np.ndarray] = {
            h: np.zeros((rows, cols), dtype=np.uint16) for h in HEADERS
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

        # User callback: on_frame(header, frame, ts)
        self.on_frame: Optional[Callable[[bytes, np.ndarray, float], None]] = None

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def connected(self) -> Tuple[bytes, ...]:
        """Headers of the sensors that have sent frames, e.g. ``(HEADER_A, HEADER_C)``."""
        with self._lock:
            return tuple(h for h in HEADERS if self._timestamps[h])

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
        self._ser.reset_input_buffer()
        self._thread = threading.Thread(
            target=self._reader_loop, name="SensXHub-Reader", daemon=True
        )
        self._thread.start()

    def stop(self) -> None:
        """Stop the background reader thread."""
        self._stop_event.set()
        if self._thread is not None and self._thread.is_alive():
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

    def _parse_frame(self, raw: bytes) -> np.ndarray:
        """Parse raw frame bytes into a numpy array (no CRC check)."""
        payload = raw[HEADER_LEN:]
        frame = np.frombuffer(payload, dtype=">u2").reshape(self.rows, self.cols)
        return frame.astype(np.uint16)

    def _find_next_header(self, buf: bytearray) -> Tuple[Optional[int], Optional[bytes]]:
        """Find the earliest occurrence of any sensor header.

        Returns ``(index, header_bytes)`` or ``(None, None)`` if none is found.
        """
        best_idx: Optional[int] = None
        best_header: Optional[bytes] = None
        for header in HEADERS:
            idx = buf.find(header)
            if idx != -1 and (best_idx is None or idx < best_idx):
                best_idx, best_header = idx, header
        return best_idx, best_header

    def _is_valid_header(self, buf: bytearray, idx: int) -> bool:
        """Check if bytes at position idx form a valid frame header."""
        if idx + HEADER_LEN > len(buf):
            return False
        return bytes(buf[idx : idx + HEADER_LEN]) in HEADERS

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

            fsize = self._frame_size

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
            frames.append((header, self._parse_frame(raw)))

    # ------------------------------------------------------------------
    # Frame storage and callbacks
    # ------------------------------------------------------------------

    def _store_frame(self, header: bytes, frame: np.ndarray, ts: float) -> None:
        """Store the latest frame and timestamp for a sensor (thread-safe)."""
        with self._lock:
            self._frames[header][:] = frame
            self._timestamps[header] = ts

    def _dispatch(self, header: bytes, frame: np.ndarray, ts: float) -> None:
        """Store a frame and run the user callback for it."""
        self._store_frame(header, frame, ts)
        if self.on_frame is not None:
            try:
                self.on_frame(header, frame, ts)
            except Exception:
                logger.exception("Error in on_frame callback")

    # ------------------------------------------------------------------
    # Synchronous (blocking) reads
    # ------------------------------------------------------------------

    def read_frame(self) -> Tuple[bytes, np.ndarray]:
        """Block until one complete frame (from any sensor) is received.

        Returns ``(header, frame)`` where *header* is one of
        :data:`HEADER_A` ... :data:`HEADER_E` and *frame* is a
        ``numpy.ndarray``.

        Frames are returned in arrival order. Every frame in each chunk read
        from the port is kept, so none are lost while the caller keeps up;
        a caller that falls far behind loses the oldest frames first. Do
        **not** mix this with ``start()``; use one pattern or the other.
        """
        while not self._pending:
            chunk = self._ser.read(_READ_CHUNK)
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

    def get_frame(self, header: bytes) -> Optional[np.ndarray]:
        """Return the latest frame from a sensor (non-blocking).

        *header* is :data:`HEADER_A` ... :data:`HEADER_E`. Returns a copy of
        the most recent frame, or None if no frame received yet.
        Thread-safe - can be called from any thread while hub is running.
        """
        with self._lock:
            if self._timestamps[header] == 0:
                return None
            return self._frames[header].copy()

    # ------------------------------------------------------------------
    # Output mode (SensX25)
    # ------------------------------------------------------------------

    def get_output_mode(self) -> str:
        """Return the sensors' output mode: ``"adc"`` or ``"pressure"``.

        Streaming pauses for a moment while the sensors are asked. Raises
        RuntimeError if the sensors are not all in the same mode.
        """
        with self._paused():
            values = self._read_modes()
        if len(set(values)) != 1:
            raise RuntimeError(
                f"The sensors are not all in the same output mode "
                f"({[_MODES[v] for v in values]}); use set_output_mode()."
            )
        return _MODES[values[0]]

    def set_output_mode(self, mode: str) -> None:
        """Switch every sensor on the hub to ``"pressure"`` or ``"adc"``.

        ``"pressure"`` is the factory setting (kPa); ``"adc"`` gives ADC
        counts. One command reaches all sensors on the hub; they cannot be
        set individually. The setting is stored in the sensors and kept
        through power-off.

        The new mode is read back from every connected sensor before
        returning; RuntimeError if it cannot be confirmed. Streaming pauses
        meanwhile (usually about a second) and resumes on its own. Do not
        call this from a frame callback.
        """
        if mode not in _MODES:
            raise ValueError(f"mode must be 'adc' or 'pressure', got {mode!r}")
        target = _MODES.index(mode)
        with self._paused():
            self._transact(_mode_request(0x06, target))
            values = self._read_modes()
        if set(values) != {target}:
            raise RuntimeError(
                f"Output mode {mode!r} not confirmed on every sensor "
                f"({[_MODES[v] for v in values]})."
            )

    def _read_modes(self) -> List[int]:
        """Ask the sensors for their output mode; return one value per sensor.

        Every connected sensor answers, but the hub loses some replies on the
        way back, sometimes several reads in a row, so the question is
        repeated until one read has a reply from every connected sensor.
        """
        query = _mode_request(0x03, 1)
        for _ in range(_MAX_MODE_READS):
            rx = self._transact(query)
            ts = time.perf_counter()
            for header, frame in self._extract_frames(bytearray(rx)):
                self._store_frame(header, frame, ts)
            values = _mode_replies(rx)
            sensors = len(self.connected)
            if sensors and len(values) >= sensors:
                return values
        raise RuntimeError(
            f"No complete reply from the {len(self.connected)} connected sensors. "
            "Check the port and connections; output modes are supported on "
            "SensX25 (5x5) sensors."
        )

    def _transact(self, request: bytes) -> bytes:
        """Send a settings command and return everything received after it."""
        wait = self._last_write + _SETTLE_S - time.perf_counter()
        if wait > 0:
            time.sleep(wait)
        self._ser.reset_input_buffer()
        self._ser.write(request)
        self._ser.flush()
        if request[1] == 0x06:
            self._last_write = time.perf_counter()
        rx = bytearray()
        deadline = time.perf_counter() + _REPLY_WINDOW_S
        while time.perf_counter() < deadline:
            rx += self._ser.read(_READ_CHUNK)
        return bytes(rx)

    @contextmanager
    def _paused(self) -> Iterator[None]:
        """Stop the reader thread (if running) around a settings command."""
        was_running = self.is_running
        self.stop()
        try:
            yield
        finally:
            # Frames received before the command may be in the old mode.
            self._buf.clear()
            self._pending.clear()
            if was_running:
                self.start()

    # ------------------------------------------------------------------
    # Background reader loop
    # ------------------------------------------------------------------

    def _reader_loop(self) -> None:
        buf = bytearray()

        while not self._stop_event.is_set():
            try:
                chunk = self._ser.read(_READ_CHUNK)
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
