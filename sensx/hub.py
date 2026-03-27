"""SensXHub -- streaming driver for a TouchTronix Hub PCB with two sensors.

The hub board multiplexes two SensX sensors on a single serial link.
Sensor-A frames use header ``0xAA 0xAA``, sensor-B frames use ``0xBB 0xBB``.

Hub frame format (NO CRC -- differs from single-sensor SensX)::

    [header: 2 bytes][payload: rows*cols*2 bytes, 12-bit big-endian]

Frames from sensors A and B alternate back-to-back on the wire.

Blocking read::

    from sensx import SensXHub

    hub = SensXHub(port="/dev/ttyUSB0")
    while True:
        frame_a, frame_b = hub.read_frames()
        if frame_a is not None:
            print("A max:", frame_a.max())
        if frame_b is not None:
            print("B max:", frame_b.max())

Callback::

    from sensx import SensXHub
    import time

    hub = SensXHub(port="/dev/ttyUSB0")
    hub.on_frame_a = lambda frame, ts: print("A", frame.max())
    hub.on_frame_b = lambda frame, ts: print("B", frame.max())
    hub.start()
    time.sleep(10)
    hub.stop()

Context manager::

    from sensx import SensXHub
    import time

    with SensXHub(port="/dev/ttyUSB0") as hub:
        hub.on_frame_a = lambda frame, ts: print("A", frame.max())
        hub.on_frame_b = lambda frame, ts: print("B", frame.max())
        hub.start()
        time.sleep(10)
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Callable, Optional, Tuple

import numpy as np
import serial


logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Protocol constants
# ---------------------------------------------------------------------------
HEADER_A = b"\xaa\xaa"
HEADER_B = b"\xbb\xbb"
HEADER_LEN = 2
BYTES_PER_PIXEL = 2


class SensXHub:
    """Driver for a TouchTronix Hub PCB hosting **two** SensX sensors.

    Parameters
    ----------
    port : str
        Serial port path (e.g. ``"/dev/ttyUSB0"``).
    baud_rate : int
        Baud rate (default 1 500 000).
    rows_a, cols_a : int
        Grid dimensions of sensor A (default 16 x 12).
    rows_b, cols_b : int
        Grid dimensions of sensor B (defaults to same as sensor A).
    serial_timeout : float
        Serial read timeout in seconds.
    read_chunk : int
        Bytes to read per ``serial.read()`` call.
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
    ) -> None:
        self.port = port
        self.baud_rate = baud_rate

        # Sensor A geometry
        self.rows_a = rows_a
        self.cols_a = cols_a
        self._payload_size_a = rows_a * cols_a * BYTES_PER_PIXEL
        self._frame_size_a = HEADER_LEN + self._payload_size_a

        # Sensor B geometry (defaults to same as A)
        self.rows_b = rows_b if rows_b is not None else rows_a
        self.cols_b = cols_b if cols_b is not None else cols_a
        self._payload_size_b = self.rows_b * self.cols_b * BYTES_PER_PIXEL
        self._frame_size_b = HEADER_LEN + self._payload_size_b

        self._read_chunk = read_chunk

        # Serial port (opened immediately so wiring errors surface early)
        self._ser = serial.Serial(port, baud_rate, timeout=serial_timeout)

        # Send init command (same as single-sensor driver)
        init_cmd = bytes.fromhex("01 06 60 10 00 01 57 CF")
        self._ser.write(init_cmd)
        self._ser.flush()

        # Latest frames (thread-safe)
        self._frame_a = np.zeros((self.rows_a, self.cols_a), dtype=np.uint16)
        self._frame_b = np.zeros((self.rows_b, self.cols_b), dtype=np.uint16)
        self._ts_a: float = 0.0
        self._ts_b: float = 0.0
        self._lock_a = threading.Lock()
        self._lock_b = threading.Lock()

        # Persistent read buffer
        self._buf = bytearray()

        # Background reader
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

        # User callbacks
        self.on_frame_a: Optional[Callable[[np.ndarray, float], None]] = None
        self.on_frame_b: Optional[Callable[[np.ndarray, float], None]] = None

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def latest_frame_a(self) -> np.ndarray:
        """Return a copy of the most recent sensor-A frame (thread-safe)."""
        with self._lock_a:
            return self._frame_a.copy()

    @property
    def latest_frame_b(self) -> np.ndarray:
        """Return a copy of the most recent sensor-B frame (thread-safe)."""
        with self._lock_b:
            return self._frame_b.copy()

    @property
    def latest_timestamp_a(self) -> float:
        """Timestamp (``time.perf_counter()``) of the most recent sensor-A frame."""
        with self._lock_a:
            return self._ts_a

    @property
    def latest_timestamp_b(self) -> float:
        """Timestamp (``time.perf_counter()``) of the most recent sensor-B frame."""
        with self._lock_b:
            return self._ts_b

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
        if header == HEADER_A:
            rows, cols = self.rows_a, self.cols_a
        else:
            rows, cols = self.rows_b, self.cols_b

        payload = raw[HEADER_LEN:]
        frame = np.frombuffer(payload, dtype=">u2").reshape(rows, cols)
        return (frame & 0x0FFF).astype(np.uint16)

    # ------------------------------------------------------------------
    # Buffer scanning
    # ------------------------------------------------------------------

    def _find_next_header(
        self, buf: bytearray, start: int = 0
    ) -> Tuple[Optional[int], Optional[bytes]]:
        """Find the earliest occurrence of ``HEADER_A`` or ``HEADER_B``.

        Returns ``(index, header_bytes)`` or ``(None, None)`` if neither
        is found.
        """
        idx_a = buf.find(HEADER_A, start)
        idx_b = buf.find(HEADER_B, start)

        if idx_a == -1 and idx_b == -1:
            return None, None
        if idx_a == -1:
            return idx_b, HEADER_B
        if idx_b == -1:
            return idx_a, HEADER_A
        if idx_a <= idx_b:
            return idx_a, HEADER_A
        return idx_b, HEADER_B

    def _frame_size_for(self, header: bytes) -> int:
        return self._frame_size_a if header == HEADER_A else self._frame_size_b

    def _is_valid_header(self, buf: bytearray, idx: int) -> bool:
        """Check if bytes at position idx form a valid frame header."""
        if idx + 2 > len(buf):
            return False
        return buf[idx:idx+2] in (HEADER_A, HEADER_B)

    def _store_frame(
        self, header: bytes, frame: np.ndarray, ts: float
    ) -> Optional[Callable[[np.ndarray, float], None]]:
        """Store frame to internal buffer and return the appropriate callback.

        Thread-safe storage for latest frame and timestamp.
        Returns the callback (on_frame_a or on_frame_b) to invoke, or None.
        """
        if header == HEADER_A:
            with self._lock_a:
                self._frame_a[:] = frame
                self._ts_a = ts
            return self.on_frame_a
        else:
            with self._lock_b:
                self._frame_b[:] = frame
                self._ts_b = ts
            return self.on_frame_b

    # ------------------------------------------------------------------
    # Synchronous (blocking) reads
    # ------------------------------------------------------------------

    def read_frame(self) -> Tuple[bytes, np.ndarray]:
        """Block until one complete frame (from either sensor) is received.

        Returns ``(header, frame)`` where *header* is :data:`HEADER_A` or
        :data:`HEADER_B` and *frame* is a ``numpy.ndarray``.

        Uses header validation: the next header must appear at the expected
        position (current_idx + frame_size) to confirm proper alignment.
        """
        buf = self._buf
        while True:
            idx, header = self._find_next_header(buf)
            if idx is not None and header is not None:
                fsize = self._frame_size_for(header)

                # Need enough data for: current frame + next header validation
                if len(buf) < idx + fsize + HEADER_LEN:
                    # Not enough data to validate next header
                    pass
                else:
                    # Validate: next bytes should be a valid header
                    next_header_idx = idx + fsize
                    if self._is_valid_header(buf, next_header_idx):
                        # Confirmed proper frame boundary
                        raw = bytes(buf[idx : idx + fsize])
                        del buf[: idx + fsize]
                        frame = self._parse_frame(header, raw)
                        ts = time.perf_counter()
                        self._store_frame(header, frame, ts)
                        return header, frame
                    else:
                        # False header found inside payload - skip it
                        del buf[: idx + 1]
                        continue

            # Not enough data -- read more
            chunk = self._ser.read(self._read_chunk)
            if chunk:
                buf += chunk
            else:
                # No data available (timeout) -- brief yield to prevent busy spin
                time.sleep(0.001)

            # Prevent unbounded growth
            max_frame = max(self._frame_size_a, self._frame_size_b)
            if len(buf) > max_frame * 8:
                buf[:] = buf[-(max_frame * 2) :]

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
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Block until one frame from each sensor has been received.

        Returns ``(frame_a, frame_b)``.  Either may be ``None`` if
        *timeout* expires first.
        """
        deadline = None if timeout is None else time.monotonic() + timeout
        got_a: Optional[np.ndarray] = None
        got_b: Optional[np.ndarray] = None

        while got_a is None or got_b is None:
            if deadline is not None and time.monotonic() > deadline:
                break
            header, frame = self.read_frame()
            if header == HEADER_A and got_a is None:
                got_a = frame
            elif header == HEADER_B and got_b is None:
                got_b = frame

        return got_a, got_b

    # ------------------------------------------------------------------
    # Background reader loop
    # ------------------------------------------------------------------

    def _reader_loop(self) -> None:
        buf = bytearray()
        max_frame = max(self._frame_size_a, self._frame_size_b)

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

            # Parse as many complete frames as possible
            while True:
                idx, header = self._find_next_header(buf)
                if idx is None or header is None:
                    # Keep buffer for partial header reassembly, don't aggressively trim.
                    # Unbounded growth prevention happens after this loop.
                    break

                fsize = self._frame_size_for(header)

                # Need enough data for current frame + next header validation
                if len(buf) < idx + fsize + HEADER_LEN:
                    if idx > 0:
                        del buf[:idx]
                    break

                # Validate: next bytes should be a valid header (or we've hit end of stream)
                next_header_idx = idx + fsize
                if not self._is_valid_header(buf, next_header_idx):
                    # False header found inside payload - skip it
                    del buf[: idx + 1]
                    continue

                # Confirmed proper frame boundary
                raw = bytes(buf[idx : idx + fsize])
                del buf[: idx + fsize]

                frame = self._parse_frame(header, raw)
                ts = time.perf_counter()
                cb = self._store_frame(header, frame, ts)

                if cb is not None:
                    try:
                        cb(frame, ts)
                    except Exception:
                        logger.exception(
                            "Error in %s callback",
                            "on_frame_a" if header == HEADER_A else "on_frame_b"
                        )

            # Prevent unbounded growth
            if len(buf) > max_frame * 8:
                buf[:] = buf[-(max_frame * 2) :]
