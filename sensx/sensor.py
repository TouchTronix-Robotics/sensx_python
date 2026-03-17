"""SensX -- streaming driver for TouchTronix tactile sensors."""

from __future__ import annotations

import threading
import time
from typing import Callable, Optional

import numpy as np
import serial


def _crc8_maxim(data: bytes) -> int:
    """Compute CRC8/Maxim (DOW) checksum."""
    crc = 0x00
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x01:
                crc = (crc >> 1) ^ 0x8C
            else:
                crc >>= 1
            crc &= 0xFF
    return crc


class SensX:
    """Read frames from a TouchTronix tactile sensor over serial.

    The sensor continuously streams frames.  Each frame is:

        [0xFF][0xFF][payload: rows * cols * 2 bytes, 12-bit big-endian][crc: 1 byte]

    Payload values are 12-bit unsigned integers stored in big-endian
    uint16 words (upper 4 bits masked off).  The trailing byte is a
    CRC8-Maxim checksum over the payload.

    Blocking read::

        sensor = SensX("/dev/ttyUSB0")
        while True:
            frame = sensor.read_frame()
            print(frame)

    Callback::

        sensor = SensX("/dev/ttyUSB0")
        sensor.on_frame = lambda frame, ts: print(frame.max())
        sensor.start()

    Callback with context manager::

        with SensX("/dev/ttyUSB0") as sensor:
            sensor.on_frame = lambda frame, ts: print(frame.max())
            sensor.start()
            time.sleep(5)

    Parameters
    ----------
    port : str
        Serial port path (e.g. ``"/dev/ttyUSB0"``).
    baud_rate : int
        Baud rate (default 15 000 000).
    rows, cols : int
        Grid dimensions (default 16 x 12).
    serial_timeout : float
        Serial read timeout in seconds.
    read_chunk : int
        Bytes to read per ``serial.read()`` call.
    init_cmd : bytes or None
        Command sent on connection to start streaming.  Defaults to the
        standard Modbus trigger.  Pass ``None`` to skip.
    """

    HEADER = b"\xff\xff"
    HEADER_LEN = 2
    BYTES_PER_PIXEL = 2
    CRC_LEN = 1

    #: Default Modbus init command that triggers continuous streaming.
    DEFAULT_INIT_CMD = bytes.fromhex("01 06 60 10 00 01 57 CF")

    def __init__(
        self,
        port: str,
        baud_rate: int = 15_000_000,
        rows: int = 16,
        cols: int = 12,
        serial_timeout: float = 0.005,
        read_chunk: int = 4096,
        init_cmd: Optional[bytes] = DEFAULT_INIT_CMD,
    ) -> None:
        self.port = port
        self.baud_rate = baud_rate
        self.rows = rows
        self.cols = cols

        self._payload_size = rows * cols * self.BYTES_PER_PIXEL
        self._frame_size = self.HEADER_LEN + self._payload_size + self.CRC_LEN
        self._read_chunk = read_chunk

        # Serial port (opened immediately so wiring errors surface early)
        self._ser = serial.Serial(port, baud_rate, timeout=serial_timeout)

        # Send init command to start streaming (if provided)
        if init_cmd is not None:
            self._ser.write(init_cmd)
            self._ser.flush()

        # Latest frame (thread-safe access via property)
        self._frame = np.zeros((rows, cols), dtype=np.uint16)
        self._timestamp: float = 0.0
        self._lock = threading.Lock()

        # Persistent read buffer (shared by read_frame and _reader_loop)
        self._buf = bytearray()

        # Background reader
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

        # User callback: on_frame(frame: np.ndarray, timestamp: float) -> None
        self.on_frame: Optional[Callable[[np.ndarray, float], None]] = None

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def latest_frame(self) -> np.ndarray:
        """Return a copy of the most recent frame (thread-safe)."""
        with self._lock:
            return self._frame.copy()

    @property
    def latest_timestamp(self) -> float:
        """Timestamp (``time.perf_counter()``) of the most recent frame."""
        with self._lock:
            return self._timestamp

    @property
    def is_running(self) -> bool:
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
            target=self._reader_loop, name="SensX-Reader", daemon=True
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

    def __enter__(self) -> "SensX":
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # ------------------------------------------------------------------
    # Synchronous (blocking) read
    # ------------------------------------------------------------------

    def _parse_frame(self, raw: bytes) -> Optional[np.ndarray]:
        """Validate CRC and parse raw frame bytes into a numpy array.

        Returns None if CRC check fails.
        """
        payload = raw[self.HEADER_LEN : self.HEADER_LEN + self._payload_size]
        crc_byte = raw[-1]
        if _crc8_maxim(payload) != crc_byte:
            return None
        frame = np.frombuffer(payload, dtype=">u2").reshape(self.rows, self.cols)
        return (frame & 0x0FFF).astype(np.uint16)

    def read_frame(self) -> np.ndarray:
        """Block until one complete frame is received and return it.

        This is an alternative to the callback approach -- useful for
        simple scripts that just want to poll.  Do **not** mix this with
        ``start()`` / ``on_frame``; use one pattern or the other.
        """
        buf = self._buf
        while True:
            # Check buffer first before reading more data
            idx = buf.find(self.HEADER)
            if idx != -1 and len(buf) >= idx + self._frame_size:
                raw = bytes(buf[idx : idx + self._frame_size])
                del buf[: idx + self._frame_size]
                frame = self._parse_frame(raw)
                if frame is None:
                    continue  # CRC failed, try next frame
                with self._lock:
                    self._frame[:] = frame
                    self._timestamp = time.perf_counter()
                return frame

            # Not enough data — read more
            chunk = self._ser.read(self._read_chunk)
            if chunk:
                buf += chunk

            # Prevent unbounded growth
            if len(buf) > self._frame_size * 8:
                buf = buf[-(self._frame_size * 2) :]

    # ------------------------------------------------------------------
    # Internal reader loop
    # ------------------------------------------------------------------

    def _reader_loop(self) -> None:
        buf = bytearray()
        while not self._stop_event.is_set():
            try:
                chunk = self._ser.read(self._read_chunk)
            except serial.SerialException:
                break

            if not chunk:
                continue

            buf += chunk

            # Parse as many complete frames as possible from the buffer
            while True:
                idx = buf.find(self.HEADER)
                if idx == -1:
                    # No header found -- keep only a small tail that might
                    # contain the start of a header byte.
                    if len(buf) > self.HEADER_LEN:
                        buf = buf[-(self.HEADER_LEN - 1) :]
                    break

                if len(buf) < idx + self._frame_size:
                    # Header found but frame incomplete -- discard bytes
                    # before the header and wait for more data.
                    if idx > 0:
                        del buf[:idx]
                    break

                # Full frame available
                raw = bytes(buf[idx : idx + self._frame_size])
                del buf[: idx + self._frame_size]

                frame = self._parse_frame(raw)
                if frame is None:
                    continue  # CRC failed, skip

                ts = time.perf_counter()

                with self._lock:
                    self._frame[:] = frame
                    self._timestamp = ts

                cb = self.on_frame
                if cb is not None:
                    try:
                        cb(frame, ts)
                    except Exception:
                        pass  # don't let user errors kill the reader

            # Prevent unbounded growth if no header is ever found
            if len(buf) > self._frame_size * 8:
                buf = buf[-(self._frame_size * 2) :]
