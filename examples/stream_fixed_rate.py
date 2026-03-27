#!/usr/bin/env python3
"""
Fixed-rate streaming example for SensX Hub.

Samples both sensors at a consistent target rate (e.g., 60 Hz) regardless of
raw frame arrival rates. Useful when firmware sends frames at mismatched
cadences (e.g., 20x8 mode where one sensor sends 2x as many frames).

Usage:
    python stream_fixed_rate.py --port /dev/ttyUSB0 --baud 921600 \\
        --rows 20 --cols 8 --rate 60
"""
from __future__ import annotations

import argparse
import sys
import time

import numpy as np

from sensx import HEADER_A, HEADER_B, SensXHub


def format_grid(frame: np.ndarray, title: str, header: bytes) -> str:
    """Format a frame as an aligned grid string."""
    rows, cols = frame.shape
    sensor = "A" if header == HEADER_A else "B"
    hex_hdr = header.hex()

    lines = [
        f"\n  -- Sensor {sensor} (0x{hex_hdr}) --",
        "         " + " ".join(f"{c:>5}" for c in range(cols)),
        "     " + "-" * (6 * cols + 3),
    ]
    for r in range(rows):
        row_vals = " ".join(f"{frame[r, c]:>5}" for c in range(cols))
        lines.append(f"{r:>2} | {row_vals}")
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(
        description="Stream SensX Hub data at a fixed sampling rate"
    )
    parser.add_argument("--port", required=True, help="Serial port (e.g., /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=921600, help="Baud rate")
    parser.add_argument("--rows", type=int, default=20, help="Grid rows (both sensors)")
    parser.add_argument("--cols", type=int, default=8, help="Grid columns (both sensors)")
    parser.add_argument("--rate", type=float, default=60.0, help="Target sampling rate in Hz (default: 60)")
    parser.add_argument("--timeout", type=float, default=0.5, help="Frame timeout in seconds")
    parser.add_argument(
        "--benchmark", action="store_true", help="Benchmark mode (no display, just stats)"
    )
    args = parser.parse_args()

    hub = SensXHub(
        port=args.port,
        baud_rate=args.baud,
        rows_a=args.rows,
        cols_a=args.cols,
        rows_b=args.rows,
        cols_b=args.cols,
        serial_timeout=0.01,
    )
    hub.start()

    period = 1.0 / args.rate
    frame_count = 0
    start_time = time.perf_counter()

    # ANSI clear codes
    CURSOR_HOME = "\x1b[H"
    CLEAR_SCREEN = "\x1b[2J"
    CLEAR_LINE = "\x1b[K"

    print(CLEAR_SCREEN + CURSOR_HOME, end="")
    print(f"SensX Fixed-Rate Stream  {args.rows}x{args.cols}  |  Target: {args.rate:.1f} Hz")
    print("Starting...")

    try:
        while True:
            loop_start = time.perf_counter()

            # Sample latest frames (non-blocking)
            frame_a = hub.get_frame_a()
            frame_b = hub.get_frame_b()

            # Only count samples when we have both sensors (fixed-rate behavior)
            if frame_a is not None and frame_b is not None:
                frame_count += 1

                # Render every sample (matching sample rate)
                if not args.benchmark:
                    now = time.perf_counter()
                    elapsed = now - start_time
                    actual_hz = frame_count / elapsed if elapsed > 0 else 0
                    jitter = abs(actual_hz - args.rate)

                    # Full screen clear + redraw for clean output
                    lines = [CLEAR_SCREEN + CURSOR_HOME]
                    lines.append(
                        f"SensX Fixed-Rate  {args.rows}x{args.cols}  |  "
                        f"Samples: {frame_count} ({actual_hz:.1f} Hz, jitter={jitter:.1f})  |  "
                        f"Ctrl+C to stop"
                    )

                    if frame_a is not None:
                        lines.append(format_grid(frame_a, "Sensor A", HEADER_A))
                    else:
                        lines.append("\n  -- Sensor A: No data --")

                    if frame_b is not None:
                        lines.append(format_grid(frame_b, "Sensor B", HEADER_B))
                    else:
                        lines.append("\n  -- Sensor B: No data --")

                    sys.stdout.write("\n".join(lines))
                    sys.stdout.flush()

            # Maintain fixed sampling rate
            elapsed = time.perf_counter() - loop_start
            sleep_time = max(0, period - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        pass
    finally:
        hub.stop()
        total_time = time.perf_counter() - start_time
        actual_hz = frame_count / total_time if total_time > 0 else 0
        print(f"\nStopped. {frame_count} samples in {total_time:.1f}s ({actual_hz:.1f} Hz avg)")


if __name__ == "__main__":
    main()
