"""Stream dual-sensor data from a TouchTronix Hub PCB.

Usage::

    python sensor_hub.py --port /dev/ttyUSB0 --baud 1500000 --rows 16 --cols 12
    python sensor_hub.py --benchmark
"""

import argparse
import sys
import time
from typing import Optional

import numpy as np

from sensx import SensXHub
from sensx.hub import HEADER_A

CURSOR_HOME = "\033[H"


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Stream TouchTronix Hub dual-sensor data."
    )
    parser.add_argument(
        "--port",
        default="/dev/ttyUSB0",
        help="Serial port (default: /dev/ttyUSB0)",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=1_500_000,
        help="Baud rate (default: 1500000)",
    )
    parser.add_argument(
        "--rows",
        type=int,
        default=16,
        help="Number of rows per sensor (default: 16)",
    )
    parser.add_argument(
        "--cols",
        type=int,
        default=12,
        help="Number of columns per sensor (default: 12)",
    )
    parser.add_argument(
        "--benchmark",
        action="store_true",
        help="Measure frame rate without printing grids",
    )
    args = parser.parse_args()

    hub = SensXHub(
        port=args.port,
        baud_rate=args.baud,
        rows_a=args.rows,
        cols_a=args.cols,
        rows_b=args.rows,
        cols_b=args.cols,
    )

    col_header = "     " + "".join(f"{c:>6}" for c in range(args.cols))
    col_line = "     " + "------" * args.cols

    frame_count_a = 0
    frame_count_b = 0
    t_start = time.perf_counter()

    last_a: Optional[np.ndarray] = None
    last_b: Optional[np.ndarray] = None

    display_interval = 1.0 / 30  # throttle display to ~30 fps
    last_display = 0.0

    if not args.benchmark:
        sys.stdout.write("\033[2J")

    try:
        while True:
            header, frame = hub.read_frame()

            if header == HEADER_A:
                frame_count_a += 1
                last_a = frame
            else:
                frame_count_b += 1
                last_b = frame

            elapsed = time.perf_counter() - t_start
            total = frame_count_a + frame_count_b
            hz_a = frame_count_a / elapsed if elapsed > 0 else 0
            hz_b = frame_count_b / elapsed if elapsed > 0 else 0

            if args.benchmark:
                if total % 100 == 0:
                    print(
                        f"[{total:>8}]  A={frame_count_a} ({hz_a:.1f} Hz)  "
                        f"B={frame_count_b} ({hz_b:.1f} Hz)"
                    )
            elif elapsed - last_display < display_interval:
                continue
            else:
                last_display = elapsed
                lines = [
                    CURSOR_HOME,
                    f"SensX Hub  {args.rows}x{args.cols}  |  "
                    f"A: {frame_count_a} ({hz_a:.1f} Hz)  "
                    f"B: {frame_count_b} ({hz_b:.1f} Hz)  |  Ctrl+C to stop\n",
                ]

                # --- Sensor A grid ---
                lines.append("  -- Sensor A (0xAAAA) --")
                lines.append(col_header)
                lines.append(col_line)
                if last_a is not None:
                    for r in range(args.rows):
                        row_str = "".join(
                            f"{last_a[r, c]:>6}" for c in range(args.cols)
                        )
                        lines.append(f" {r:>2} |{row_str}")
                else:
                    lines.append("     (waiting for data...)")

                lines.append("")

                # --- Sensor B grid ---
                lines.append("  -- Sensor B (0xBBBB) --")
                lines.append(col_header)
                lines.append(col_line)
                if last_b is not None:
                    for r in range(args.rows):
                        row_str = "".join(
                            f"{last_b[r, c]:>6}" for c in range(args.cols)
                        )
                        lines.append(f" {r:>2} |{row_str}")
                else:
                    lines.append("     (waiting for data...)")

                sys.stdout.write("\n".join(lines) + "\n")
                sys.stdout.flush()

    except KeyboardInterrupt:
        pass
    finally:
        hub.close()
        elapsed = time.perf_counter() - t_start
        total = frame_count_a + frame_count_b
        hz_a = frame_count_a / elapsed if elapsed > 0 else 0
        hz_b = frame_count_b / elapsed if elapsed > 0 else 0
        print(
            f"\nStopped. {total} frames in {elapsed:.1f}s  "
            f"A={frame_count_a} ({hz_a:.1f} Hz)  "
            f"B={frame_count_b} ({hz_b:.1f} Hz)"
        )


if __name__ == "__main__":
    main()
