"""Stream sensor data and print the grid to the terminal.

Usage:
    python stream_example.py                          # defaults: /dev/ttyUSB0, 921600, 5x5
    python stream_example.py --rows 20 --cols 8       # SensX 160
    python stream_example.py --port COM3 --baud 15000000 --rows 16 --cols 12
    python stream_example.py --benchmark              # measure frame rate only
"""

import argparse
import sys
import time

from sensx import SensX

CURSOR_HOME = "\033[H"


def main() -> None:
    parser = argparse.ArgumentParser(description="Stream TouchTronix sensor data.")
    parser.add_argument(
        "--port", default="/dev/ttyUSB0", help="Serial port (default: /dev/ttyUSB0)"
    )
    parser.add_argument(
        "--baud", type=int, default=921600, help="Baud rate (default: 921600)"
    )
    parser.add_argument(
        "--rows", type=int, default=5, help="Number of rows (default: 5)"
    )
    parser.add_argument(
        "--cols", type=int, default=5, help="Number of columns (default: 5)"
    )
    parser.add_argument(
        "--benchmark",
        action="store_true",
        help="Measure frame rate without printing grid",
    )
    args = parser.parse_args()

    sensor = SensX(port=args.port, baud_rate=args.baud, rows=args.rows, cols=args.cols)

    # Pre-build static parts of the display
    col_header = "     " + "".join(f"{c:>6}" for c in range(sensor.cols))
    col_line = "     " + "------" * sensor.cols

    frame_count = 0
    t_start = time.perf_counter()

    if not args.benchmark:
        sys.stdout.write("\033[2J")

    try:
        while True:
            frame = sensor.read_frame()
            frame_count += 1

            elapsed = time.perf_counter() - t_start
            hz = frame_count / elapsed if elapsed > 0 else 0

            if args.benchmark:
                if frame_count % 100 == 0:
                    print(f"[{frame_count:>8}]  max={int(frame.max()):>5}  {hz:.1f} Hz")
            else:
                lines = [
                    CURSOR_HOME,
                    f"SensX  {sensor.rows}x{sensor.cols}  |  {hz:.1f} Hz  |  Ctrl+C to stop\n",
                    col_header,
                    col_line,
                ]
                for r in range(sensor.rows):
                    row_str = "".join(f"{frame[r, c]:>6}" for c in range(sensor.cols))
                    lines.append(f" {r:>2} |{row_str}")

                sys.stdout.write("\n".join(lines) + "\n")
                sys.stdout.flush()

    except KeyboardInterrupt:
        pass
    finally:
        sensor.close()
        elapsed = time.perf_counter() - t_start
        print(
            f"\nStopped. {frame_count} frames in {elapsed:.1f}s "
            f"({frame_count / elapsed:.1f} Hz)"
        )


if __name__ == "__main__":
    main()
