"""Minimal example: stream sensor data and print the grid to the terminal."""

import os
import time

from sensx import SensX


def main() -> None:
    sensor = SensX(port="/dev/ttyUSB0", baud_rate=921600, rows=20, cols=8)

    frame_count = 0
    t_start = time.perf_counter()

    print(f"SensX  {sensor.rows}x{sensor.cols}  |  Ctrl+C to stop\n")

    try:
        while True:
            frame = sensor.read_frame()
            frame_count += 1

            elapsed = time.perf_counter() - t_start
            hz = frame_count / elapsed if elapsed > 0 else 0

            # Clear terminal
            os.system("clear")

            print(
                f"SensX  {sensor.rows}x{sensor.cols}  |  {hz:.1f} Hz  |  Ctrl+C to stop\n"
            )

            # Column header
            print("     " + "".join(f"{c:>6}" for c in range(sensor.cols)))
            print("     " + "------" * sensor.cols)

            # Sensor grid
            for r in range(sensor.rows):
                row_str = "".join(f"{frame[r, c]:>6}" for c in range(sensor.cols))
                print(f" {r:>2} |{row_str}")

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
