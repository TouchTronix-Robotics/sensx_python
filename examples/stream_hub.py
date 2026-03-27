"""Stream dual-sensor data from a TouchTronix Hub PCB.

Usage::

    python sensor_hub.py --port /dev/ttyUSB0 --baud 1500000 --rows 16 --cols 12
    python sensor_hub.py --benchmark
"""

import argparse
import sys
import time
from collections import deque
from typing import Optional, Tuple

import numpy as np

from sensx import SensXHub
from sensx.hub import HEADER_A

CURSOR_HOME = "\033[H"
CLEAR_LINE = "\033[K"
ROLLING_WINDOW_S = 2.0  # Calculate Hz over last 2 seconds


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

    # Rolling window for Hz calculation: deque of (timestamp, count_a, count_b)
    history: deque[Tuple[float, int, int]] = deque()

    if not args.benchmark:
        sys.stdout.write("\033[2J")

    def update_hz_history(now: float) -> Tuple[float, float]:
        """Calculate rolling Hz over last ROLLING_WINDOW_S seconds."""
        # Add current point
        history.append((now, frame_count_a, frame_count_b))
        # Remove old points outside window
        cutoff = now - ROLLING_WINDOW_S
        while history and history[0][0] < cutoff:
            history.popleft()
        # Need at least 2 points and window filled for accurate Hz
        if len(history) >= 2:
            t_old, a_old, b_old = history[0]
            t_new, a_new, b_new = history[-1]
            dt = t_new - t_old
            if dt > 0:
                hz_a = (a_new - a_old) / dt
                hz_b = (b_new - b_old) / dt
                return hz_a, hz_b
        # Fall back to cumulative Hz if window not ready
        elapsed = now - t_start
        if elapsed > 0:
            return frame_count_a / elapsed, frame_count_b / elapsed
        return 0.0, 0.0

    try:
        while True:
            header, frame = hub.read_frame()
            now = time.perf_counter()

            if header == HEADER_A:
                frame_count_a += 1
                last_a = frame
            else:
                frame_count_b += 1
                last_b = frame

            if args.benchmark:
                total = frame_count_a + frame_count_b
                if total % 100 == 0:
                    elapsed_total = now - t_start
                    hz_a_cum = frame_count_a / elapsed_total if elapsed_total > 0 else 0
                    hz_b_cum = frame_count_b / elapsed_total if elapsed_total > 0 else 0
                    print(
                        f"[{total:>8}]  A={frame_count_a} ({hz_a_cum:.1f} Hz)  "
                        f"B={frame_count_b} ({hz_b_cum:.1f} Hz)"
                    )
            elif now - last_display < display_interval:
                continue
            else:
                last_display = now
                hz_a, hz_b = update_hz_history(now)
                lines = [
                    CURSOR_HOME + CLEAR_LINE,
                    f"SensX Hub  {args.rows}x{args.cols}  |  "
                    f"A: {frame_count_a} ({hz_a:.1f} Hz)  "
                    f"B: {frame_count_b} ({hz_b:.1f} Hz)  |  Ctrl+C to stop" + CLEAR_LINE,
                    "",
                ]

                # --- Sensor A grid ---
                lines.append("  -- Sensor A (0xAAAA) --" + CLEAR_LINE)
                lines.append(col_header + CLEAR_LINE)
                lines.append(col_line + CLEAR_LINE)
                if last_a is not None:
                    for r in range(args.rows):
                        row_str = "".join(
                            f"{last_a[r, c]:>6}" for c in range(args.cols)
                        )
                        lines.append(f" {r:>2} |{row_str}" + CLEAR_LINE)
                else:
                    lines.append("     (waiting for data...)" + CLEAR_LINE)

                lines.append(CLEAR_LINE)

                # --- Sensor B grid ---
                lines.append("  -- Sensor B (0xBBBB) --" + CLEAR_LINE)
                lines.append(col_header + CLEAR_LINE)
                lines.append(col_line + CLEAR_LINE)
                if last_b is not None:
                    for r in range(args.rows):
                        row_str = "".join(
                            f"{last_b[r, c]:>6}" for c in range(args.cols)
                        )
                        lines.append(f" {r:>2} |{row_str}" + CLEAR_LINE)
                else:
                    lines.append("     (waiting for data...)" + CLEAR_LINE)

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
