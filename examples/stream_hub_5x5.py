"""Stream five SensX25 (5x5) sensors from one TouchTronix hub.

WHAT IT DOES
    Connects to the hub, reports the sensors' output mode (pressure or ADC),
    optionally switches it, and then either shows all five sensors live,
    logs every frame to a CSV file, or prints a short summary per sensor.

REQUIREMENTS
    Python 3.8 or newer and this package. From the repository folder:
        pip install -e .
    (this also installs pyserial and numpy)

FINDING THE PORT
    Windows: Device Manager > Ports (COM & LPT). The adapter shows as
             something like "USB-SERIAL CH340 (COM7)" or "USB Serial Port (COM7)".
             Use the COM number, e.g. --port COM7
    Linux:   usually /dev/ttyUSB0. If you get "permission denied", run
             sudo chmod a+rw /dev/ttyUSB0  (or add your user to the dialout group).
    Only one program can use the port at a time; close any other serial tool first.

HOW TO RUN
    Live view of all five sensors (Ctrl+C to stop):
        python examples/stream_hub_5x5.py --port COM7

    Live view AND save every frame to a CSV file (Ctrl+C stops and saves):
        python examples/stream_hub_5x5.py --port COM7 --csv recording_01.csv

    Record for 10 seconds, then print a summary per sensor (nothing is saved):
        python examples/stream_hub_5x5.py --port COM7 --check 10

    Switch the output mode first, then stream as above:
        python examples/stream_hub_5x5.py --port COM7 --mode adc
        python examples/stream_hub_5x5.py --port COM7 --mode pressure

    An existing CSV with the same name is overwritten, so use a new name per recording.

WHICH SENSOR IS WHICH
    Each channel letter belongs to a connector on the hub, not to a sensor.
    Seen from the top, with the PC cable connector on the right:

                  +----------------------+
             A ---|                      |
                  |         HUB          |--- PC
             B ---|                      |
                  +----------------------+
                       |      |      |
                       C      E      D

    Note that the bottom row reads C, E, D from left to right. A connector
    with no sensor sends nothing, and the other letters stay the same.
    To check, start the live view and press one sensor at a time.

OUTPUT MODES
    pressure  Factory setting. Values are pressure in kPa, from 0 to 534;
              534 is the top of the measuring range.
    adc       Values are ADC counts, from 0 to 3945: the sensor's 12-bit ADC
              reading minus a fixed offset of 150, so an unloaded sensor reads
              0 and full scale is 4095 - 150 = 3945.
    The mode is stored in the sensors and kept through power-off, so it only
    needs to be set once. Through the hub one command sets all five sensors;
    they cannot be set individually. A sensor that is not connected when the
    mode is set keeps its old mode, so set it again after adding a sensor.

CSV FILE
    One row per frame, in the order the frames arrived:
        time_s, sensor, mode, p0, p1, ..., p24
    time_s   seconds since the recording started, taken when the PC received the
             frame. USB serial adapters hand data over in small bursts, so several
             rows can share nearly the same time. Row order is the true order.
    sensor   hub channel A, B, C, D or E (frame header AA AA, BB BB, ... EE EE).
    mode     output mode at the start of the recording: pressure or adc.
    p0..p24  the 25 pixel values exactly as the hub sent them (no scaling, no
             filtering), in row order: pixel number = row * 5 + column,
             row 0 / column 0 first.

SUMMARY TABLE (--check)
    Frames   frames received from that channel during the recording
    Hz       frames per second for that channel
    Floor    the highest value that some pixel never dropped below. 0 means every
             pixel returned to zero at some point; a larger number means a pixel
             stayed loaded (or stuck) for the whole recording.
    Peak     the largest value seen on any pixel
    Changed  how many of the 25 pixels changed by more than CHANGE_THRESHOLD
    It then prints each channel's 5x5 map of the peak value per pixel.

DATA FORMAT ON THE WIRE (for reference)
    Serial 921600 baud, 8 data bits, no parity, 1 stop bit.
    The hub sends frames from all sensors back to back, about 100 per second
    per sensor. The order of the channels varies, so the header tells them apart:
        2-byte header (AA AA, BB BB, CC CC, DD DD or EE EE)
        25 pixel values, each a 16-bit unsigned integer, big-endian (50 bytes)
    52 bytes per frame, no checksum.
"""

import argparse
import sys
import time

import numpy as np
import serial

from sensx import SensXHub

ROWS, COLS = 5, 5
SENSOR_LETTERS = "ABCDE"                  # hub channels, one per connector
DEFAULT_BAUD = 921600
UNITS = {"pressure": "kPa", "adc": "ADC counts", "unknown": "unknown units"}
CHANGE_THRESHOLD = 20                     # used by the "Changed" column of --check
NO_DATA_WARNING_S = 3.0                   # warn if nothing arrives for this long
SCREEN_REFRESH_HZ = 20                    # live view redraw rate (all frames are still logged)


def open_hub(port, baud):
    """Open the hub, or exit with a readable message."""
    try:
        return SensXHub(port, baud_rate=baud, rows_a=ROWS, cols_a=COLS)
    except serial.SerialException as e:
        sys.exit(f"Could not open {port}: {e}\n"
                 "Check the port name, that the hub is plugged in, and that no other "
                 "program is using it.")


def prepare_mode(hub, wanted):
    """Switch the output mode if asked, then return the mode in use."""
    try:
        if wanted:
            print(f"Setting all sensors to {wanted} output...")
            hub.set_output_mode(wanted)
            return wanted
        return hub.get_output_mode()
    except RuntimeError as e:
        if wanted:
            hub.close()
            sys.exit(f"Output mode not changed: {e}")
        print(f"Could not read the output mode ({e}); streaming anyway.")
        return "unknown"


def warn_if_silent(hub, t0, warned):
    """Print a hint once if no frame has arrived for NO_DATA_WARNING_S."""
    last = max([hub.get_timestamp(ch) for ch in SENSOR_LETTERS] + [t0])
    silent = time.perf_counter() - last > NO_DATA_WARNING_S
    if silent and not warned:
        sys.stderr.write(
            f"\nNo frames for {NO_DATA_WARNING_S:g} s. Check that the hub is powered, "
            "the port number is right, and no other program has the port open.\n")
    return warned or silent


def check(hub, seconds, mode):
    """Collect for `seconds`, then print a summary table and peak maps."""
    count = {ch: 0 for ch in SENSOR_LETTERS}
    low = {ch: np.full((ROWS, COLS), 65535, np.uint16) for ch in SENSOR_LETTERS}
    high = {ch: np.zeros((ROWS, COLS), np.uint16) for ch in SENSOR_LETTERS}

    def on_frame(ch, frame, ts):
        count[ch] += 1
        np.minimum(low[ch], frame, out=low[ch])
        np.maximum(high[ch], frame, out=high[ch])

    hub.on_frame = on_frame
    print(f"Recording {seconds:g} s - press the sensors now (Ctrl+C ends early)...")
    t0 = time.perf_counter()
    hub.start()
    warned = False
    try:
        while time.perf_counter() - t0 < seconds:
            time.sleep(0.1)
            warned = warn_if_silent(hub, t0, warned)
    except KeyboardInterrupt:
        print("\nStopped early.")
    hub.stop()
    elapsed = max(time.perf_counter() - t0, 1e-9)

    print(f"\nOutput mode: {mode} ({UNITS[mode]})")
    print(f"\n{'Channel':<9}{'Frames':>8}{'Hz':>8}{'Floor':>8}{'Peak':>8}{'Changed':>10}")
    for ch in SENSOR_LETTERS:
        if count[ch] == 0:
            print(f"{ch:<9}{0:>8}   no frames received")
            continue
        changed = int(((high[ch].astype(int) - low[ch].astype(int)) > CHANGE_THRESHOLD).sum())
        print(f"{ch:<9}{count[ch]:>8}{count[ch] / elapsed:>8.1f}{int(low[ch].max()):>8}"
              f"{int(high[ch].max()):>8}{changed:>7}/25")
    for ch in SENSOR_LETTERS:
        if count[ch]:
            print(f"\nChannel {ch}: peak value per pixel (row 0 at the top)")
            for r in range(ROWS):
                print("  " + "".join(f"{int(v):>7}" for v in high[ch][r]))


def draw(hub, count, elapsed, mode, csv_path, rows_logged):
    """Redraw the live view: channels A-C on top, D-E below."""
    out = ["\033[H"]                                  # cursor to top-left
    rates = "   ".join(f"{ch}: {count[ch] / elapsed:6.1f} Hz" for ch in SENSOR_LETTERS)
    out.append(f"{rates}   |   Ctrl+C to stop\033[K")
    out.append(f"Output mode: {mode} ({UNITS[mode]})   |   " +
               (f"Saving to {csv_path}  ({rows_logged} rows)" if csv_path
                else "Not saving (add --csv FILE to record)") + "\033[K")
    for group in ("ABC", "DE"):
        out.append("\033[K")
        out.append("".join(f"  Channel {ch}".ljust(32) for ch in group) + "\033[K")
        frames = {ch: hub.get_frame(ch) for ch in group}
        for r in range(ROWS):
            line = ""
            for ch in group:
                f = frames[ch]
                cells = ("".join(f"{int(v):>6}" for v in f[r]) if f is not None
                         else "   (no data yet)".ljust(30))
                line += cells + "  "
            out.append(line + "\033[K")
    sys.stdout.write("\n".join(out) + "\n")
    sys.stdout.flush()


def live(hub, csv_path, mode):
    """Show all channels live; if csv_path is given, log every frame to it."""
    log = open(csv_path, "w", newline="") if csv_path else None
    if log:
        log.write("time_s,sensor,mode," + ",".join(f"p{i}" for i in range(ROWS * COLS)) + "\n")
    count = {ch: 0 for ch in SENSOR_LETTERS}
    t0 = time.perf_counter()

    def on_frame(ch, frame, ts):
        count[ch] += 1
        if log:
            log.write(f"{ts - t0:.4f},{ch},{mode}," +
                      ",".join(str(int(v)) for v in frame.ravel()) + "\n")

    hub.on_frame = on_frame
    sys.stdout.write("\033[2J")                       # clear the screen once
    hub.start()
    warned = False
    last_flush = t0
    try:
        while True:
            time.sleep(1.0 / SCREEN_REFRESH_HZ)
            now = time.perf_counter()
            draw(hub, count, max(now - t0, 1e-9), mode, csv_path, sum(count.values()))
            warned = warn_if_silent(hub, t0, warned)
            if log and now - last_flush >= 1.0:       # keep the file current on disk
                log.flush()
                last_flush = now
    except KeyboardInterrupt:
        pass
    finally:
        hub.stop()
        if log:
            log.close()
            print(f"\nSaved {sum(count.values())} rows to {csv_path}")


def main():
    ap = argparse.ArgumentParser(
        description="Stream five SensX25 (5x5) sensors from a TouchTronix hub: live view, "
                    "CSV recording, or a short per-channel summary.",
        epilog="Example: python examples/stream_hub_5x5.py --port COM7 --csv recording_01.csv")
    ap.add_argument("--port", required=True,
                    help="serial port of the hub, e.g. COM7 (Windows) or /dev/ttyUSB0 (Linux)")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD,
                    help=f"baud rate (default {DEFAULT_BAUD}, the hub's rate)")
    ap.add_argument("--mode", choices=["pressure", "adc"],
                    help="switch all five sensors to this output mode first (stored in the sensors)")
    ap.add_argument("--csv", metavar="FILE",
                    help="save every frame to this CSV file (live view keeps running; Ctrl+C stops and saves)")
    ap.add_argument("--check", type=float, metavar="SECONDS",
                    help="record for SECONDS, then print a summary per channel instead of the live view")
    args = ap.parse_args()

    hub = open_hub(args.port, args.baud)
    with hub:
        mode = prepare_mode(hub, args.mode)
        print(f"Output mode: {mode} ({UNITS[mode]})")
        if args.check:
            check(hub, args.check, mode)
        else:
            live(hub, args.csv, mode)


if __name__ == "__main__":
    main()
