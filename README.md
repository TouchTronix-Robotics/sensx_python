# SensX Python API

Python driver for TouchTronix tactile sensors.

## Install

```bash
cd sensx_python
pip install --upgrade pip setuptools wheel
pip install -e .
```

### Serial Permissions (Linux)

```bash
sudo chmod a+rw /dev/ttyUSB0
```

### Run Example

```bash
# Print sensor grid (default: 5x5)
python examples/stream_example.py

# Specify sensor size
python examples/stream_example.py --rows 20 --cols 8

# Measure frame rate
python examples/stream_example.py --benchmark

# All options
python examples/stream_example.py --port /dev/ttyUSB0 --baud 921600 --rows 5 --cols 5
```

### Supported Sensors

| Model     | Grid  | Part Number        | Example Command |
|-----------|-------|--------------------|-----------------|
| SensX 25  | 5x5   | SNX0505-SNS-01     | `python examples/stream_example.py --port /dev/ttyUSB0 --baud 921600 --rows 5 --cols 5` |
| SensX 160 | 20x8  | SNX2006-SNS-01     | `python examples/stream_example.py --port /dev/ttyUSB0 --baud 921600 --rows 20 --cols 8` |
| SensX 192 | 16x12 | SNX1216-SNS-01     | `python examples/stream_example.py --port /dev/ttyUSB0 --baud 15000000 --rows 16 --cols 12` * |

\* SensX 192 baud rate is unverified — may be 921600 instead of 15000000.

## Quick Start

Blocking read:

```python
from sensx import SensX

sensor = SensX(port="/dev/ttyUSB0")
while True:
    frame = sensor.read_frame()
    print(frame)
```

Callback:

```python
from sensx import SensX

sensor = SensX(port="/dev/ttyUSB0")
sensor.on_frame = lambda frame, ts: print(frame.max())
sensor.start()
```

Callback with context manager:

```python
from sensx import SensX
import time

with SensX(port="/dev/ttyUSB0") as sensor:
    sensor.on_frame = lambda frame, ts: print(frame.max())
    sensor.start()
    time.sleep(5)
```

## API

### `SensX(port, baud_rate=15_000_000, rows=16, cols=12)`

| Method / Property     | Description                                      |
|-----------------------|--------------------------------------------------|
| `start()`             | Start background reader thread                   |
| `stop()`              | Stop the reader thread                           |
| `close()`             | Stop and close the serial port                   |
| `read_frame()`        | Blocking read -- returns one frame (no threading)|
| `latest_frame`        | Thread-safe copy of the most recent frame        |
| `latest_timestamp`    | `time.perf_counter()` of the most recent frame   |
| `on_frame`            | Callback: `fn(frame: np.ndarray, ts: float)`     |

## Troubleshooting

### `build_editable` error on `pip install -e .`

```
ERROR: Project ... has a 'pyproject.toml' and its build backend is missing the 'build_editable' hook.
```

Your `setuptools` is too old. Editable installs require **setuptools >= 64.0** (PEP 660). Fix:

```bash
pip install --upgrade pip setuptools wheel
pip install -e .
```

If you still can't use editable mode, a regular install also works:

```bash
pip install .
```

### `ModuleNotFoundError: No module named 'sensx'`

The install step above likely failed. Check the install output and retry.

### `command 'python' not found`

On Ubuntu 22.04+, use `python3` instead of `python`, or install the compatibility package:

```bash
sudo apt install python-is-python3
```
