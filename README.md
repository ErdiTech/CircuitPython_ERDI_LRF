# CircuitPython_ERDI_LRF

CircuitPython driver for **ERDI 905 nm laser rangefinder modules** read over **UART** (`busio.UART`). Use it for a **laser distance sensor on CircuitPython boards** such as RP2040 (Raspberry Pi Pico, Feather RP2040), ESP32-S2/S3 and nRF52, and for desktop CPython tests. Module pages, datasheets and public prices are in the [ERDI laser rangefinder model list](https://erdilrf.com/models/?utm_source=github&utm_medium=readme&utm_campaign=circuitpython-erdi-lrf).

It supports both published ERDI protocol families, taken from the module manuals:

- **Stream modules** (460800 baud): LRF22VB, LRF50VB (distance in millimetres), LRF50H, LRF100H, LRF200H, LRF300H, LRF600H, LRF1500H (distance in centimetres). Read the newest value with `read_latest()`.
- **Single-shot modules** (115200 baud): LR1000E2 to LR2000E4, LRF1200A1, LRF3000A1, SPD1200N0/N2/N4. Request one reading with `measure()`.

The model table also holds each model's out-of-range marker, so the right unit and marker are applied automatically. Check the delivered UART interface and logic level of your module against the board before wiring (a UART level shifter may be needed).

## Install

Copy `erdi_lrf.py` to the `lib` folder of the CIRCUITPY drive. For a desktop test: `pip install circuitpython-erdi-lrf` is not published yet, use the file directly.

## Usage

```python
import board, busio, erdi_lrf

uart = busio.UART(board.TX, board.RX, baudrate=460800, timeout=0)
lrf = erdi_lrf.ErdiLRF(uart, "LRF22VB")
print(lrf.read_latest())      # metres, or None when out of range / no frame yet
```

Single-shot modules:

```python
uart = busio.UART(board.TX, board.RX, baudrate=115200, timeout=0)
lrf = erdi_lrf.ErdiLRF(uart, "LRF3000A1")
print(lrf.measure(timeout_ms=1000))   # metres, None when the module reports no echo, OSError on timeout
```

See [`examples/`](examples) for complete scripts.

## Tests and status

`python -m pytest tests` runs 13 protocol and reader tests on CPython with a fake UART (frame vectors from the module manuals, unit handling, out-of-range marker, checksum rejection, resync, single-shot request and reply, timeout). The frame parsing is the same code that runs in the [MicroPython driver](https://github.com/ErdiTech/erdi-lrf-esp32-micropython). The `busio.UART` wrapper has been tested only against the fake UART so far, not on a CircuitPython board.

## Related

[erdi-lrf-drivers](https://github.com/ErdiTech/erdi-lrf-drivers) (Python, Arduino, ROS 2), [erdi-lrf-esp32-micropython](https://github.com/ErdiTech/erdi-lrf-esp32-micropython), [erdi-lrf-home-assistant](https://github.com/ErdiTech/erdi-lrf-home-assistant).

## License

MIT, copyright 2026 ERDI TECH LTD. See [LICENSE](LICENSE).
