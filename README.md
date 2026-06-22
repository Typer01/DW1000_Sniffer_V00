ESP-IDF Based UWB Sniffer utilizing the DW1000 chip. Intended for use on a makerfabs UWB Pro with Display module

## Projects

This repo contains two independent ESP-IDF projects, each with its own `build/` directory and its own `idf.py` lifecycle:

- **Receiver** (repo root) — sweeps a set of DW1000 configs listening for preambles/SFDs/frames. See [main/main.c](main/main.c).
- **Transmitter** ([transmitter/](transmitter/)) — continuously transmits a known fixed frame on a single config, used to validate the receiver against a real over-the-air signal. See [transmitter/main/main.c](transmitter/main/main.c).

Both share the DW1000 driver components in [components/](components/) (`deca_spi`, `deca_gpio`, `decadriver`); the transmitter project pulls them in via `EXTRA_COMPONENT_DIRS` in [transmitter/CMakeLists.txt](transmitter/CMakeLists.txt) instead of duplicating them.

## Build / Flash / Monitor

Each project is built and flashed from its own directory — `idf.py` always acts on whichever project's `build/` folder is in your current working directory, so there's no shared state between the two.

Find the connected board's serial port first:
```
ls /dev/cu.usbserial-*
```

### Receiver (repo root)
```
idf.py set-target esp32          # first time only
idf.py build
idf.py -p /dev/cu.usbserial-XXXX flash monitor
```

### Transmitter (`transmitter/`)
```
cd transmitter
idf.py set-target esp32          # first time only
idf.py build
idf.py -p /dev/cu.usbserial-XXXX flash monitor
cd ..
```

If you have both boards connected at once, `ls /dev/cu.usbserial-*` will list both ports — match them up (e.g. by unplugging one and re-running `ls`) before flashing, since flashing the wrong port will overwrite the wrong board's firmware.

`monitor` alone (without `flash`) reattaches to a board that's already running, without reflashing it. Press `Ctrl+]` to exit. Only one process can hold a given serial port open at a time — close any other open monitor/terminal session on that port first, or you'll see `device reports readiness to read but returned no data` / "multiple access" errors.


