ESP-IDF Based UWB Sniffer utilizing the DW1000 chip. Intended for use on a makerfabs UWB Pro with Display module

## Usage

This device listens on a single, fixed DW1000 config (channel 3, 64 MHz PRF, 1024-symbol preamble, 850 kbps, extended PHY header — see `rx_config` in [main/main.c](main/main.c)) and logs every reception — good frames and near-miss errors alike — with detailed signal diagnostics (noise/amplitude, frequency offset, RX timestamp, decoded frame-info fields), so it can be used to characterize a UWB transmitter you don't control the configuration of.

**Hardware**: an ESP32 wired to a DW1000 module. Pins are defined in `hardware_defs.h` (per-project copy, since the transmitter rig may be a different physical board) — `SPI_MOSI/MISO/CLK`, `UWB_CS`, `UWB_RST`, `UWB_IRQ`. Adjust these to match your actual wiring before building.

**Typical workflow:**
1. **Capture.** Run `idf.py monitor | python3 uwb_live.py` against the receiver's port (see [uwb_live.py](uwb_live.py) for full usage/options) to turn the live serial log into a `.pcap` you can open in Wireshark, plus a clean text log (`uwb_capture.txt`) with the full per-frame diagnostics.
3. **Point it at the real target.** Once validated, swap the known transmitter for the unknown one you're trying to diagnose, listening on the same config (or edit `rx_config` to match whatever channel/PRF/rate you expect it to use — see the PLEN/PAC/data-rate reference table in `main/main.c`). If Wireshark dissects the result as clean, standard IEEE 802.15.4 the way it did during validation, the unknown device is using standard framing; if it doesn't (or frames never arrive at all), that's a real signal — either the device uses a non-standard/proprietary format, encryption, or a different PHY config than what the receiver is listening for. The per-frame diagnostics in the text log (noise, frequency offset, RX_FINFO-decoded rate/PRF) are there to help tell those cases apart even when the dissector can't.

## Projects

This repo contains two independent ESP-IDF projects, each with its own `build/` directory and its own `idf.py` lifecycle:

- **Receiver** (repo root) — listens continuously on a single fixed DW1000 config and logs rich RX diagnostics for every reception, good or bad. See [main/main.c](main/main.c).
- **Transmitter** ([transmitter/](transmitter/)) — continuously transmits a minimal, standards-compliant IEEE 802.15.4 Data frame on the same config, used to validate the receiver (and the Wireshark pipeline) against a known-good over-the-air signal. See [transmitter/main/main.c](transmitter/main/main.c).

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


