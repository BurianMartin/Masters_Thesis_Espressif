# SD-SPI emulator

An ESP32-S3 that acts as an SDHC card in SPI mode for an unmodified ESP-IDF host, with programmable fault injection and live tracing. The current transport works at 400 kHz with a bit-banged transport; going faster is the open research question (`docs/DESIGN.md` section 4).

Reading order:

1. `docs/DESIGN.md`: reasoning, timing trade-offs, host findings, bring-up log
2. `components/sdemu/include/sdemu.h`: the API
3. `components/sdemu/src/sdemu_engine.c`: the protocol state machine
4. `test/sim/sim_host.c`: what the real host does on the wire
5. `components/sdemu/port/esp32s3/sdemu_esp.c`, then `apps/`, then `tools/sdemu.py`

## Layout

| Path | Content |
|---|---|
| `components/sdemu/include/sdemu.h` | engine API: card config, storage backends, fault rules, trace events |
| `components/sdemu/src/sdemu_engine.c` | protocol state machine |
| `components/sdemu/src/sdemu_{regs,crc,backend,fault}.c` | registers, CRC, RAM/pattern storage, fault rules |
| `components/sdemu/port/esp32s3/` | bit-banged transport, staged PSRAM storage, trace ring, diagnostics |
| `apps/card/` | ESP32-S3 card firmware with the serial control channel |
| `apps/host/` | classic ESP32 test host (stock ESP-IDF SD stack) |
| `tools/sdemu.py`, `tools/scenarios/*.yaml` | PC scenario runner and 7 fault scenarios |
| `test/sim/` | laptop test bench (engine against a simulated ESP-IDF host) |

## Testing

Connect both boards to the laptop (wiring below) and run:

```sh
make test
```

This runs the laptop tests, builds both firmwares, finds which port has the
ESP32-S3 and which the ESP32 (via esptool), flashes both, runs the hardware
tests and prints a PASS/FAIL summary. Logs go to `test-logs/<timestamp>/`.

| Target | What |
|---|---|
| `make test` | everything |
| `make sim` | laptop tests only (no ESP-IDF, no boards) |
| `make build` | firmware builds only |
| `make hwtest` | build, flash, hardware tests |

Requirements:

- ESP-IDF v5.5 at `~/esp/esp-idf` (or `IDF_EXPORT=/path/to/export.sh`, or an already exported shell)
- `gcc` and `make` for the laptop tests
- your user in the `dialout` group (`sudo usermod -aG dialout $USER`, then log in again)
- the S3 connected through its **UART** USB port (the console is on UART0), not the native USB one
- close any `idf.py monitor` first

Ports can be forced: `make test CARD_PORT=/dev/ttyACM0 HOST_PORT=/dev/ttyUSB0`.

## Hardware

| Signal | ESP32 host | ESP32-S3 card |
|---|---|---|
| SCLK | GPIO 18 | GPIO 12 |
| MOSI | GPIO 23 | GPIO 11 |
| MISO | GPIO 19 | GPIO 13 |
| CS | GPIO 5 | GPIO 10 |
| GND | GND | GND |

- No 3V3/5V between the boards; each is powered by its own USB cable.
- MOSI to MOSI, MISO to MISO (SPI is not crossed).
- Keep the CS jumper short and firmly seated; `status` shows filtered CS spikes.
- The S3 pins are SPI2's native pins and avoid GPIO 33–37 (octal PSRAM).

## Build and flash (ESP-IDF v5.5)

```sh
. ~/esp/esp-idf/export.sh
cd apps/card && idf.py set-target esp32s3 && idf.py -p /dev/ttyACM0 flash monitor   # S3
cd apps/host && idf.py set-target esp32   && idf.py -p /dev/ttyUSB0 flash monitor   # ESP32
```

Check ports with `ls /dev/ttyACM* /dev/ttyUSB*`. Opening a port resets the board.

## Use

Host console: `info`, `fat 16`, `raw 2000 4`, `loop 5 16`, `write LBA`, `read LBA`, `verify LBA`, `debug on`.

Card console: `help`, `status`, `info`, `trace on`, `fault add action=write_error cmd=24 nth=2`, `fault list`, `fault clear`, `power_cycle`, `pins`, `capture 1` then `capture dump`, `csfilter 16`, `slipcheck on|off`, `diag reset`.

Fault scenarios from the PC (close the monitors first):

```sh
python3 tools/sdemu.py run --card /dev/ttyACM0 --host /dev/ttyUSB0 tools/scenarios/*.yaml
```

Clock sweep: `idf.py menuconfig` in `apps/host` → "SDSPI test host" → "SD clock after initialization (kHz)", rebuild, flash, and watch `status` on the card (worst-case cycles, PCNT slips).
