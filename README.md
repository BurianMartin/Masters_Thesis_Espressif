# SD-SPI Emulator

[![CI](https://github.com/BurianMartin/Masters_Thesis_Espressif/actions/workflows/ci.yml/badge.svg)](https://github.com/BurianMartin/Masters_Thesis_Espressif/actions/workflows/ci.yml)

An **ESP32-S3 that pretends to be an SD card** in SPI mode. A second board running the unmodified ESP-IDF SD driver talks to it as if it were a real SDHC card: it initializes it, mounts FAT on it, writes and reads files. Faults can be programmed on the card side, so host failures become reproducible on demand: a missing response, a delayed response, a corrupted CRC on block N, a rejected write, long busy periods. Every command is traced live.

It is a test instrument for SD hosts: firmware that uses an SD card can be checked against exactly the failure it is supposed to handle.

| | |
|---|---|
| Card | ESP32-S3 with 8 MB PSRAM, 6 MiB SDHC RAM disk |
| Host (reference) | classic ESP32, ESP-IDF `sdspi_host` + `sdmmc` + FATFS, unmodified |
| Protocol | SD Physical Layer Spec v9.10, chapter 7 (SPI mode): SDHC, CSD v2 |
| SPI clock | **400 kHz** (software transport); faster transports are work in progress |
| Faults | 11 actions at command, read-block and write-block level |
| Toolchain | ESP-IDF v5.5.5 |

## Contents

- [How it works](#how-it-works)
- [Repository layout](#repository-layout)
- [Hardware](#hardware)
- [Quick start](#quick-start)
- [Installation in detail](#installation-in-detail)
- [Running the tests](#running-the-tests)
- [Using it by hand](#using-it-by-hand)
- [Fault injection reference](#fault-injection-reference)
- [Troubleshooting](#troubleshooting)
- [Development](#development)
- [License](#license)

## How it works

```
            ESP32-S3 (card)                                       ESP32 (host)
  ┌───────────────────────────────────────────────┐            ┌───────────────────────┐
  │ core 1 (interrupts off, IRAM/DRAM only)       │  SCLK/MOSI │ unmodified ESP-IDF     │
  │   bit-banged transport ──► sdemu engine ◄─────┼────────────┤ sdspi_host + sdmmc     │
  │          │ trace ring         │ staging bufs  │  MISO/CS   │ + FATFS, test console  │
  ├──────────┼────────────────────┼───────────────┤            └───────────┬───────────┘
  │ core 0   ▼                    ▼               │                        │ USB serial
  │   console / trace printer   storage worker ──► PSRAM RAM disk          │
  └──────────┬────────────────────────────────────┘                        │
             │ USB serial                                                  │
             └──────────────────────────── tools/sdemu.py (PC) ────────────┘
```

- **The engine** (`components/sdemu`) is portable C with no ESP-IDF dependency. It sees only bytes: "this byte arrived on MOSI", "what goes out next on MISO". The same code runs on the laptop against a simulated host and on the S3 against a real one.
- **The transport** runs on core 1 of the S3 with interrupts off. It follows every SPI clock edge in software using the S3's single-cycle dedicated GPIO instructions. At 400 kHz and 240 MHz that leaves 600 CPU cycles per bit, and the protocol needs up to ~460 of them.
- **Everything slow** runs on core 0: the PSRAM disk (through a staging worker), the console and trace printing. The two cores only meet through lock-free buffers.

The hard real-time constraints, and why the design looks the way it does, are explained in [docs/DESIGN.md](docs/DESIGN.md). Read it before the code.

## Repository layout

| Path | Content |
|---|---|
| `components/sdemu/include/sdemu.h` | engine API: card config, storage backends, fault rules, trace events |
| `components/sdemu/src/` | protocol engine, registers, CRC, storage backends, fault rules |
| `components/sdemu/port/esp32s3/` | bit-banged transport, staged PSRAM storage, trace ring, diagnostics |
| `apps/card/` | ESP32-S3 card firmware with a serial control channel |
| `apps/host/` | classic ESP32 test host (stock ESP-IDF SD stack) |
| `test/sim/` | laptop test bench: engine vs. a byte-accurate model of the ESP-IDF host |
| `tools/run_tests.sh` | the `make test` pipeline |
| `tools/setup.sh` | installs the prerequisites (`make setup`) |
| `tools/sdemu.py` | PC tool: runs fault scenarios against both boards |
| `tools/scenarios/*.yaml` | fault scenarios |
| `docs/DESIGN.md` | design notes, ESP-IDF host behaviour, timing budget, bring-up log |

## Hardware

**You need:**

- 1× ESP32-S3 DevKitC with octal PSRAM (e.g. ESP32-S3-WROOM-1 **N8R8** or N16R8)
- 1× classic ESP32 DevKit (ESP32-WROOM-32 / ESP32-D0WD)
- 5× female-female jumper wires, as short as possible (10–20 cm)
- 2× USB cables **that carry data** (many only charge)

**Wiring:**

| Wire | Signal | ESP32 (host) | ESP32-S3 (card) |
|---|---|---|---|
| 1 | SCLK | GPIO 18 | GPIO 12 |
| 2 | MOSI (host → card) | GPIO 23 | GPIO 11 |
| 3 | MISO (card → host) | GPIO 19 | GPIO 13 |
| 4 | CS | GPIO 5 | GPIO 10 |
| 5 | GND | GND | GND |

```
  Classic ESP32                          ESP32-S3
  GPIO 18  SCLK ───────────────────────► GPIO 12
  GPIO 23  MOSI ───────────────────────► GPIO 11
  GPIO 19  MISO ◄─────────────────────── GPIO 13
  GPIO  5  CS   ───────────────────────► GPIO 10
  GND           ──────────────────────── GND
      │ USB (/dev/ttyUSB0)                   │ USB "UART" port (/dev/ttyACM0)
```

- **Do not connect 3V3 or 5V** between the boards. Each is powered by its own USB cable; only GND is shared.
- SPI is not crossed: MOSI goes to MOSI, MISO to MISO.
- Go by the silkscreen labels; clone boards order their headers differently.
- Plug the S3 in through its **UART** USB port (CH343 bridge). The console is on UART0, so the native "USB" port shows nothing.
- The S3 pins are SPI2's native pins and stay clear of GPIO 33–37, which the octal PSRAM uses.
- Host pins can be changed in `idf.py menuconfig` → *SDSPI test host*. Card pins are at the top of `apps/card/main/main.c`.

## Quick start

On a Linux machine (Ubuntu/Debian, Fedora or Arch):

```sh
git clone https://github.com/BurianMartin/Masters_Thesis_Espressif.git
cd Masters_Thesis_Espressif
make setup        # system packages, ESP-IDF v5.5.5, serial port access (~2 GB, once)
                  # log out and back in if it added you to the dialout group
make test         # wire and plug in both boards first
```

`make test` ends with a summary like this:

```
== Summary
  PASS  sim: protocol test bench                     14/14 test groups passed (108 checks, 0 failed)
  PASS  ESP-IDF environment                          ESP-IDF v5.5.5
  PASS  build: card firmware (ESP32-S3)
  PASS  build: host firmware (ESP32)
  PASS  detect: ESP32-S3 card                        /dev/ttyACM0
  PASS  detect: ESP32 host                           /dev/ttyUSB0
  PASS  flash: card -> /dev/ttyACM0
  PASS  flash: host -> /dev/ttyUSB0
  PASS  hw: host initializes the emulated card       name=SDEMU sectors=12288
  PASS  hw: fault scenarios
  PASS    scenario: baseline
  PASS    scenario: write_error
  ...
```

## Installation in detail

`make setup` runs [`tools/setup.sh`](tools/setup.sh). It asks before each step, skips what is already done, and is safe to re-run.

| Step | What it does |
|---|---|
| System packages | `git gcc make cmake ninja python3 python3-venv flex bison gperf ccache libusb dfu-util …` via apt, dnf or pacman |
| ESP-IDF | clones `v5.5.5` into `~/esp/esp-idf` and runs `install.sh esp32,esp32s3` (toolchains + Python env) |
| Serial access | adds you to `dialout` (`uucp` on Arch) so you can open `/dev/ttyUSB*` and `/dev/ttyACM*` |

Options:

```sh
tools/setup.sh --dry-run                 # print what would happen
tools/setup.sh --yes                     # no questions
tools/setup.sh --no-system               # skip the package manager (no sudo)
tools/setup.sh --idf-dir /opt/esp-idf    # other ESP-IDF location
tools/setup.sh --idf-version v5.5.6      # other ESP-IDF version (untested)
```

**Doing it by hand** instead:

```sh
sudo apt-get install git wget flex bison gperf python3 python3-pip python3-venv cmake \
     ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0 build-essential
mkdir -p ~/esp && cd ~/esp
git clone -b v5.5.5 --recursive --depth 1 --shallow-submodules https://github.com/espressif/esp-idf.git
~/esp/esp-idf/install.sh esp32,esp32s3
sudo usermod -aG dialout $USER    # then log out and back in
```

An existing ESP-IDF installation elsewhere works too: `make test IDF_EXPORT=/path/to/esp-idf/export.sh`, or run `make test` from a shell where `export.sh` has already been sourced.

**Only the laptop tests** need nothing but `gcc` and `make`: `make sim`.

**Windows / macOS:** the firmware builds anywhere ESP-IDF runs, but the test pipeline is a bash script for Linux. On Windows, use a native Linux machine or pass the USB devices into WSL2 with [usbipd-win](https://github.com/dorssel/usbipd-win).

## Running the tests

| Command | What it does | Needs |
|---|---|---|
| `make sim` | engine against the simulated ESP-IDF host (14 test groups) | gcc, make |
| `make build` | builds both firmwares | ESP-IDF |
| `make hwtest` | build, detect boards, flash, hardware tests | ESP-IDF, both boards |
| `make test` | all of the above | ESP-IDF, both boards |
| `make clean` | removes builds and logs | |

What `make test` does, in order:

1. **Laptop tests:** CRC, init, single/multi block, every fault type, exact fault counting, CRC rejection, power cycle.
2. **Builds** the card firmware (`esp32s3`) and the host firmware (`esp32`).
3. **Detects the boards:** asks every `/dev/ttyACM*` and `/dev/ttyUSB*` with esptool which chip it is.
4. **Flashes** both boards.
5. **Smoke test:** the host initializes the emulated card (`info`).
6. **Fault scenarios** from `tools/scenarios/`: a normal card, then six faults, each checked against what the stock ESP-IDF stack must report.
7. **Card diagnostics:** worst-case CPU cycles per bit, CS glitches, lost clock edges.

A stage that fails prints the end of its log, and stages that depend on it are skipped with the reason. Full logs: `test-logs/<timestamp>/`. The exit code is 0 only if nothing failed.

Ports are detected automatically; to force them:

```sh
make test CARD_PORT=/dev/ttyACM0 HOST_PORT=/dev/ttyUSB0
```

### Scenarios

| Scenario | Card does | Expected host result |
|---|---|---|
| `baseline` | behaves normally | init, FAT (16 KiB file) and raw sectors OK |
| `write_error` | rejects the 2nd write (`0x0D`) | 2nd write `ESP_FAIL`, 1st and 3rd OK |
| `corrupt_crc16` | wrong CRC16 on LBA 4000 | `ESP_ERR_INVALID_CRC`; LBA 4001 OK |
| `flip_byte` | flips a byte of LBA 5000, CRC valid | driver says OK, only the application's compare fails |
| `cmd8_no_response` | ignores CMD8 | host assumes a v1 card, `ESP_ERR_TIMEOUT` after ~3 s |
| `write_busy` | stays busy 300 ms after a write | write OK, takes ≥ 300 ms |
| `cmd13_no_response` | ignores the status check after a write | both writes still reported OK |

## Using it by hand

Open a serial monitor on each board (exit with `Ctrl+]`):

```sh
. ~/esp/esp-idf/export.sh
idf.py -C apps/card -p /dev/ttyACM0 monitor     # S3
idf.py -C apps/host -p /dev/ttyUSB0 monitor     # ESP32
```

**Host commands** (each test re-initializes the card and ends with a `RESULT …` line):

| Command | Does |
|---|---|
| `info` | initialize the card and print CID/CSD info |
| `fat [KB]` | mount + format FAT, write a file, read it back, compare (default 16 KB) |
| `loop [RUNS] [KB]` | the FAT test several times |
| `raw [LBA] [N]` | multi-block write + read + compare of N sectors |
| `write LBA` / `read LBA` / `verify LBA` | single-sector operations |
| `debug on\|off` | ESP-IDF SD driver debug log |

**Card commands** (each answers `OK` or `ERR …`):

| Command | Does |
|---|---|
| `help` | list commands |
| `status` | counters, worst-case cycles, CS spikes, PCNT slips, fault rules |
| `info` | register contents |
| `trace on\|off` | print every command live |
| `fault add key=value …` | add a fault rule (see below) |
| `fault list` / `fault clear` | show / remove rules |
| `power_cycle` | card goes silent until the next CMD0 |
| `pins` | sample the four SPI pins for 200 ms |
| `capture N` then `capture dump` | record every pin change of the next N CS windows (a built-in logic analyzer) |
| `csfilter N` | CS glitch filter length (default 16 reads) |
| `slipcheck on\|off` | hardware edge counter cross-check |
| `diag reset` | clear transport counters |

**PC tool** (close the monitors first):

```sh
python tools/sdemu.py run  --card /dev/ttyACM0 --host /dev/ttyUSB0 tools/scenarios/*.yaml
python tools/sdemu.py card --card /dev/ttyACM0 status
python tools/sdemu.py host --host /dev/ttyUSB0 fat 16
```

It needs `pyserial` and `pyyaml`; both come with the ESP-IDF Python environment.

**Clock sweep:** `idf.py -C apps/host menuconfig` → *SDSPI test host* → *SD clock after initialization (kHz)*, then rebuild, flash, and watch `status` on the card.

## Fault injection reference

```
fault add action=<action> [cmd=N|acmdN|any] [lba=A|A-B] [nth=N] [every=K] [max=M] [param=…] [mask=…]
```

| Key | Meaning | Default |
|---|---|---|
| `action` | what to do (table below) | required |
| `cmd` | command it applies to: `17`, `24`, `acmd41`, `any` | `any` |
| `lba` | block address or range | all |
| `nth` | fire on the n-th matching occurrence | `1` |
| `every` | then fire every k-th match after it (`0` = only once) | `0` |
| `max` | cap on the number of fires (`0` = no cap) | `0` |
| `param` | the action's value; aliases `ms`, `bytes`, `offset`, `token`, `bits` | |
| `mask` | XOR mask for `flip_byte` | `0xFF` |

| Stage | Action | Effect | Value |
|---|---|---|---|
| command | `no_response` | card stays silent, command not executed | |
| command | `delay_response` | extra `0xFF` bytes before R1 (ESP-IDF tolerates up to 6) | `bytes` |
| command | `r1_error` | R1 with these error bits, command not executed | `bits` |
| read block | `read_delay` | wait before the data token | `ms` |
| read block | `error_token` | data error token instead of the block | `token` (default `0x01`) |
| read block | `corrupt_crc16` | wrong CRC16 | |
| read block | `flip_byte` | flip a byte, CRC stays valid | `offset`, `mask` |
| write block | `write_error` | data response `0x0D`, block not stored | |
| write block | `write_crc_error` | data response `0x0B`, block not stored | |
| write block | `busy` | hold MISO low longer | `ms` |
| write block | `drop_write` | accept (`0x05`) but silently don't store | |

Examples:

```
fault add action=write_error cmd=24 nth=2             # reject the 2nd single-block write
fault add action=corrupt_crc16 lba=4000 every=1       # every read of LBA 4000
fault add action=flip_byte cmd=18 nth=2 every=2 max=2 offset=10 mask=0x01
fault add action=no_response cmd=8 every=1            # card ignores CMD8
```

Read-block rules count only blocks that are really sent: a block the card prepared but that CMD12 cancelled does not count. Adding or clearing rules resets all counters.

### Writing a scenario

```yaml
name: my_scenario
description: What the card does and what the host should report.
faults:
  - action=write_error cmd=24 nth=2
steps:
  - host: write 3000
    expect: {status: ok}
  - host: write 3001
    expect: {status: fail, err: ESP_FAIL, step: write}
  - host: info
    expect: {status: ok, min_ms: 0, max_ms: 5000}
    timeout: 30
```

`expect` can check `status`, `err`, `step`, `min_ms` and `max_ms` against the host's `RESULT` line. Put the file in `tools/scenarios/` and `make test` picks it up.

## Troubleshooting

| Symptom | Likely cause / fix |
|---|---|
| `detect: … FAIL`, board not found | Cable only charges, or a USB hub port without data. Try another cable/port; `lsusb` and `journalctl -k -f` show whether the board enumerates at all. |
| `no write permission` in the detect log | You're not in `dialout` yet: run `make setup` or `sudo usermod -aG dialout $USER`, then log out and in. |
| S3 found but the card console is silent | The S3 is on its native USB port; use the UART one. |
| `ESP-IDF environment FAIL` | Run `make setup`, or `make test IDF_EXPORT=/path/to/export.sh`. |
| Flashing fails with "port is busy" | An `idf.py monitor` or other serial program still has the port open. |
| `hw: host initializes the emulated card` fails | Wiring. Run `pins` on the card while the host runs `info`: CS must go low and SCLK/MOSI must toggle. Check MISO goes to MISO. |
| Random failures, `cs_spikes` in the diagnostics | Marginal CS contact: shorter wires, re-seat the CS jumper. |
| `pcnt slips` > 0 | The card lost a clock edge: host clock too fast for the software transport, or a noisy SCLK wire. |
| Boards reset when the tool starts | Expected: opening a port toggles DTR/RTS. The tool waits 3.5 s for both to boot. |

## Development

- **Laptop first:** `make sim` runs in about a second and checks the protocol logic byte by byte against a model of the ESP-IDF host (`test/sim/sim_host.c`). Add a test group in `test/sim/test_main.c` for every protocol change. `./test/sim/sim_test -v N` runs one group with a full host/card log.
- **Timing rules for core 1:** no flash or PSRAM access, no `esp_timer`, no logging, no FreeRTOS calls, and bounded work per engine call. See [docs/DESIGN.md](docs/DESIGN.md), sections 3–5.
- **CI** (GitHub Actions) runs the laptop tests and builds both firmwares with ESP-IDF v5.5.5 on every push.

### Known limitations

- Verified only at 400 kHz; ESP-IDF's default SD clock is 20 MHz.
- Core 1 is used entirely by the card; flash writes (NVS, OTA) are not allowed while it runs.
- SDHC only: no SDSC (byte addressing, CSD v1), write protection, lock/unlock, CMD6 high-speed or erase.
- A 6 MiB SDHC card is outside the spec (SDHC starts above 2 GB). ESP-IDF accepts it; other hosts might not.

## License

[Apache License 2.0](LICENSE)
