# SD-SPI emulator: design notes

The *why* behind the code. Read this before `sdemu.h` and `sdemu_engine.c`.

## 1. What the ESP-IDF host actually does

Read from the ESP-IDF v5.5 source (`components/sdmmc`, `components/esp_driver_sdspi`) and confirmed on hardware. This is what the emulator must satisfy, and it is stricter than the spec in places.

**Init sequence** (`sdmmc_card_init()` in SPI mode):

| # | Command | Card must answer | Notes |
|---|---|---|---|
| 1 | CMD52 `0x80000C08` | nothing | SDIO reset, sent *before* CMD0 while the card is still in SD mode |
| 2 | CMD0, twice | R1 `0x01` | first answer ignored; 80 clocks with CS high before each |
| 3 | CMD8 `0x1AA` | R7 `01 00 00 01 AA` | only the `0xAA` echo is checked; no answer ⇒ host assumes a v1 card |
| 4 | CMD5 | R1 `0x05` (illegal) | SDIO probe: **any OK answer makes the host treat the card as SDIO** |
| 5 | CMD59 arg 1 | R1 | **CRC checking on by default**; read-data CRC16 verified from here on |
| 6 | CMD55 + ACMD41 (HCS) | `0x01` … then `0x00` | up to 300 tries, 10 ms apart |
| 7 | CMD58 | R3, OCR with CCS = 1, **bit 24 = 0** | bit 24 would start the UHS-I voltage switch |
| 8 | CMD10, CMD9 | R1 + 16-byte block | CSD_STRUCTURE must be 0 or 1 |
| 9 | CMD55 + ACMD51 | R1 + 8-byte SCR | SCR_STRUCTURE must be 0 |
| 10 | CMD55 + ACMD13 | R2 + 64-byte block | always sent; init fails without it |
| 11 | clock switch | | to `max_freq_khz` (default 20 MHz) |
| 12 | CMD55 + ACMD51 | identical SCR | compared byte for byte with step 9 |

In normal operation: CMD17/CMD18(+CMD12) for reads, CMD24/CMD25 for writes, CMD13 after reads and writes, never ACMD23, no erase from FAT.

**Byte framing:**

- **Command responses are strict.** A command and its response are one fixed-length transaction (15, 16 or 19 bytes). R1 is searched in the 2nd..9th byte after the command, never the 1st. Not there ⇒ the command fails at once, no retry.
- **The write data response is strictest:** CRC16 and the data response are one 3-byte transaction, and the response is the byte right after the last CRC byte. Zero slack.
- **After CMD12** the 2nd byte is a stuff byte and ignored too.
- **Data tokens and busy are lenient:** polled in separate small transactions (8 bytes for tokens, 1 byte for busy) with gaps of tens of µs, timeouts 1 s (read) / 5 s (write). Before `0xFE` only `0xFF` or `0x00` are accepted.
- **Before every command** the host reads MISO with CS high until it sees two non-zero bytes, so MISO must be high while the card is deselected.

The conclusion that shapes everything: **only the command response window and the write data response are hard real-time.** Everything else can be made slow on purpose.

## 2. Architecture

```
            ESP32-S3 (card)                                       classic ESP32 (host)
  ┌───────────────────────────────────────────────┐            ┌───────────────────────┐
  │ core 1 (interrupts off, IRAM/DRAM only)       │  SCLK/MOSI │ unmodified ESP-IDF     │
  │   bit-banged transport ──► sdemu engine ◄─────┼────────────┤ sdspi_host + sdmmc     │
  │          │ trace ring         │ staging bufs  │  MISO/CS   │ + FATFS, test console  │
  ├──────────┼────────────────────┼───────────────┤            └───────────┬───────────┘
  │ core 0   ▼                    ▼               │                        │ USB serial
  │   console/trace printer   storage worker ──► 6 MiB PSRAM RAM disk      │
  └──────────┬────────────────────────────────────┘                        │
             │ USB serial                                                  │
             └──────────────────────────── tools/sdemu.py (PC) ────────────┘
```

Core 1 runs only the transport and the engine, with bounded work per byte. Everything unbounded (PSRAM, console, printing) runs on core 0. The cores meet only through lock-free structures: the staging buffers with request flags, the trace ring, and the double-buffered fault table.

## 3. Engine design

### 3.1 Portable engine, thin transport
The engine is plain C without ESP-IDF. It sees bytes (`sdemu_rx`, `sdemu_tx`), a clock, a trace callback and a storage interface. The same code runs on the laptop against a simulated host and on the S3 against a real one, and a faster transport could replace the bit-banged one without touching protocol logic.

### 3.2 The byte pipeline
At a byte boundary the transport must put the next byte's first bit on MISO within half a bit, too soon to ask the engine. So it always holds the next byte in advance:

1. byte k ends → start shifting out the prepared byte k+1, then `rx(byte k)`
2. middle of byte k+1 → `tx()` for byte k+2

A reaction to byte k appears at byte k+2 at the earliest. `rx()` and `tx()` sit in different bit gaps, so their costs never add up.

**The write data response breaks the pipeline:** it must be the byte right after the last CRC byte, one byte too early. The engine sends a pre-computed *guess* (`0x05`, or what a fault rule decided at block start, or `0x0B` if the first CRC byte already mismatches) and corrects it in `rx()` of the last CRC byte. All data response tokens are `000xxxx1`, so bit 7, already on the wire, never changes. (Laptop test 11 forces this path.)

### 3.3 Split command handling (N_CR = 3)
Decode + CRC7 + fault lookup + execute + trace in one bit gap was too much (~520 of 600 cycles). Now:

- `rx(last command byte)` only marks the frame complete,
- the next `tx()` decodes and checks CRC7 (sends `0xFF`),
- the `tx()` after that executes and returns R1.

R1 lands on the 3rd byte after the command: inside ESP-IDF's window (2nd–9th) and the spec's N_CR (1–8). A `delay_response` fault can add up to 6 more bytes before the host gives up.

### 3.4 Bounded work per call
Blocks are copied and CRC'd in 8-byte steps, one step per `tx()`. A read prepares its block while sending `0xFF` (the host keeps polling for `0xFE`); a write is committed while signalling busy. Register blocks (CID, CSD, SCR, the 64-byte SD status) go through the same 8-byte path.

**N_AC:** the read token is held back at least 16 bytes after R1. The host does not scan the rest of the command transaction for a token, so a token that arrives inside it would be lost (register reads are prepared in 2 steps, fast enough to hit that).

### 3.5 Core 1 never touches PSRAM
A PSRAM cache miss costs ~600 cycles. The storage interface lets a backend answer `BUSY`; the S3 port's staged backend keeps one 512-byte read buffer and one write buffer in internal RAM, and a core-0 worker moves whole blocks to and from PSRAM. While a block isn't staged, the engine keeps the host waiting with `0xFF` (read) or busy `0x00` (write), like a slow card. The worker polls every 1 ms (`CONFIG_FREERTOS_HZ=1000`), far inside the host's 1 s / 5 s timeouts. After the last chunk of a block it prefetches the next one for CMD18.

### 3.6 Everything hot in IRAM, cheap time
`linker.lf` places the whole component in IRAM/DRAM. Time comes from the CPU cycle counter (one instruction), not `esp_timer_get_time()` (hundreds of cycles). The cycle counter is per core, so it is anchored on core 1 and kept from wrapping by the idle loop.

### 3.7 Fault rules: preview, then commit
Rules match on command (ACMDs as 100+n), LBA range and occurrence (`nth`, `every`, `max`), at three stages:

| Stage | Actions |
|---|---|
| command | `no_response`, `delay_response` (bytes), `r1_error` (bits) |
| read block | `read_delay` (ms), `error_token`, `corrupt_crc16`, `flip_byte` (offset, mask; CRC stays valid) |
| write block | `write_error` (0x0D), `write_crc_error` (0x0B), `busy` (ms), `drop_write` |

During CMD18 the engine prepares the *next* block before it knows whether CMD12 is coming. Read rules are therefore only *previewed* during preparation and *committed* when the token really goes out; otherwise "fail the 3rd block" would count a block the host never received (laptop test 9).

The table is double-buffered: core 0 writes the inactive copy and publishes it; the engine swaps at the next command decode, with a compare-and-swap so the writer can safely take back a table that hasn't been picked up yet. Adding or clearing rules resets all counters.

### 3.8 CS glitch filter
On jumper wires the S3 saw short spikes on CS. A CS change only counts after 16 identical reads in a row (~70 ns), and rejected spikes are counted (`status`).

## 4. Timing budget

| SPI clock | Time per bit | CPU cycles per bit (240 MHz) | Bit-banging? |
|---|---|---|---|
| 400 kHz | 2.5 µs | 600 | yes, measured worst case 463 |
| 1 MHz | 1 µs | 240 | not with the current per-byte work |
| 5 MHz | 200 ns | 48 | only a bare shift loop |
| 20 MHz | 50 ns | 12 | no |

The response deadline in *time* at 20 MHz (9 bytes = 3.6 µs ≈ 860 cycles) is not hopeless; following every *edge* in software is. Faster transports (hardware SPI slave with live DMA patching, a hybrid with hardware for data blocks, LCD_CAM/I2S as serial engines) are the open research question.

## 5. Transport (port/esp32s3/sdemu_esp.c)

- **Dedicated GPIO:** SCLK, MOSI and CS are read, and MISO is written, with single-cycle CPU instructions (`dedic_gpio_cpu_ll_*`). The bundles are created on core 1, because dedicated GPIO is per-CPU.
- **Core 1** runs the card task with `portDISABLE_INTERRUPTS()` and never yields. The task and interrupt watchdogs for CPU1 are disabled in `sdkconfig.defaults`.
- **MISO** is driven high while deselected (single card on the bus, so no tri-state is needed).
- **No flash writes while running:** flash operations stall the other core through a high-priority interrupt that `portDISABLE_INTERRUPTS()` does not mask. No NVS, no OTA.

## 6. Diagnostics

| Tool | How | Command |
|---|---|---|
| Pin sampler | core 0 samples the 4 pins 20 000× over 200 ms | `pins` |
| Cycle counters | worst-case cycles of `rx()`, `tx()`, select | `status` |
| PCNT cross-check | hardware counts SCLK rising edges while CS is low; compared with the software count at every byte; first mismatch per window logged | `status`, `slipcheck on/off` |
| Capture mode | core 1 records every pin change with a cycle stamp for N CS windows, instead of answering | `capture N`, `capture dump` |
| Idle stall detector | gaps > 1 µs between idle-loop iterations | `status` |
| CS spike counter | changes rejected by the glitch filter | `status` |
| Host debug log | ESP-IDF's own SD driver debug messages | host: `debug on` |

PCNT saturates at 32 767 edges, so windows longer than ~4095 bytes are not checked.

## 7. Testing

`test/sim` runs the engine against a byte-level model of the ESP-IDF host (section 1) through exactly the same pipeline as the S3 transport, with 20 µs of simulated time per byte. 14 groups: CRC sanity, init, single and multi block, write error, read CRC, no/delayed response, busy and read delay, flipped byte, exact fault counting across CMD18+CMD12, error token and R1 error, bad command CRC and in-flight data-response correction, power cycle, and the CMD8-ignored host behaviour. It catches protocol logic errors in seconds and cannot catch timing problems.

`tools/sdemu.py` + `tools/scenarios/*.yaml` run the same faults against the real host.

## 8. Bring-up log (2026-10-26, 400 kHz)

| # | Symptom | Found with | Cause | Fix |
|---|---|---|---|---|
| 1 | card counts 1 transaction, decodes nothing | `pins`: CS never low at the card | CS jumper not connected | rewire |
| 2 | CMD0 decoded, host reads "CRC error" in R1 | cycle counter: 895 cycles at a byte boundary | 160 MHz, `-Og` | 240 MHz, `-O2` |
| 3 | still ~700 cycles | cycle counter | `esp_timer_get_time()` for trace stamps | CPU cycle counter clock |
| 4 | CMD8 decoded as "CMD21 0x0FFFFFFF" | bit arithmetic + PCNT: one edge lost | decode + execute in one bit gap | split decode/execute (N_CR = 3) |
| 5 | windows cut short or shifted by 2–3 bits | `capture`: host timing constant, CS spikes at the card | marginal CS contact | CS glitch filter (16 reads) |
| 6 | ACMD13 token read as `0x7F` | host debug log + bit arithmetic | 64-byte register copy in one step | 8-byte steps |
| 7 | FAT mount fails on first sector read | PCNT + cycle counter: `tx()` = 610 cycles | PSRAM cache miss on core 1 | core-0 storage worker |

False leads worth remembering: an SCLK glitch filter made things worse (it only added latency); "long gaps between edges" was a broken metric because the host pauses between transactions with CS low; and core 1 was never stalled by the system.

**Results:** `info` 6/6, `fat 16` 5/5 (28 kB/s write, 35 kB/s read, line rate 50 kB/s), raw sectors OK, 7/7 fault scenarios as expected. Worst case `rx()` 266, `tx()` 463 of 600 cycles; 33 CS spikes filtered in one FAT run.

**Host behaviour found with the emulator:**
1. A lost CMD13 answer after a write is only logged; the write is reported successful.
2. A card that ignores CMD8 is silently treated as v1; an SDHC card then times out after ~3 s with no hint about CMD8.
3. A flipped byte with a valid CRC passes the entire stack.
4. The response window is strict: 6 extra bytes before R1 work, 9 fail immediately.

## 9. Known limitations

- Verified only at 400 kHz; ESP-IDF's default is 20 MHz.
- Core 1 is consumed entirely; flash writes are forbidden while the card runs.
- Debug instrumentation (PCNT reads, counters) runs in the hot loop and costs cycles.
- Missing: SDSC (byte addressing, CSD v1), write protection, lock/unlock, CMD6 high speed, erase, SDUC.
- A 6 MiB SDHC card is outside the spec (SDHC starts above 2 GB). ESP-IDF accepts it; other hosts might not. A large advertised capacity with the pattern or a sparse backend would be spec-conformant.
- Not yet an ESP-IDF component in the goal-5 sense: no on-device tests, no CI, no registry packaging.
