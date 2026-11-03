/*
 * SD card emulator firmware for the ESP32-S3.
 *
 * A 6 MiB SDHC card backed by PSRAM, plus a line-based control channel on the
 * serial console, used by humans (idf.py monitor) and by tools/sdemu.py.
 * Every command answers with a final line "OK" or "ERR <reason>".
 *
 * Wiring (SPI2's native pins, clear of the octal PSRAM on GPIO 33-37):
 *   SCLK 12, MOSI 11, MISO 13, CS 10, plus GND. No 3V3 between the boards.
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "driver/uart.h"
#include "sdemu_esp.h"

#define PIN_SCLK 12
#define PIN_MOSI 11
#define PIN_MISO 13
#define PIN_CS   10
#define CAPACITY_BLOCKS 12288u   /* 6 MiB */

static const char *TAG = "card";
static bool s_trace_on;

static void reply_ok(void)
{
    printf("OK\n");
    fflush(stdout);
}

static void reply_err(const char *why)
{
    printf("ERR %s\n", why);
    fflush(stdout);
}

/* ---- trace printing (core 0) ------------------------------------------- */

static void print_cmd(const sdemu_event_t *ev)
{
    if (ev->cmd >= 100) {
        printf("ACMD%-2u", ev->cmd - 100);
    } else {
        printf("CMD%-3u", ev->cmd);
    }
    printf(" %-22s", sdemu_cmd_name(ev->cmd));
}

static void print_event(const sdemu_event_t *ev)
{
    printf("T %10lu ", (unsigned long)ev->t_us);
    switch (ev->type) {
    case SDEMU_EV_CMD:
        print_cmd(ev);
        if (ev->r1 == 0xFF) {
            printf(" arg=%08lx  (no response)\n", (unsigned long)ev->arg);
        } else {
            printf(" arg=%08lx  R1=%02x\n", (unsigned long)ev->arg, ev->r1);
        }
        break;
    case SDEMU_EV_CMD_CRC:
        print_cmd(ev);
        printf(" arg=%08lx  CRC7 ERROR, R1=%02x\n", (unsigned long)ev->arg, ev->r1);
        break;
    case SDEMU_EV_READ_BLOCK:
        printf("  read  lba=%-8lu token=%02x\n", (unsigned long)ev->arg, ev->extra);
        break;
    case SDEMU_EV_WRITE_BLOCK:
        printf("  write lba=%-8lu response=%02x\n", (unsigned long)ev->arg, ev->extra);
        break;
    case SDEMU_EV_WRITE_STOP:
        printf("  stop token\n");
        break;
    case SDEMU_EV_FAULT:
        printf("  FAULT %s (cmd %u, arg/lba %lu)\n", sdemu_fault_action_name(ev->extra), ev->cmd,
               (unsigned long)ev->arg);
        break;
    case SDEMU_EV_POWER:
        printf("  power cycle\n");
        break;
    default:
        printf("  event %u\n", ev->type);
        break;
    }
}

static void drain_trace(void)
{
    sdemu_event_t ev;
    int n = 0;
    while (sdemu_esp_trace_pop(&ev)) {
        if (s_trace_on) {
            print_event(&ev);
        }
        if (++n >= 64) {
            break;   /* keep the console responsive */
        }
    }
}

/* ---- commands ---------------------------------------------------------- */

static void cmd_status(void)
{
    sdemu_stats_t st;
    sdemu_esp_diag_t d;
    sdemu_fault_table_t ft;
    sdemu_get_stats(sdemu_esp_card(), &st);
    sdemu_esp_get_diag(&d);
    sdemu_faults_get(sdemu_esp_card(), &ft);
    uint32_t budget = d.cpu_mhz * 1000u / 400u;   /* cycles per bit at 400 kHz */
    printf("engine: commands=%lu crc_errors=%lu blocks_read=%lu blocks_written=%lu "
           "write_rejects=%lu faults_fired=%lu storage_waits=%lu\n",
           (unsigned long)st.commands, (unsigned long)st.cmd_crc_errors, (unsigned long)st.blocks_read,
           (unsigned long)st.blocks_written, (unsigned long)st.write_rejects,
           (unsigned long)st.faults_fired, (unsigned long)st.storage_busy);
    printf("transport: transactions=%lu bytes=%lu partial=%lu cs_spikes=%lu\n",
           (unsigned long)d.windows, (unsigned long)d.bytes, (unsigned long)d.partial_windows,
           (unsigned long)d.cs_spikes);
    printf("cycles (worst): rx=%lu tx=%lu select=%lu  (bit at 400 kHz = %lu)\n",
           (unsigned long)d.worst_rx_cycles, (unsigned long)d.worst_tx_cycles,
           (unsigned long)d.worst_select_cycles, (unsigned long)budget);
    printf("idle: iterations=%lu stalls>1us=%lu worst_stall=%lu cycles\n",
           (unsigned long)d.idle_iters, (unsigned long)d.stalls, (unsigned long)d.worst_stall_cycles);
    printf("pcnt slips=%lu", (unsigned long)d.slips);
    for (uint32_t i = 0; i < d.slips && i < SDEMU_ESP_SLIP_LOG; i++) {
        printf(" @byte%lu", (unsigned long)d.slip_byte[i]);
    }
    printf("\ntrace dropped=%lu\n", (unsigned long)d.trace_dropped);
    for (uint32_t i = 0; i < ft.count; i++) {
        char buf[200];
        sdemu_fault_format(&ft.rules[i], buf, sizeof(buf));
        printf("fault[%lu] %s\n", (unsigned long)i, buf);
    }
}

static void cmd_info(void)
{
    const sdemu_t *c = sdemu_esp_card();
    printf("CID:");
    for (int i = 0; i < 16; i++) {
        printf(" %02x", c->cid[i]);
    }
    printf("  (%.5s)\nCSD:", (const char *)&c->cid[3]);
    for (int i = 0; i < 16; i++) {
        printf(" %02x", c->csd[i]);
    }
    printf("  (%lu blocks, %lu MiB)\nSCR:", (unsigned long)c->cfg.capacity_blocks,
           (unsigned long)(c->cfg.capacity_blocks / 2048));
    for (int i = 0; i < 8; i++) {
        printf(" %02x", c->scr[i]);
    }
    printf("\nOCR after init: 0xC0FF8000 (busy done, CCS=1: SDHC)\n");
}

static void handle_line(char *line)
{
    char *argv[16];
    int argc = 0;
    for (char *t = strtok(line, " \t"); t && argc < 16; t = strtok(NULL, " \t")) {
        argv[argc++] = t;
    }
    if (argc == 0) {
        return;
    }
    const char *cmd = argv[0];

    if (strcmp(cmd, "help") == 0) {
        printf("commands:\n"
               "  status                     counters, worst-case cycles, fault rules\n"
               "  info                       register contents\n"
               "  trace on|off               print every command live\n"
               "  fault add key=value ...    e.g. fault add action=write_error cmd=24 nth=2\n"
               "           actions: no_response delay_response r1_error read_delay error_token\n"
               "                    corrupt_crc16 flip_byte write_error write_crc_error busy drop_write\n"
               "           keys: cmd lba(=a or a-b) nth every max param|ms|bytes|offset|token|bits mask\n"
               "  fault list | fault clear\n"
               "  power_cycle                card silent until the next CMD0\n"
               "  pins                       sample the four SPI pins for 200 ms\n"
               "  capture N | capture dump   record the next N CS windows instead of answering\n"
               "  csfilter N                 CS glitch filter (identical reads, default 16)\n"
               "  slipcheck on|off           PCNT edge cross-check at every byte\n"
               "  diag reset                 clear transport counters\n");
        reply_ok();
    } else if (strcmp(cmd, "status") == 0) {
        cmd_status();
        reply_ok();
    } else if (strcmp(cmd, "info") == 0) {
        cmd_info();
        reply_ok();
    } else if (strcmp(cmd, "trace") == 0 && argc == 2) {
        s_trace_on = strcmp(argv[1], "on") == 0;
        reply_ok();
    } else if (strcmp(cmd, "fault") == 0 && argc >= 2) {
        if (strcmp(argv[1], "clear") == 0) {
            sdemu_faults_clear(sdemu_esp_card());
            reply_ok();
        } else if (strcmp(argv[1], "list") == 0) {
            sdemu_fault_table_t ft;
            sdemu_faults_get(sdemu_esp_card(), &ft);
            for (uint32_t i = 0; i < ft.count; i++) {
                char buf[200];
                sdemu_fault_format(&ft.rules[i], buf, sizeof(buf));
                printf("fault[%lu] %s\n", (unsigned long)i, buf);
            }
            reply_ok();
        } else if (strcmp(argv[1], "add") == 0 && argc >= 3) {
            char text[160] = "";
            for (int i = 2; i < argc; i++) {
                strlcat(text, argv[i], sizeof(text));
                strlcat(text, " ", sizeof(text));
            }
            sdemu_fault_rule_t r;
            char err[80];
            if (!sdemu_fault_parse(text, &r, err, sizeof(err))) {
                reply_err(err);
            } else if (!sdemu_faults_add(sdemu_esp_card(), &r)) {
                reply_err("rule table full");
            } else {
                reply_ok();
            }
        } else {
            reply_err("usage: fault add ... | fault list | fault clear");
        }
    } else if (strcmp(cmd, "power_cycle") == 0) {
        sdemu_esp_power_cycle();
        reply_ok();
    } else if (strcmp(cmd, "pins") == 0) {
        sdemu_esp_pins(stdout);
        reply_ok();
    } else if (strcmp(cmd, "capture") == 0 && argc == 2) {
        if (strcmp(argv[1], "dump") == 0) {
            sdemu_esp_capture_dump(stdout);
            reply_ok();
        } else if (sdemu_esp_capture_arm((uint32_t)atoi(argv[1])) == ESP_OK) {
            printf("capture armed: the card does not answer during the captured windows\n");
            reply_ok();
        } else {
            reply_err("bad window count");
        }
    } else if (strcmp(cmd, "csfilter") == 0 && argc == 2) {
        sdemu_esp_set_cs_filter((uint32_t)atoi(argv[1]));
        reply_ok();
    } else if (strcmp(cmd, "slipcheck") == 0 && argc == 2) {
        sdemu_esp_set_slipcheck(strcmp(argv[1], "on") == 0);
        reply_ok();
    } else if (strcmp(cmd, "diag") == 0 && argc == 2 && strcmp(argv[1], "reset") == 0) {
        sdemu_esp_reset_diag();
        reply_ok();
    } else {
        reply_err("unknown command (try help)");
    }
}

/* ---- console ----------------------------------------------------------- */

static void console_task(void *arg)
{
    (void)arg;
    char line[200];
    size_t len = 0;
    for (;;) {
        uint8_t ch;
        int n = uart_read_bytes(UART_NUM_0, &ch, 1, pdMS_TO_TICKS(10));
        drain_trace();
        if (n <= 0) {
            continue;
        }
        if (ch == '\r' || ch == '\n') {
            if (len) {
                line[len] = '\0';
                printf("\n");
                handle_line(line);
                len = 0;
            }
        } else if ((ch == 0x08 || ch == 0x7F) && len) {
            len--;
        } else if (len < sizeof(line) - 1 && ch >= 0x20) {
            line[len++] = (char)ch;
        }
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 512, 0, 0, NULL, 0));

    size_t bytes = (size_t)CAPACITY_BLOCKS * SDEMU_BLOCK_SIZE;
    uint8_t *disk = heap_caps_calloc(1, bytes, MALLOC_CAP_SPIRAM);
    if (!disk) {
        ESP_LOGE(TAG, "no PSRAM for the %u KiB RAM disk", (unsigned)(bytes / 1024));
        return;
    }
    ESP_LOGI(TAG, "PSRAM: %u KiB free after allocating the %u KiB RAM disk",
             (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024), (unsigned)(bytes / 1024));

    sdemu_esp_config_t cfg = {
        .pin_sclk = PIN_SCLK,
        .pin_mosi = PIN_MOSI,
        .pin_miso = PIN_MISO,
        .pin_cs = PIN_CS,
        .core = 1,
        .disk = disk,
        .cs_filter = 16,
    };
    sdemu_config_default(&cfg.card);
    cfg.card.capacity_blocks = CAPACITY_BLOCKS;
    ESP_ERROR_CHECK(sdemu_esp_start(&cfg));

    cmd_info();
    printf("type 'help' for commands\n");
    xTaskCreatePinnedToCore(console_task, "console", 6144, NULL, 5, NULL, 0);
}
