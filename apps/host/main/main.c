/*
 * SDSPI test host for the classic ESP32: the unmodified ESP-IDF SD stack
 * (sdspi_host + sdmmc + FATFS), driven from the serial console.
 *
 * Every test re-initializes the card and ends with one machine-readable line:
 *   RESULT test=<name> status=ok|fail err=<esp_err> step=<where> ms=<n> [...]
 *
 * Wiring: SCLK 18, MOSI 23, MISO 19, CS 5, plus GND (menuconfig "SDSPI test host").
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/stat.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_vfs_fat.h"
#include "driver/uart.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "sdmmc_cmd.h"
#include "sdkconfig.h"

#define MOUNT "/sd"
#define SPI_HOST_ID SPI2_HOST

static const char *TAG = "host";
static const char *const s_sd_tags[] = { "sdspi_host", "sdspi_transaction", "sdmmc_cmd", "sdmmc_init",
                                         "sdmmc_sd", "sdmmc_common", "vfs_fat_sdmmc" };

static sdmmc_host_t make_host(void)
{
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI_HOST_ID;
    host.max_freq_khz = CONFIG_HOST_SD_FREQ_KHZ;
    return host;
}

static sdspi_device_config_t make_dev(void)
{
    sdspi_device_config_t dev = SDSPI_DEVICE_CONFIG_DEFAULT();
    dev.gpio_cs = CONFIG_HOST_PIN_CS;
    dev.host_id = SPI_HOST_ID;
    return dev;
}

static int64_t ms_since(int64_t t0)
{
    return (esp_timer_get_time() - t0) / 1000;
}

static void result(const char *test, esp_err_t err, const char *step, int64_t ms, const char *extra)
{
    printf("RESULT test=%s status=%s err=%s step=%s ms=%lld%s%s\n", test, err == ESP_OK ? "ok" : "fail",
           esp_err_to_name(err), step, (long long)ms, extra ? " " : "", extra ? extra : "");
    fflush(stdout);
}

/* ---- raw card access (no filesystem) ----------------------------------- */

typedef struct {
    sdmmc_host_t host;
    sdspi_dev_handle_t dev;
    sdmmc_card_t card;
} card_t;

static esp_err_t card_open(card_t *c)
{
    memset(c, 0, sizeof(*c));
    c->host = make_host();
    sdspi_device_config_t dev = make_dev();
    esp_err_t err = sdspi_host_init_device(&dev, &c->dev);
    if (err != ESP_OK) {
        return err;
    }
    c->host.slot = c->dev;
    err = sdmmc_card_init(&c->host, &c->card);
    if (err != ESP_OK) {
        sdspi_host_remove_device(c->dev);
    }
    return err;
}

static void card_close(card_t *c)
{
    sdspi_host_remove_device(c->dev);
}

static uint8_t pattern(uint32_t lba, uint32_t i, uint32_t salt)
{
    return (uint8_t)((lba * 131u) ^ (i * 7u) ^ (i >> 8) ^ salt);
}

static uint8_t *dma_buf(size_t n)
{
    return heap_caps_malloc(n, MALLOC_CAP_DMA);
}

static void test_info(void)
{
    card_t c;
    int64_t t0 = esp_timer_get_time();
    esp_err_t err = card_open(&c);
    int64_t ms = ms_since(t0);
    if (err != ESP_OK) {
        result("info", err, "init", ms, NULL);
        return;
    }
    sdmmc_card_print_info(stdout, &c.card);
    char extra[64];
    snprintf(extra, sizeof(extra), "name=%s sectors=%lu", c.card.cid.name, (unsigned long)c.card.csd.capacity);
    card_close(&c);
    result("info", ESP_OK, "init", ms, extra);
}

/* raw LBA N: CMD25 x N, CMD18 x N, compare */
static void test_raw(uint32_t lba, uint32_t n)
{
    card_t c;
    int64_t t0 = esp_timer_get_time();
    esp_err_t err = card_open(&c);
    if (err != ESP_OK) {
        result("raw", err, "init", ms_since(t0), NULL);
        return;
    }
    uint8_t *w = dma_buf(n * 512), *r = dma_buf(n * 512);
    const char *step = "write";
    for (uint32_t i = 0; i < n * 512; i++) {
        w[i] = pattern(lba + i / 512, i % 512, 0x5A);
    }
    t0 = esp_timer_get_time();
    err = sdmmc_write_sectors(&c.card, w, lba, n);
    if (err == ESP_OK) {
        step = "read";
        memset(r, 0, n * 512);
        err = sdmmc_read_sectors(&c.card, r, lba, n);
    }
    if (err == ESP_OK) {
        step = "compare";
        if (memcmp(w, r, n * 512) != 0) {
            err = ESP_ERR_INVALID_CRC;   /* data mismatch */
        }
    }
    result("raw", err, step, ms_since(t0), NULL);
    free(w);
    free(r);
    card_close(&c);
}

/* single-sector operations used by the fault scenarios */
static void test_sector(const char *what, uint32_t lba)
{
    card_t c;
    int64_t t0 = esp_timer_get_time();
    esp_err_t err = card_open(&c);
    if (err != ESP_OK) {
        result(what, err, "init", ms_since(t0), NULL);
        return;
    }
    uint8_t *w = dma_buf(512), *r = dma_buf(512);
    for (uint32_t i = 0; i < 512; i++) {
        w[i] = pattern(lba, i, 0xA5);
    }
    const char *step = "write";
    t0 = esp_timer_get_time();
    if (strcmp(what, "read") != 0) {
        err = sdmmc_write_sectors(&c.card, w, lba, 1);
    }
    if (err == ESP_OK && strcmp(what, "write") != 0) {
        step = "read";
        err = sdmmc_read_sectors(&c.card, r, lba, 1);
    }
    char extra[48] = "";
    if (err == ESP_OK && strcmp(what, "verify") == 0) {
        step = "compare";
        for (int i = 0; i < 512; i++) {
            if (r[i] != w[i]) {
                snprintf(extra, sizeof(extra), "mismatch_at=%d", i);
                break;
            }
        }
    }
    int64_t ms = ms_since(t0);
    if (extra[0]) {
        /* the driver said OK; only the application's compare noticed */
        printf("RESULT test=%s status=fail err=ESP_OK step=compare ms=%lld %s\n", what, (long long)ms, extra);
        fflush(stdout);
    } else {
        result(what, err, step, ms, NULL);
    }
    free(w);
    free(r);
    card_close(&c);
}

/* fat KB: mount (format), write a file, read it back, compare, unmount */
static esp_err_t fat_once(uint32_t kb, double *wr_kbps, double *rd_kbps, const char **step)
{
    sdmmc_host_t host = make_host();
    sdspi_device_config_t dev = make_dev();
    esp_vfs_fat_sdmmc_mount_config_t mcfg = {
        .format_if_mount_failed = true,
        .max_files = 2,
        .allocation_unit_size = 4096,
    };
    sdmmc_card_t *card = NULL;
    *step = "mount";
    esp_err_t err = esp_vfs_fat_sdspi_mount(MOUNT, &host, &dev, &mcfg, &card);
    if (err != ESP_OK) {
        return err;
    }
    *step = "format";
    err = esp_vfs_fat_sdcard_format(MOUNT, card);
    size_t n = (size_t)kb * 1024;
    uint8_t *buf = malloc(4096), *rb = malloc(4096);
    if (err == ESP_OK) {
        *step = "write";
        FILE *f = fopen(MOUNT "/test.bin", "wb");
        int64_t t0 = esp_timer_get_time();
        if (!f) {
            err = ESP_FAIL;
        }
        for (size_t off = 0; err == ESP_OK && off < n; off += 4096) {
            size_t len = n - off < 4096 ? n - off : 4096;
            for (size_t i = 0; i < len; i++) {
                buf[i] = pattern((uint32_t)(off / 512), (uint32_t)i, 0x3C);
            }
            if (fwrite(buf, 1, len, f) != len) {
                err = ESP_FAIL;
            }
        }
        if (f && fclose(f) != 0) {
            err = ESP_FAIL;
        }
        *wr_kbps = n / 1.024 / (double)(esp_timer_get_time() - t0) * 1000.0;
    }
    if (err == ESP_OK) {
        *step = "read";
        FILE *f = fopen(MOUNT "/test.bin", "rb");
        int64_t t0 = esp_timer_get_time();
        if (!f) {
            err = ESP_FAIL;
        }
        for (size_t off = 0; err == ESP_OK && off < n; off += 4096) {
            size_t len = n - off < 4096 ? n - off : 4096;
            if (fread(rb, 1, len, f) != len) {
                err = ESP_FAIL;
                break;
            }
            for (size_t i = 0; i < len; i++) {
                if (rb[i] != pattern((uint32_t)(off / 512), (uint32_t)i, 0x3C)) {
                    *step = "compare";
                    err = ESP_ERR_INVALID_CRC;
                    break;
                }
            }
        }
        if (f) {
            fclose(f);
        }
        *rd_kbps = n / 1.024 / (double)(esp_timer_get_time() - t0) * 1000.0;
    }
    free(buf);
    free(rb);
    esp_err_t uerr = esp_vfs_fat_sdcard_unmount(MOUNT, card);
    if (err == ESP_OK && uerr != ESP_OK) {
        *step = "unmount";
        err = uerr;
    }
    return err;
}

static void test_fat(uint32_t kb)
{
    double w = 0, r = 0;
    const char *step = "mount";
    int64_t t0 = esp_timer_get_time();
    esp_err_t err = fat_once(kb, &w, &r, &step);
    char extra[64];
    snprintf(extra, sizeof(extra), "write_kBps=%.1f read_kBps=%.1f", w, r);
    result("fat", err, err == ESP_OK ? "done" : step, ms_since(t0), extra);
}

static void test_loop(uint32_t runs, uint32_t kb)
{
    uint32_t ok = 0;
    int64_t t0 = esp_timer_get_time();
    esp_err_t first_err = ESP_OK;
    const char *first_step = "done";
    for (uint32_t i = 0; i < runs; i++) {
        double w = 0, r = 0;
        const char *step = "mount";
        esp_err_t err = fat_once(kb, &w, &r, &step);
        printf("run %lu: %s (%s) write %.1f kB/s read %.1f kB/s\n", (unsigned long)i + 1,
               esp_err_to_name(err), step, w, r);
        if (err == ESP_OK) {
            ok++;
        } else if (first_err == ESP_OK) {
            first_err = err;
            first_step = step;
        }
    }
    char extra[48];
    snprintf(extra, sizeof(extra), "runs=%lu passed=%lu", (unsigned long)runs, (unsigned long)ok);
    result("loop", first_err, first_step, ms_since(t0), extra);
}

static void set_debug(bool on)
{
    for (size_t i = 0; i < sizeof(s_sd_tags) / sizeof(s_sd_tags[0]); i++) {
        esp_log_level_set(s_sd_tags[i], on ? ESP_LOG_DEBUG : ESP_LOG_INFO);
    }
}

static void handle_line(char *line)
{
    char *argv[6];
    int argc = 0;
    for (char *t = strtok(line, " \t"); t && argc < 6; t = strtok(NULL, " \t")) {
        argv[argc++] = t;
    }
    if (argc == 0) {
        return;
    }
    const char *cmd = argv[0];
    uint32_t a1 = argc > 1 ? (uint32_t)strtoul(argv[1], NULL, 0) : 0;
    uint32_t a2 = argc > 2 ? (uint32_t)strtoul(argv[2], NULL, 0) : 0;

    if (strcmp(cmd, "info") == 0) {
        test_info();
    } else if (strcmp(cmd, "fat") == 0) {
        test_fat(a1 ? a1 : 16);
    } else if (strcmp(cmd, "raw") == 0) {
        test_raw(argc > 1 ? a1 : 2000, a2 ? a2 : 4);
    } else if (strcmp(cmd, "loop") == 0) {
        test_loop(a1 ? a1 : 5, a2 ? a2 : 16);
    } else if ((strcmp(cmd, "write") == 0 || strcmp(cmd, "read") == 0 || strcmp(cmd, "verify") == 0) && argc == 2) {
        test_sector(cmd, a1);
    } else if (strcmp(cmd, "debug") == 0 && argc == 2) {
        set_debug(strcmp(argv[1], "on") == 0);
        printf("debug %s\n", argv[1]);
    } else {
        printf("commands: info | fat [KB] | raw [LBA] [N] | loop [RUNS] [KB] |\n"
               "          write LBA | read LBA | verify LBA | debug on|off\n"
               "SD clock after init: %d kHz (menuconfig)\n", CONFIG_HOST_SD_FREQ_KHZ);
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 512, 0, 0, NULL, 0));

    spi_bus_config_t bus = {
        .mosi_io_num = CONFIG_HOST_PIN_MOSI,
        .miso_io_num = CONFIG_HOST_PIN_MISO,
        .sclk_io_num = CONFIG_HOST_PIN_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI_HOST_ID, &bus, SDSPI_DEFAULT_DMA));
    ESP_ERROR_CHECK(sdspi_host_init());
    ESP_LOGI(TAG, "SDSPI host ready: SCLK=%d MOSI=%d MISO=%d CS=%d, %d kHz after init",
             CONFIG_HOST_PIN_SCLK, CONFIG_HOST_PIN_MOSI, CONFIG_HOST_PIN_MISO, CONFIG_HOST_PIN_CS,
             CONFIG_HOST_SD_FREQ_KHZ);
    printf("type 'info' to initialize the card, 'help' for commands\n");

    char line[128];
    size_t len = 0;
    for (;;) {
        uint8_t ch;
        if (uart_read_bytes(UART_NUM_0, &ch, 1, portMAX_DELAY) <= 0) {
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
