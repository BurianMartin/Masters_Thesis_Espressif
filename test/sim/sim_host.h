/*
 * A model of what the ESP-IDF SDSPI host (v5.5, components/esp_driver_sdspi +
 * components/sdmmc) does on the wire, byte for byte, driving the engine
 * through the same one-byte pipeline as the S3 transport.
 *
 * Simulated time advances 20 us per byte (400 kHz).
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "sdemu.h"

enum {
    SIM_OK = 0,
    SIM_ERR_TIMEOUT,
    SIM_ERR_INVALID_CRC,
    SIM_ERR_INVALID_RESPONSE,
    SIM_ERR_NOT_SUPPORTED,
    SIM_ERR_INVALID_ARG,
    SIM_ERR_FAIL,
};

typedef struct {
    sdemu_t *card;
    uint32_t t_us;
    bool selected;
    uint8_t cur;
    bool crc_enabled;      /* host verifies read-data CRC16 after CMD59 */
    bool verbose;
    uint32_t bytes;
    uint8_t last_r1;
    uint32_t cmd13_failures;   /* logged-only failures, like ESP-IDF */
    uint16_t write_crc_xor;    /* test hook: corrupt the CRC16 of the next written block */
    uint32_t corrections;      /* data responses corrected by rx() */
    uint32_t pipeline_violations;
} sim_host_t;

typedef struct {
    uint8_t cid[16];
    uint8_t csd[16];
    uint8_t scr[8];
    uint32_t ocr;
    uint32_t capacity_blocks;
    bool sdhc;
    char name[6];
} sim_card_info_t;

void sim_host_init(sim_host_t *h, sdemu_t *card);
uint32_t sim_clock(void *ctx);
const char *sim_err_name(int err);

void sim_delay_us(sim_host_t *h, uint32_t us);
void sim_cs(sim_host_t *h, bool low);
uint8_t sim_byte(sim_host_t *h, uint8_t mosi);

/* One command transaction (CS must be low). resp gets R1 + up to 4 bytes. */
int sim_cmd(sim_host_t *h, uint8_t idx, uint32_t arg, int resp_len, uint8_t *resp, bool bad_crc);

int sim_card_init(sim_host_t *h, sim_card_info_t *info);
int sim_send_status(sim_host_t *h, uint8_t *r2);
int sim_read_blocks(sim_host_t *h, uint32_t lba, uint32_t n, uint8_t *buf);
int sim_write_blocks(sim_host_t *h, uint32_t lba, uint32_t n, const uint8_t *buf);
