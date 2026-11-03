/*
 * ESP32-S3 port of sdemu: bit-banged SPI slave transport on a dedicated core,
 * staged PSRAM storage, trace ring and on-chip diagnostics.
 *
 * Core split (docs/DESIGN.md section 2):
 *   core 1  transport + engine only, interrupts off, never yields, IRAM/DRAM only
 *   core 0  everything unbounded: storage worker (PSRAM), console, trace printing
 */
#pragma once

#include <stdio.h>
#include <stdbool.h>
#include "esp_err.h"
#include "sdemu.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int pin_sclk;
    int pin_mosi;
    int pin_miso;
    int pin_cs;
    int core;                 /* core that runs the card loop (1) */
    uint8_t *disk;            /* capacity_blocks * 512 bytes, PSRAM is fine */
    sdemu_config_t card;      /* storage/clock/trace are filled in by the port */
    uint32_t cs_filter;       /* identical CS reads before a change counts (16) */
} sdemu_esp_config_t;

#define SDEMU_ESP_SLIP_LOG 8

typedef struct {
    uint32_t windows;           /* CS-low windows */
    uint32_t bytes;             /* complete bytes */
    uint32_t partial_windows;   /* windows that ended mid-byte */
    uint32_t worst_rx_cycles;   /* sdemu_rx() at a byte boundary */
    uint32_t worst_tx_cycles;   /* sdemu_tx() in the middle of a byte */
    uint32_t worst_select_cycles;
    uint32_t cs_spikes;         /* CS changes rejected by the filter */
    uint32_t slips;             /* windows where PCNT and software disagreed */
    uint32_t slip_byte[SDEMU_ESP_SLIP_LOG];   /* byte index of the first mismatch */
    uint32_t idle_iters;        /* idle loop iterations */
    uint32_t stalls;            /* idle loop gaps > 1 us (core 1 paused) */
    uint32_t worst_stall_cycles;
    uint32_t trace_dropped;
    uint32_t cpu_mhz;
} sdemu_esp_diag_t;

esp_err_t sdemu_esp_start(const sdemu_esp_config_t *cfg);
sdemu_t *sdemu_esp_card(void);

/* trace events, drained by core 0 */
bool sdemu_esp_trace_pop(sdemu_event_t *ev);

/* diagnostics */
void sdemu_esp_get_diag(sdemu_esp_diag_t *out);
void sdemu_esp_reset_diag(void);
void sdemu_esp_set_cs_filter(uint32_t reads);
void sdemu_esp_set_slipcheck(bool on);

/* requests executed by core 1 between CS windows */
void sdemu_esp_power_cycle(void);

/* capture mode: core 1 records every pin change of the next `windows` CS
 * windows instead of acting as a card */
esp_err_t sdemu_esp_capture_arm(uint32_t windows);
bool sdemu_esp_capture_done(void);
void sdemu_esp_capture_dump(FILE *out);

/* pin sampler (runs on the calling core, ~200 ms) */
void sdemu_esp_pins(FILE *out);

#ifdef __cplusplus
}
#endif
