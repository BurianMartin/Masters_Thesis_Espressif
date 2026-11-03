/*
 * ESP32-S3 transport for sdemu: SPI mode 0 slave in software on core 1.
 *
 * Why software: the GP-SPI slave works in pre-arranged transactions, but an
 * SD card's answer depends on the command it is still receiving. Here the
 * CPU is the shift register, so it can react at any byte. The price is that
 * every clock edge passes through CPU instructions: ~600 cycles per bit at
 * 400 kHz and 240 MHz. See docs/DESIGN.md sections 2-4.
 *
 * Rules for everything reachable from card_loop():
 *   - IRAM/DRAM only (the component's linker fragment places it there):
 *     a flash or PSRAM cache miss costs a whole bit
 *   - no esp_timer, no logging, no FreeRTOS calls, interrupts are off
 *   - bounded work per call into the engine
 */
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"
#include "esp_cpu.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "driver/gpio.h"
#include "driver/dedic_gpio.h"
#include "driver/pulse_cnt.h"
#include "hal/dedic_gpio_cpu_ll.h"
#include "sdkconfig.h"
#include "sdemu_esp.h"

static const char *TAG = "sdemu_esp";

/* ------------------------------------------------------------------------ */
/* state                                                                     */
/* ------------------------------------------------------------------------ */

static sdemu_esp_config_t s_cfg;
static sdemu_t s_card;
static sdemu_storage_t s_storage;
static volatile sdemu_esp_diag_t s_diag;
static volatile bool s_started;

/* dedicated GPIO channel masks (bit positions depend on allocation) */
static uint32_t s_m_sclk, s_m_mosi, s_m_cs, s_sh_mosi, s_m_miso;

static pcnt_unit_handle_t s_pcnt;
static volatile bool s_slipcheck = true;
static volatile uint32_t s_cs_filter = 16;

/* requests from core 0, handled by core 1 between windows */
static volatile bool s_req_power_cycle;
static volatile bool s_req_reset_diag;

/* ------------------------------------------------------------------------ */
/* clock: CPU cycle counter, extended to microseconds (core 1 only)          */
/* ------------------------------------------------------------------------ */

static uint32_t s_cpu_mhz;
static uint32_t s_clk_last, s_clk_rem, s_clk_us;

static uint32_t IRAM_ATTR clock_us(void *ctx)
{
    (void)ctx;
    uint32_t cc = esp_cpu_get_cycle_count();
    uint32_t d = (cc - s_clk_last) + s_clk_rem;
    s_clk_last = cc;
    s_clk_us += d / s_cpu_mhz;
    s_clk_rem = d % s_cpu_mhz;
    return s_clk_us;
}

/* ------------------------------------------------------------------------ */
/* trace ring: single producer (core 1), single consumer (core 0)            */
/* ------------------------------------------------------------------------ */

#define TRACE_N 512
static sdemu_event_t s_ring[TRACE_N];
static volatile uint32_t s_ring_head, s_ring_tail;

static void IRAM_ATTR trace_push(void *ctx, const sdemu_event_t *ev)
{
    (void)ctx;
    uint32_t h = s_ring_head;
    if (h - __atomic_load_n(&s_ring_tail, __ATOMIC_ACQUIRE) >= TRACE_N) {
        s_diag.trace_dropped++;
        return;
    }
    s_ring[h % TRACE_N] = *ev;
    __atomic_store_n(&s_ring_head, h + 1, __ATOMIC_RELEASE);
}

bool sdemu_esp_trace_pop(sdemu_event_t *ev)
{
    uint32_t t = s_ring_tail;
    if (t == __atomic_load_n(&s_ring_head, __ATOMIC_ACQUIRE)) {
        return false;
    }
    *ev = s_ring[t % TRACE_N];
    __atomic_store_n(&s_ring_tail, t + 1, __ATOMIC_RELEASE);
    return true;
}

/* ------------------------------------------------------------------------ */
/* staged storage: core 1 only touches two internal buffers, a worker on     */
/* core 0 copies whole blocks to and from the PSRAM disk (DESIGN.md 3.5)     */
/* ------------------------------------------------------------------------ */

typedef struct {
    uint8_t *disk;
    uint32_t blocks;
    /* read slot */
    volatile uint32_t rd_req;       /* 1 = worker should load rd_req_lba */
    volatile uint32_t rd_req_lba;
    volatile uint32_t rd_valid;
    volatile uint32_t rd_lba;
    uint8_t rd_buf[SDEMU_BLOCK_SIZE];
    /* write slot */
    volatile uint32_t wr_state;     /* 0 idle, 1 commit requested, 2 done ok, 3 done error */
    volatile uint32_t wr_lba;
    uint8_t wr_buf[SDEMU_BLOCK_SIZE];
} staging_t;

static DRAM_ATTR staging_t s_stage;

static void IRAM_ATTR stage_request(staging_t *s, uint32_t lba)
{
    s->rd_valid = 0;
    s->rd_req_lba = lba;
    __atomic_store_n(&s->rd_req, 1, __ATOMIC_RELEASE);
}

static sdemu_io_t IRAM_ATTR stage_read(void *ctx, uint32_t lba, uint32_t off, uint8_t *dst, uint32_t len)
{
    staging_t *s = ctx;
    bool pending = __atomic_load_n(&s->rd_req, __ATOMIC_ACQUIRE) != 0;
    if (!pending && s->rd_valid && s->rd_lba == lba) {
        memcpy(dst, &s->rd_buf[off], len);
        if (off + len == SDEMU_BLOCK_SIZE && lba + 1 < s->blocks) {
            stage_request(s, lba + 1);   /* prefetch for CMD18 */
        }
        return SDEMU_IO_OK;
    }
    if (!pending || s->rd_req_lba != lba) {
        stage_request(s, lba);
    }
    return SDEMU_IO_BUSY;
}

static sdemu_io_t IRAM_ATTR stage_write(void *ctx, uint32_t lba, uint32_t off, const uint8_t *src, uint32_t len)
{
    staging_t *s = ctx;
    (void)lba;
    if (__atomic_load_n(&s->wr_state, __ATOMIC_ACQUIRE) == 1) {
        return SDEMU_IO_BUSY;
    }
    memcpy(&s->wr_buf[off], src, len);
    return SDEMU_IO_OK;
}

static sdemu_io_t IRAM_ATTR stage_commit(void *ctx, uint32_t lba)
{
    staging_t *s = ctx;
    switch (__atomic_load_n(&s->wr_state, __ATOMIC_ACQUIRE)) {
    case 0:
        if (s->rd_lba == lba) {
            s->rd_valid = 0;   /* the cached copy is about to be stale */
        }
        s->wr_lba = lba;
        __atomic_store_n(&s->wr_state, 1, __ATOMIC_RELEASE);
        return SDEMU_IO_BUSY;
    case 1:
        return SDEMU_IO_BUSY;
    case 2:
        __atomic_store_n(&s->wr_state, 0, __ATOMIC_RELEASE);
        return SDEMU_IO_OK;
    default:
        __atomic_store_n(&s->wr_state, 0, __ATOMIC_RELEASE);
        return SDEMU_IO_ERROR;
    }
}

static void storage_worker(void *arg)
{
    staging_t *s = arg;
    for (;;) {
        bool did = false;
        if (__atomic_load_n(&s->wr_state, __ATOMIC_ACQUIRE) == 1) {   /* writes before reads */
            uint32_t lba = s->wr_lba;
            bool ok = lba < s->blocks;
            if (ok) {
                memcpy(s->disk + (size_t)lba * SDEMU_BLOCK_SIZE, s->wr_buf, SDEMU_BLOCK_SIZE);
            }
            __atomic_store_n(&s->wr_state, ok ? 2 : 3, __ATOMIC_RELEASE);
            did = true;
        }
        if (__atomic_load_n(&s->rd_req, __ATOMIC_ACQUIRE)) {
            uint32_t lba = s->rd_req_lba;
            s->rd_valid = 0;
            if (lba < s->blocks) {
                memcpy(s->rd_buf, s->disk + (size_t)lba * SDEMU_BLOCK_SIZE, SDEMU_BLOCK_SIZE);
            }
            s->rd_lba = lba;
            __atomic_store_n(&s->rd_valid, 1, __ATOMIC_RELEASE);
            __atomic_store_n(&s->rd_req, 0, __ATOMIC_RELEASE);
            did = true;
        }
        if (!did) {
            vTaskDelay(1);   /* 1 ms with CONFIG_FREERTOS_HZ=1000 */
        }
    }
}

/* ------------------------------------------------------------------------ */
/* pins                                                                      */
/* ------------------------------------------------------------------------ */

static inline uint32_t IRAM_ATTR pins_read(void)
{
    return dedic_gpio_cpu_ll_read_in();
}

static inline void IRAM_ATTR miso_set(uint32_t bit)
{
    dedic_gpio_cpu_ll_write_mask(s_m_miso, bit ? s_m_miso : 0);
}

/* CS reads high: only believe it after s_cs_filter identical reads */
static bool IRAM_ATTR cs_high_confirmed(void)
{
    uint32_t n = s_cs_filter;
    for (uint32_t i = 0; i < n; i++) {
        if (!(pins_read() & s_m_cs)) {
            s_diag.cs_spikes++;
            return false;
        }
    }
    return true;
}

/* ------------------------------------------------------------------------ */
/* capture mode                                                              */
/* ------------------------------------------------------------------------ */

#define CAP_N 4096
static uint32_t s_cap_cc[CAP_N];
static uint8_t s_cap_pins[CAP_N];
static volatile uint32_t s_cap_len;
static volatile uint32_t s_cap_armed;    /* windows still to capture */
static volatile bool s_cap_done;

static void IRAM_ATTR capture_window(void)
{
    uint32_t n = s_cap_len;
    uint32_t last = 0xFFFFFFFF;
    uint32_t high_run = 0;
    while (n < CAP_N) {
        uint32_t v = pins_read();
        uint32_t cc = esp_cpu_get_cycle_count();
        uint32_t p = ((v & s_m_sclk) ? 1 : 0) | ((v & s_m_mosi) ? 2 : 0) | ((v & s_m_cs) ? 4 : 0);
        if (p != last) {
            s_cap_cc[n] = cc;
            s_cap_pins[n] = (uint8_t)p;
            n++;
            last = p;
        }
        if (p & 4) {
            if (++high_run >= 64) {
                break;
            }
        } else {
            high_run = 0;
        }
    }
    s_cap_len = n;
    if (--s_cap_armed == 0 || n >= CAP_N) {
        s_cap_armed = 0;
        s_cap_done = true;
    }
}

esp_err_t sdemu_esp_capture_arm(uint32_t windows)
{
    if (!s_started || windows == 0) {
        return ESP_ERR_INVALID_STATE;
    }
    s_cap_len = 0;
    s_cap_done = false;
    __atomic_store_n(&s_cap_armed, windows, __ATOMIC_RELEASE);
    return ESP_OK;
}

bool sdemu_esp_capture_done(void)
{
    return s_cap_done;
}

void sdemu_esp_capture_dump(FILE *out)
{
    uint32_t n = s_cap_len;
    if (!s_cap_done) {
        fprintf(out, "capture: not finished (%lu samples so far)\n", (unsigned long)n);
        return;
    }
    fprintf(out, "capture: %lu transitions (cycles @ %lu MHz; pins S=SCLK M=MOSI C=CS)\n",
            (unsigned long)n, (unsigned long)s_cpu_mhz);
    uint32_t min_period = UINT32_MAX, max_period = 0, last_rise = 0, cs_to_clk = 0;
    uint32_t cs_low_at = 0;
    uint8_t byte = 0;
    int bits = 0;
    fprintf(out, "MOSI bytes:");
    for (uint32_t i = 0; i < n; i++) {
        uint8_t p = s_cap_pins[i], prev = i ? s_cap_pins[i - 1] : 0x04;
        if (!(p & 4) && (prev & 4)) {
            cs_low_at = s_cap_cc[i];
            bits = 0;
            last_rise = 0;
            fprintf(out, "\n  [CS low]");
        }
        if ((p & 1) && !(prev & 1) && !(p & 4)) {   /* rising SCLK while selected */
            if (last_rise) {
                uint32_t per = s_cap_cc[i] - last_rise;
                min_period = per < min_period ? per : min_period;
                max_period = per > max_period ? per : max_period;
            } else if (cs_low_at) {
                cs_to_clk = s_cap_cc[i] - cs_low_at;
            }
            last_rise = s_cap_cc[i];
            byte = (uint8_t)((byte << 1) | ((p & 2) ? 1 : 0));
            if (++bits == 8) {
                fprintf(out, " %02x", byte);
                bits = 0;
            }
        }
    }
    fprintf(out, "\n");
    if (max_period) {
        fprintf(out, "SCLK period: %lu..%lu ns, CS low -> first edge: %lu ns\n",
                (unsigned long)(min_period * 1000 / s_cpu_mhz), (unsigned long)(max_period * 1000 / s_cpu_mhz),
                (unsigned long)(cs_to_clk * 1000 / s_cpu_mhz));
    }
    uint32_t show = n < 200 ? n : 200;
    fprintf(out, "first %lu transitions (dt in cycles):\n", (unsigned long)show);
    for (uint32_t i = 0; i < show; i++) {
        uint8_t p = s_cap_pins[i];
        fprintf(out, "  %6lu  S%d M%d C%d\n", (unsigned long)(i ? s_cap_cc[i] - s_cap_cc[i - 1] : 0),
                p & 1, (p >> 1) & 1, (p >> 2) & 1);
    }
}

/* ------------------------------------------------------------------------ */
/* the card loop (core 1, interrupts off, never returns)                     */
/* ------------------------------------------------------------------------ */

static inline void IRAM_ATTR worst(volatile uint32_t *slot, uint32_t v)
{
    if (v > *slot) {
        *slot = v;
    }
}

static void IRAM_ATTR handle_requests(void)
{
    if (s_req_power_cycle) {
        sdemu_power_cycle(&s_card);
        s_req_power_cycle = false;
    }
    if (s_req_reset_diag) {
        uint32_t mhz = s_diag.cpu_mhz;
        memset((void *)&s_diag, 0, sizeof(s_diag));
        s_diag.cpu_mhz = mhz;
        s_req_reset_diag = false;
    }
}

/* wait for a filtered CS low; meanwhile measure the idle loop */
static void IRAM_ATTR wait_selected(void)
{
    uint32_t low_run = 0;
    uint32_t prev = esp_cpu_get_cycle_count();
    uint32_t iters = 0;
    for (;;) {
        uint32_t v = pins_read();
        uint32_t now = esp_cpu_get_cycle_count();
        uint32_t gap = now - prev;
        prev = now;
        if (gap > s_cpu_mhz) {        /* > 1 us between iterations: core 1 was paused */
            s_diag.stalls++;
            worst(&s_diag.worst_stall_cycles, gap);
        }
        if (!(v & s_m_cs)) {
            if (++low_run >= s_cs_filter) {
                s_diag.idle_iters += iters;
                return;
            }
            continue;
        }
        if (low_run) {
            s_diag.cs_spikes++;
        }
        low_run = 0;
        if ((++iters & 0xFFFF) == 0) {
            s_diag.idle_iters += iters;
            iters = 0;
            clock_us(NULL);          /* keep the cycle-counter clock from wrapping */
            handle_requests();
            prev = esp_cpu_get_cycle_count();
        }
    }
}

static void IRAM_ATTR card_loop(void)
{
    const uint32_t m_sclk = s_m_sclk, m_mosi = s_m_mosi, m_cs = s_m_cs, sh_mosi = s_sh_mosi;
    for (;;) {
        handle_requests();
        wait_selected();

        if (s_cap_armed) {
            capture_window();
            continue;
        }

        uint32_t t0 = esp_cpu_get_cycle_count();
        if (s_slipcheck) {
            pcnt_unit_clear_count(s_pcnt);
        }
        sdemu_select(&s_card);
        uint8_t cur = sdemu_tx(&s_card);
        uint8_t next = 0xFF;
        miso_set(cur & 0x80);
        worst(&s_diag.worst_select_cycles, esp_cpu_get_cycle_count() - t0);

        uint32_t in = 0, bit = 0, edges = 0, bytes = 0;
        bool check = s_slipcheck;
        for (;;) {
            uint32_t v;
            /* rising edge: sample MOSI */
            while (!((v = pins_read()) & m_sclk)) {
                if ((v & m_cs) && cs_high_confirmed()) {
                    goto deselected;
                }
            }
            in = (in << 1) | ((v >> sh_mosi) & 1);
            edges++;
            if (++bit == 8) {
                /* byte boundary: next byte's bit 7 goes out first, then rx() */
                cur = next;
                miso_set(cur & 0x80);
                uint32_t a = esp_cpu_get_cycle_count();
                int fix = sdemu_rx(&s_card, (uint8_t)in);
                if (fix >= 0) {
                    cur = (uint8_t)fix;   /* same bit 7 by construction */
                }
                worst(&s_diag.worst_rx_cycles, esp_cpu_get_cycle_count() - a);
                bit = 0;
                in = 0;
                bytes++;
                if (check && edges < 32767) {
                    int cnt = 0;
                    pcnt_unit_get_count(s_pcnt, &cnt);
                    if ((uint32_t)cnt != edges) {
                        uint32_t k = s_diag.slips++;
                        if (k < SDEMU_ESP_SLIP_LOG) {
                            s_diag.slip_byte[k] = bytes - 1;
                        }
                        check = false;   /* one entry per window */
                    }
                }
            } else {
                miso_set((uint32_t)(cur << bit) & 0x80);
                if (bit == 4) {
                    /* middle of the byte: ask for the one after it */
                    uint32_t a = esp_cpu_get_cycle_count();
                    next = sdemu_tx(&s_card);
                    worst(&s_diag.worst_tx_cycles, esp_cpu_get_cycle_count() - a);
                }
            }
            /* falling edge */
            while ((v = pins_read()) & m_sclk) {
                if ((v & m_cs) && cs_high_confirmed()) {
                    goto deselected;
                }
            }
        }
deselected:
        miso_set(1);
        sdemu_deselect(&s_card);
        s_diag.windows++;
        s_diag.bytes += bytes;
        if (bit) {
            s_diag.partial_windows++;
        }
    }
}

static void card_task(void *arg)
{
    TaskHandle_t parent = arg;

    /* dedicated GPIO bundles belong to the CPU that creates them: create here */
    const int in_pins[3] = { s_cfg.pin_sclk, s_cfg.pin_mosi, s_cfg.pin_cs };
    const int out_pins[1] = { s_cfg.pin_miso };
    dedic_gpio_bundle_handle_t in_b = NULL, out_b = NULL;
    dedic_gpio_bundle_config_t in_cfg = { .gpio_array = in_pins, .array_size = 3, .flags = { .in_en = 1 } };
    dedic_gpio_bundle_config_t out_cfg = { .gpio_array = out_pins, .array_size = 1, .flags = { .out_en = 1 } };
    ESP_ERROR_CHECK(dedic_gpio_new_bundle(&in_cfg, &in_b));
    ESP_ERROR_CHECK(dedic_gpio_new_bundle(&out_cfg, &out_b));
    uint32_t in_off = 0, out_off = 0;
    ESP_ERROR_CHECK(dedic_gpio_get_in_offset(in_b, &in_off));
    ESP_ERROR_CHECK(dedic_gpio_get_out_offset(out_b, &out_off));
    s_m_sclk = 1u << (in_off + 0);
    s_m_mosi = 1u << (in_off + 1);
    s_sh_mosi = in_off + 1;
    s_m_cs = 1u << (in_off + 2);
    s_m_miso = 1u << out_off;
    miso_set(1);

    s_cpu_mhz = (uint32_t)CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
    s_diag.cpu_mhz = s_cpu_mhz;
    s_clk_last = esp_cpu_get_cycle_count();   /* the cycle counter is per core: anchor it here */
    s_started = true;
    xTaskNotifyGive(parent);

    portDISABLE_INTERRUPTS();
    card_loop();
}

esp_err_t sdemu_esp_start(const sdemu_esp_config_t *cfg)
{
    if (s_started) {
        return ESP_ERR_INVALID_STATE;
    }
    s_cfg = *cfg;
    s_cs_filter = cfg->cs_filter ? cfg->cs_filter : 16;

    /* pins: CS and MOSI pulled up, SCLK driven by the host, MISO output high */
    gpio_config_t in = {
        .pin_bit_mask = (1ULL << cfg->pin_cs) | (1ULL << cfg->pin_mosi) | (1ULL << cfg->pin_sclk),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&in));
    gpio_set_pull_mode(cfg->pin_sclk, GPIO_FLOATING);
    gpio_config_t out = {
        .pin_bit_mask = 1ULL << cfg->pin_miso,
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&out));
    gpio_set_level(cfg->pin_miso, 1);

    /* PCNT: counts SCLK rising edges while CS is low, independently of the CPU */
    pcnt_unit_config_t ucfg = { .low_limit = -1, .high_limit = 32767 };
    ESP_ERROR_CHECK(pcnt_new_unit(&ucfg, &s_pcnt));
    pcnt_chan_config_t ccfg = { .edge_gpio_num = cfg->pin_sclk, .level_gpio_num = cfg->pin_cs };
    pcnt_channel_handle_t ch;
    ESP_ERROR_CHECK(pcnt_new_channel(s_pcnt, &ccfg, &ch));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(ch, PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                                                 PCNT_CHANNEL_EDGE_ACTION_HOLD));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(ch, PCNT_CHANNEL_LEVEL_ACTION_HOLD,    /* CS high */
                                                  PCNT_CHANNEL_LEVEL_ACTION_KEEP));  /* CS low */
    ESP_ERROR_CHECK(pcnt_unit_enable(s_pcnt));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(s_pcnt));
    ESP_ERROR_CHECK(pcnt_unit_start(s_pcnt));

    /* storage worker on core 0 */
    s_stage.disk = cfg->disk;
    s_stage.blocks = cfg->card.capacity_blocks;
    s_storage.ctx = &s_stage;
    s_storage.read = stage_read;
    s_storage.write = stage_write;
    s_storage.commit = stage_commit;
    xTaskCreatePinnedToCore(storage_worker, "sdemu_store", 3072, &s_stage, 10, NULL, 0);

    /* the engine */
    sdemu_config_t card = cfg->card;
    card.storage = &s_storage;
    card.now_us = clock_us;
    card.clock_ctx = NULL;
    card.trace = trace_push;
    card.trace_ctx = NULL;
    sdemu_init(&s_card, &card);

    xTaskCreatePinnedToCore(card_task, "sdemu_card", 4096, xTaskGetCurrentTaskHandle(),
                            configMAX_PRIORITIES - 1, NULL, cfg->core);
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
    if (!s_started) {
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "card armed on core %d: SCLK=%d MOSI=%d MISO=%d CS=%d", cfg->core,
             cfg->pin_sclk, cfg->pin_mosi, cfg->pin_miso, cfg->pin_cs);
    return ESP_OK;
}

sdemu_t *sdemu_esp_card(void)
{
    return &s_card;
}

void sdemu_esp_get_diag(sdemu_esp_diag_t *out)
{
    memcpy(out, (const void *)&s_diag, sizeof(*out));
}

void sdemu_esp_reset_diag(void)
{
    s_req_reset_diag = true;
}

void sdemu_esp_set_cs_filter(uint32_t reads)
{
    s_cs_filter = reads ? reads : 1;
}

void sdemu_esp_set_slipcheck(bool on)
{
    s_slipcheck = on;
}

void sdemu_esp_power_cycle(void)
{
    s_req_power_cycle = true;
}

void sdemu_esp_pins(FILE *out)
{
    const int pins[4] = { s_cfg.pin_sclk, s_cfg.pin_mosi, s_cfg.pin_miso, s_cfg.pin_cs };
    const char *names[4] = { "SCLK", "MOSI", "MISO", "CS" };
    uint32_t lows[4] = { 0 }, edges[4] = { 0 };
    int last[4];
    const uint32_t samples = 20000;
    for (int p = 0; p < 4; p++) {
        last[p] = gpio_get_level(pins[p]);
    }
    for (uint32_t i = 0; i < samples; i++) {
        for (int p = 0; p < 4; p++) {
            int l = gpio_get_level(pins[p]);
            lows[p] += l == 0;
            edges[p] += l != last[p];
            last[p] = l;
        }
        esp_rom_delay_us(10);
    }
    fprintf(out, "pins over ~%lu ms (%lu samples):\n", (unsigned long)(samples * 10 / 1000), (unsigned long)samples);
    for (int p = 0; p < 4; p++) {
        fprintf(out, "  %-4s GPIO%-2d low %5.1f %%  changes %lu\n", names[p], pins[p],
                100.0 * lows[p] / samples, (unsigned long)edges[p]);
    }
}
