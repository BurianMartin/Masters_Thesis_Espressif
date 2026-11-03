/*
 * Laptop test bench: the engine against the simulated ESP-IDF host.
 * Catches protocol logic errors in seconds. It cannot catch timing problems;
 * those need the hardware (docs/DESIGN.md section 8).
 *
 *   make test          all groups
 *   ./sim_test -v 3    one group, verbose host log
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sim_host.h"
#include "../../components/sdemu/src/sdemu_internal.h"

#define CAPACITY 12288u

static int s_fail, s_checks;
static bool s_verbose;

#define CHECK(cond, ...) do { \
    s_checks++; \
    if (!(cond)) { s_fail++; printf("  FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } \
} while (0)

#define CHECK_ERR(expr, want) do { int got_ = (expr); \
    CHECK(got_ == (want), "got %s, want %s", sim_err_name(got_), sim_err_name(want)); } while (0)

/* ---- fixture ------------------------------------------------------------- */

static uint8_t *s_disk;
static sdemu_ram_backend_t s_ram;
static sdemu_storage_t s_storage;
static sdemu_t s_card;
static sim_host_t s_host;
static int s_events[16];

static void on_trace(void *ctx, const sdemu_event_t *ev)
{
    (void)ctx;
    if (ev->type < 16) {
        s_events[ev->type]++;
    }
    if (s_verbose) {
        printf("    card: t=%u type=%u cmd=%u arg=%08x r1=%02x x=%02x\n",
               ev->t_us, ev->type, ev->cmd, ev->arg, ev->r1, ev->extra);
    }
}

static void setup(void)
{
    sdemu_config_t cfg;
    memset(s_disk, 0, (size_t)CAPACITY * SDEMU_BLOCK_SIZE);
    memset(s_events, 0, sizeof(s_events));
    sdemu_config_default(&cfg);
    cfg.capacity_blocks = CAPACITY;
    sdemu_ram_backend_init(&s_ram, &s_storage, s_disk, CAPACITY);
    cfg.storage = &s_storage;
    cfg.now_us = sim_clock;
    cfg.clock_ctx = &s_host;
    cfg.trace = on_trace;
    sdemu_init(&s_card, &cfg);
    sim_host_init(&s_host, &s_card);
    s_host.verbose = s_verbose;
}

static void fault(const char *text)
{
    sdemu_fault_rule_t r;
    char err[64];
    bool ok = sdemu_fault_parse(text, &r, err, sizeof(err));
    CHECK(ok, "parse '%s': %s", text, err);
    if (ok) {
        CHECK(sdemu_faults_add(&s_card, &r), "add rule");
    }
}

static void fill(uint8_t *buf, size_t n, uint32_t seed)
{
    for (size_t i = 0; i < n; i++) {
        buf[i] = (uint8_t)(seed * 31u + i * 7u + (i >> 9));
    }
}

static bool init_ok(void)
{
    sim_card_info_t info;
    int err = sim_card_init(&s_host, &info);
    CHECK_ERR(err, SIM_OK);
    return err == SIM_OK;
}

/* ---- tests --------------------------------------------------------------- */

static void t0_crc(void)
{
    const uint8_t cmd0[5] = { 0x40, 0, 0, 0, 0 };
    const uint8_t cmd8[5] = { 0x48, 0, 0, 0x01, 0xAA };
    CHECK(sdemu_crc7(cmd0, 5) == 0x4A, "crc7(CMD0) = %02x", sdemu_crc7(cmd0, 5));
    CHECK(sdemu_crc7(cmd8, 5) == 0x43, "crc7(CMD8) = %02x", sdemu_crc7(cmd8, 5));
    uint8_t ff[512];
    memset(ff, 0xFF, sizeof(ff));
    CHECK(sdemu_crc16(ff, 512) == 0x7FA1, "crc16(512 x FF) = %04x", sdemu_crc16(ff, 512));
    for (int i = 0; i < 256; i++) {   /* table against the bitwise definition */
        uint16_t c = (uint16_t)(i << 8);
        for (int b = 0; b < 8; b++) {
            c = (uint16_t)((c & 0x8000) ? ((c << 1) ^ 0x1021) : (c << 1));
        }
        if (c != sdemu_crc16_table[i]) {
            CHECK(0, "crc16 table[%d] = %04x, want %04x", i, sdemu_crc16_table[i], c);
            break;
        }
    }
}

static void t1_init(void)
{
    sim_card_info_t info;
    int err = sim_card_init(&s_host, &info);
    CHECK_ERR(err, SIM_OK);
    CHECK(info.sdhc, "CCS not set");
    CHECK(info.capacity_blocks == CAPACITY, "capacity %u", info.capacity_blocks);
    CHECK(strcmp(info.name, "SDEMU") == 0, "name '%s'", info.name);
    CHECK((info.ocr & (1u << 24)) == 0, "S18A set");
    CHECK(s_card.crc_on, "CRC not enabled by CMD59");
    printf("  init took %u ms of simulated time\n", s_host.t_us / 1000);
}

static void t2_single(void)
{
    uint8_t w[512], r[512];
    if (!init_ok()) return;
    fill(w, 512, 1);
    CHECK_ERR(sim_write_blocks(&s_host, 100, 1, w), SIM_OK);
    CHECK(memcmp(s_disk + 100 * 512, w, 512) == 0, "block not stored");
    CHECK_ERR(sim_read_blocks(&s_host, 100, 1, r), SIM_OK);
    CHECK(memcmp(w, r, 512) == 0, "read back differs");
    CHECK(s_host.cmd13_failures == 0, "CMD13 failures");
}

static void t3_multi(void)
{
    static uint8_t w[8 * 512], r[8 * 512];
    if (!init_ok()) return;
    fill(w, sizeof(w), 2);
    CHECK_ERR(sim_write_blocks(&s_host, 200, 8, w), SIM_OK);
    CHECK(memcmp(s_disk + 200 * 512, w, sizeof(w)) == 0, "blocks not stored");
    CHECK_ERR(sim_read_blocks(&s_host, 200, 8, r), SIM_OK);
    CHECK(memcmp(w, r, sizeof(w)) == 0, "read back differs");
    /* still in sync afterwards */
    CHECK_ERR(sim_read_blocks(&s_host, 203, 1, r), SIM_OK);
    CHECK(memcmp(w + 3 * 512, r, 512) == 0, "single read after multi differs");
    CHECK_ERR(sim_send_status(&s_host, NULL), SIM_OK);
}

static void t4_write_error(void)
{
    uint8_t w[512], old[512];
    sdemu_fault_rule_t r;
    char err[64];
    /* parser sanity */
    CHECK(!sdemu_fault_parse("cmd=24", &r, err, sizeof(err)), "missing action accepted");
    CHECK(!sdemu_fault_parse("action=bogus", &r, err, sizeof(err)), "bad action accepted");
    CHECK(sdemu_fault_parse("action=flip_byte lba=10-20 offset=3", &r, err, sizeof(err)) &&
          r.lba_first == 10 && r.lba_last == 20 && r.param == 3 && r.param2 == 0xFF, "flip_byte parse");

    if (!init_ok()) return;
    fault("action=write_error cmd=24 nth=2");
    fill(old, 512, 9);
    memcpy(s_disk + 301 * 512, old, 512);
    fill(w, 512, 3);
    CHECK_ERR(sim_write_blocks(&s_host, 300, 1, w), SIM_OK);
    CHECK_ERR(sim_write_blocks(&s_host, 301, 1, w), SIM_ERR_FAIL);
    CHECK(memcmp(s_disk + 301 * 512, old, 512) == 0, "rejected block was stored");
    CHECK_ERR(sim_write_blocks(&s_host, 302, 1, w), SIM_OK);
    CHECK(memcmp(s_disk + 302 * 512, w, 512) == 0, "3rd write not stored");
    sdemu_fault_table_t t;
    sdemu_faults_get(&s_card, &t);
    CHECK(t.count == 1 && t.rules[0].matches == 3 && t.rules[0].fires == 1,
          "counters matches=%u fires=%u", t.rules[0].matches, t.rules[0].fires);
}

static void t5_read_crc(void)
{
    uint8_t r[512];
    if (!init_ok()) return;
    fault("action=corrupt_crc16 lba=400 every=1");
    CHECK_ERR(sim_read_blocks(&s_host, 400, 1, r), SIM_ERR_INVALID_CRC);
    CHECK_ERR(sim_read_blocks(&s_host, 401, 1, r), SIM_OK);
    CHECK_ERR(sim_read_blocks(&s_host, 400, 1, r), SIM_ERR_INVALID_CRC);
}

static void t6_response_timing(void)
{
    if (!init_ok()) return;
    fault("action=no_response cmd=13 nth=1");
    CHECK_ERR(sim_send_status(&s_host, NULL), SIM_ERR_TIMEOUT);
    CHECK_ERR(sim_send_status(&s_host, NULL), SIM_OK);

    sdemu_faults_clear(&s_card);
    fault("action=delay_response cmd=13 bytes=6");
    CHECK_ERR(sim_send_status(&s_host, NULL), SIM_OK);      /* R1 at the 9th byte: last allowed */

    sdemu_faults_clear(&s_card);
    fault("action=delay_response cmd=13 bytes=9");
    CHECK_ERR(sim_send_status(&s_host, NULL), SIM_ERR_TIMEOUT);
    sdemu_faults_clear(&s_card);
    CHECK_ERR(sim_send_status(&s_host, NULL), SIM_OK);
}

static void t7_busy_and_delay(void)
{
    uint8_t w[512], r[512];
    if (!init_ok()) return;
    fault("action=busy cmd=24 ms=50");
    fault("action=read_delay cmd=17 ms=30");
    fill(w, 512, 7);
    uint32_t t0 = s_host.t_us;
    CHECK_ERR(sim_write_blocks(&s_host, 700, 1, w), SIM_OK);
    uint32_t dt_w = s_host.t_us - t0;
    CHECK(dt_w >= 50000, "write took only %u us", dt_w);
    t0 = s_host.t_us;
    CHECK_ERR(sim_read_blocks(&s_host, 700, 1, r), SIM_OK);
    uint32_t dt_r = s_host.t_us - t0;
    CHECK(dt_r >= 30000, "read took only %u us", dt_r);
    CHECK(memcmp(w, r, 512) == 0, "data differs");
    printf("  write %u ms, read %u ms\n", dt_w / 1000, dt_r / 1000);
}

static void t8_flip(void)
{
    static uint8_t w[5 * 512], r[5 * 512];
    if (!init_ok()) return;
    fill(w, sizeof(w), 8);
    CHECK_ERR(sim_write_blocks(&s_host, 800, 5, w), SIM_OK);
    fault("action=flip_byte cmd=18 nth=2 every=2 max=2 offset=10 mask=0x01");
    CHECK_ERR(sim_read_blocks(&s_host, 800, 5, r), SIM_OK);   /* CRC is valid */
    for (int b = 0; b < 5; b++) {
        bool flipped = (b == 1 || b == 3);   /* blocks 2 and 4 */
        for (int i = 0; i < 512; i++) {
            uint8_t want = w[b * 512 + i] ^ ((flipped && i == 10) ? 0x01 : 0x00);
            if (r[b * 512 + i] != want) {
                CHECK(0, "block %d byte %d: %02x want %02x", b + 1, i, r[b * 512 + i], want);
                break;
            }
        }
    }
}

static void t9_exact_counting(void)
{
    static uint8_t w[4 * 512], r[2 * 512];
    if (!init_ok()) return;
    fill(w, sizeof(w), 9);
    CHECK_ERR(sim_write_blocks(&s_host, 900, 4, w), SIM_OK);
    /* the 3rd block delivered by CMD18 gets flipped. After the 2nd block of
     * the first read the card already prepares a 3rd one, which CMD12
     * cancels: it must not be counted. */
    fault("action=flip_byte cmd=18 nth=3 offset=0 mask=0x80");
    CHECK_ERR(sim_read_blocks(&s_host, 900, 2, r), SIM_OK);
    CHECK(memcmp(r, w, 1024) == 0, "first CMD18 modified");
    CHECK_ERR(sim_read_blocks(&s_host, 902, 2, r), SIM_OK);
    CHECK(r[0] == (w[2 * 512] ^ 0x80), "3rd delivered block not flipped");
    CHECK(memcmp(r + 1, w + 2 * 512 + 1, 511) == 0 && memcmp(r + 512, w + 3 * 512, 512) == 0,
          "other bytes changed");
    sdemu_fault_table_t t;
    sdemu_faults_get(&s_card, &t);
    CHECK(t.rules[0].matches == 4 && t.rules[0].fires == 1,
          "matches=%u fires=%u", t.rules[0].matches, t.rules[0].fires);
}

static void t10_error_token_r1(void)
{
    uint8_t r[512], r2[2];
    if (!init_ok()) return;
    fault("action=error_token cmd=17 lba=500 token=0x01");
    CHECK_ERR(sim_read_blocks(&s_host, 500, 1, r), SIM_ERR_INVALID_RESPONSE);
    CHECK_ERR(sim_read_blocks(&s_host, 500, 1, r), SIM_OK);   /* only once */

    fault("action=r1_error cmd=13 bits=0x04");
    CHECK_ERR(sim_send_status(&s_host, r2), SIM_ERR_NOT_SUPPORTED);
    CHECK(s_host.last_r1 == 0x04, "R1 %02x", s_host.last_r1);
}

static void t11_bad_cmd_crc(void)
{
    uint8_t r[2];
    if (!init_ok()) return;
    sim_cs(&s_host, true);
    int err = sim_cmd(&s_host, 13, 0, 2, r, true);
    sim_cs(&s_host, false);
    CHECK_ERR(err, SIM_ERR_INVALID_CRC);
    CHECK(s_card.stats.cmd_crc_errors == 1, "crc errors %u", s_card.stats.cmd_crc_errors);
    CHECK_ERR(sim_send_status(&s_host, NULL), SIM_OK);

    /* bad data CRC in the last byte only: the card has already started
     * sending 0x05 as its guess and must correct it to 0x0B in flight */
    uint8_t w[512];
    fill(w, 512, 11);
    s_host.write_crc_xor = 0x0001;
    CHECK_ERR(sim_write_blocks(&s_host, 1100, 1, w), SIM_ERR_INVALID_CRC);
    CHECK(s_host.corrections == 1, "corrections %u", s_host.corrections);
    CHECK(s_host.pipeline_violations == 0, "bit 7 changed");
    CHECK(memcmp(s_disk + 1100 * 512, w, 512) != 0, "block with bad CRC was stored");
    s_host.write_crc_xor = 0x0100;   /* first CRC byte wrong: guessed right, no correction */
    CHECK_ERR(sim_write_blocks(&s_host, 1100, 1, w), SIM_ERR_INVALID_CRC);
    CHECK(s_host.corrections == 1, "unexpected correction");
    CHECK_ERR(sim_write_blocks(&s_host, 1100, 1, w), SIM_OK);
}

static void t12_power_cycle(void)
{
    uint8_t w[512], r[512];
    if (!init_ok()) return;
    fill(w, 512, 12);
    CHECK_ERR(sim_write_blocks(&s_host, 1200, 1, w), SIM_OK);
    sdemu_power_cycle(&s_card);
    CHECK_ERR(sim_send_status(&s_host, NULL), SIM_ERR_TIMEOUT);   /* silent until CMD0 */
    if (!init_ok()) return;
    CHECK_ERR(sim_read_blocks(&s_host, 1200, 1, r), SIM_OK);
    CHECK(memcmp(w, r, 512) == 0, "data lost");
}

/* host behaviour documented in DESIGN.md: CMD8 ignored -> v1 path -> timeout */
static void t13_cmd8_ignored(void)
{
    sim_card_info_t info;
    fault("action=no_response cmd=8 every=1");
    int err = sim_card_init(&s_host, &info);
    CHECK_ERR(err, SIM_ERR_TIMEOUT);
    CHECK(s_host.t_us > 2900000, "gave up after %u ms", s_host.t_us / 1000);
    printf("  host gave up after %u ms\n", s_host.t_us / 1000);
}

static const struct {
    const char *name;
    void (*fn)(void);
} s_tests[] = {
    { "CRC sanity", t0_crc },
    { "initialization", t1_init },
    { "single block", t2_single },
    { "multi block", t3_multi },
    { "write error", t4_write_error },
    { "read CRC", t5_read_crc },
    { "no / delayed response", t6_response_timing },
    { "busy and read delay", t7_busy_and_delay },
    { "flipped byte", t8_flip },
    { "exact fault counting", t9_exact_counting },
    { "error token, R1 error", t10_error_token_r1 },
    { "bad command CRC", t11_bad_cmd_crc },
    { "power cycle", t12_power_cycle },
    { "CMD8 ignored (host behaviour)", t13_cmd8_ignored },
};

int main(int argc, char **argv)
{
    int only = -1;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-v") == 0) {
            s_verbose = true;
        } else {
            only = atoi(argv[i]);
        }
    }
    s_disk = malloc((size_t)CAPACITY * SDEMU_BLOCK_SIZE);
    int groups = 0, failed_groups = 0;
    for (int i = 0; i < (int)(sizeof(s_tests) / sizeof(s_tests[0])); i++) {
        if (only >= 0 && i != only) {
            continue;
        }
        int before = s_fail;
        printf("[%2d] %s\n", i, s_tests[i].name);
        setup();
        s_tests[i].fn();
        groups++;
        if (s_fail != before) {
            failed_groups++;
            printf("  -> FAILED\n");
        }
    }
    printf("\n%d/%d test groups passed (%d checks, %d failed)\n",
           groups - failed_groups, groups, s_checks, s_fail);
    free(s_disk);
    return s_fail ? 1 : 0;
}
