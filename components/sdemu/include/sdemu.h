/*
 * sdemu - SD card (SPI mode) emulator engine.
 *
 * Everything a user of the engine sees is in this header: card config,
 * storage backend interface, fault rules, trace events and the engine calls.
 *
 * The engine is plain C with no ESP-IDF dependency. It only sees bytes:
 *
 *   sdemu_select()      CS went low
 *   sdemu_tx()          "give me the byte after the one now on the wire"
 *   sdemu_rx(byte)      "this byte just arrived on MOSI"
 *   sdemu_deselect()    CS went high
 *
 * A transport (bit-banged on the ESP32-S3, or the laptop simulator) must
 * drive it with this exact pipeline:
 *
 *   on CS low:            select(); cur = tx();
 *   for each byte i:      shift `cur` out on MISO while receiving `in`;
 *     middle of byte i:   next = tx();          (output for byte i+1)
 *     end of byte i:      cur = next;
 *                         fix = rx(in);         (fix >= 0: replace cur,
 *                         if (fix >= 0)          bit 7 is guaranteed equal)
 *                             cur = fix;
 *   on CS high:           deselect();
 *
 * The engine's reaction to byte i therefore appears at byte i+2 at the
 * earliest. See docs/DESIGN.md section 3 for why.
 *
 * Threading: all engine calls except the sdemu_faults_* writer functions and
 * sdemu_get_stats() must come from one thread (core 1 on the S3).
 */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SDEMU_BLOCK_SIZE 512u

/* ------------------------------------------------------------------------ */
/* Storage backend                                                           */
/* ------------------------------------------------------------------------ */

typedef enum {
    SDEMU_IO_OK = 0,   /* done */
    SDEMU_IO_BUSY,     /* not ready yet, ask again later (engine keeps the host waiting) */
    SDEMU_IO_ERROR,    /* failed: read -> data error token, write -> write error */
} sdemu_io_t;

/*
 * All calls must return quickly (bounded work): the engine calls them from
 * the real-time loop with at most 8 bytes at a time. A backend that needs
 * slow work (PSRAM, flash, a PC) answers SDEMU_IO_BUSY and does the work
 * elsewhere.
 */
typedef struct {
    void *ctx;
    /* Copy `len` bytes at offset `off` of block `lba` into dst. */
    sdemu_io_t (*read)(void *ctx, uint32_t lba, uint32_t off, uint8_t *dst, uint32_t len);
    /* Store `len` bytes at offset `off` of the block being written. */
    sdemu_io_t (*write)(void *ctx, uint32_t lba, uint32_t off, const uint8_t *src, uint32_t len);
    /* Commit the block written with write(). Called repeatedly until not BUSY. */
    sdemu_io_t (*commit)(void *ctx, uint32_t lba);
} sdemu_storage_t;

/* Plain RAM disk: reads and writes go straight to `mem` (capacity_blocks * 512). */
typedef struct {
    uint8_t *mem;
    uint32_t blocks;
    uint8_t wbuf[SDEMU_BLOCK_SIZE];
} sdemu_ram_backend_t;
void sdemu_ram_backend_init(sdemu_ram_backend_t *be, sdemu_storage_t *out, uint8_t *mem, uint32_t blocks);

/* Pattern disk: reads return a deterministic pattern derived from the LBA,
 * writes are accepted and discarded. Useful for huge advertised capacities. */
void sdemu_pattern_backend_init(sdemu_storage_t *out);
uint8_t sdemu_pattern_byte(uint32_t lba, uint32_t off);

/* ------------------------------------------------------------------------ */
/* Fault injection                                                           */
/* ------------------------------------------------------------------------ */

typedef enum {
    SDEMU_FA_NONE = 0,
    /* command stage: acts on the command's R1 */
    SDEMU_FA_NO_RESPONSE,      /* card stays silent, command not executed */
    SDEMU_FA_DELAY_RESPONSE,   /* param = extra 0xFF bytes before R1 */
    SDEMU_FA_R1_ERROR,         /* param = R1 bits to set; command not executed */
    /* read block stage: acts on one data block sent to the host */
    SDEMU_FA_READ_DELAY,       /* param = ms before the 0xFE token */
    SDEMU_FA_ERROR_TOKEN,      /* param = data error token (default 0x01) instead of 0xFE */
    SDEMU_FA_CORRUPT_CRC16,    /* wrong CRC16 */
    SDEMU_FA_FLIP_BYTE,        /* param = offset, param2 = xor mask (default 0xFF); CRC stays valid */
    /* write block stage: acts on one data block received from the host */
    SDEMU_FA_WRITE_ERROR,      /* data response 0x0D, block not stored */
    SDEMU_FA_WRITE_CRC_ERROR,  /* data response 0x0B, block not stored */
    SDEMU_FA_BUSY,             /* param = extra busy time in ms */
    SDEMU_FA_DROP_WRITE,       /* data response 0x05, block silently not stored */
    SDEMU_FA__COUNT
} sdemu_fault_action_t;

typedef enum {
    SDEMU_STAGE_CMD = 0,
    SDEMU_STAGE_READ,
    SDEMU_STAGE_WRITE,
} sdemu_fault_stage_t;

#define SDEMU_CMD_ANY   0xFFu
#define SDEMU_ACMD(n)   (100u + (n))   /* ACMDs are matched as 100+n */
#define SDEMU_MAX_RULES 8

/*
 * A rule matches when its stage sees command `cmd` (any if SDEMU_CMD_ANY)
 * and an LBA in [lba_first, lba_last]. For command-stage rules the LBA is the
 * command argument of CMD17/18/24/25 and ignored otherwise.
 *
 * Occurrence: every match is counted. The rule fires on match number `nth`
 * (1 = the first), and then every `every` matches after it (0 = only once).
 * `max` caps the total number of fires (0 = no cap).
 */
typedef struct {
    uint8_t  action;      /* sdemu_fault_action_t */
    uint8_t  cmd;         /* 0..63, SDEMU_ACMD(n), or SDEMU_CMD_ANY */
    uint32_t lba_first;
    uint32_t lba_last;
    uint32_t nth;
    uint32_t every;
    uint32_t max;
    uint32_t param;
    uint32_t param2;
    /* runtime, written by the engine */
    uint32_t matches;
    uint32_t fires;
} sdemu_fault_rule_t;

typedef struct {
    uint32_t count;
    sdemu_fault_rule_t rules[SDEMU_MAX_RULES];
} sdemu_fault_table_t;

void sdemu_fault_rule_default(sdemu_fault_rule_t *r);
sdemu_fault_stage_t sdemu_fault_stage(uint8_t action);
const char *sdemu_fault_action_name(uint8_t action);
/* Parse "action=write_error cmd=24 nth=2 lba=100-200 param=5 mask=0x01 ..." */
bool sdemu_fault_parse(const char *text, sdemu_fault_rule_t *out, char *err, size_t err_len);
/* Format a rule as the same key=value text. */
int sdemu_fault_format(const sdemu_fault_rule_t *r, char *buf, size_t len);

/* ------------------------------------------------------------------------ */
/* Trace                                                                     */
/* ------------------------------------------------------------------------ */

typedef enum {
    SDEMU_EV_CMD = 1,      /* cmd, arg, r1 (0xFF = no response) */
    SDEMU_EV_CMD_CRC,      /* command rejected: bad CRC7; cmd, arg */
    SDEMU_EV_READ_BLOCK,   /* arg = lba, extra = token sent */
    SDEMU_EV_WRITE_BLOCK,  /* arg = lba, extra = data response token */
    SDEMU_EV_WRITE_STOP,   /* stop token of CMD25 */
    SDEMU_EV_FAULT,        /* cmd, arg = lba/arg, extra = action */
    SDEMU_EV_POWER,        /* power cycle */
} sdemu_event_type_t;

typedef struct {
    uint32_t t_us;
    uint32_t arg;
    uint8_t  type;
    uint8_t  cmd;      /* ACMD as 100+n */
    uint8_t  r1;
    uint8_t  extra;
} sdemu_event_t;

typedef void (*sdemu_trace_fn)(void *ctx, const sdemu_event_t *ev);
typedef uint32_t (*sdemu_clock_fn)(void *ctx);   /* microseconds, may wrap */

/* ------------------------------------------------------------------------ */
/* Card config                                                               */
/* ------------------------------------------------------------------------ */

typedef struct {
    uint32_t capacity_blocks;      /* multiple of 1024 (CSD v2 granularity) */
    uint8_t  manufacturer_id;
    char     oem_id[3];            /* 2 chars */
    char     product_name[6];      /* 5 chars */
    uint8_t  product_rev;
    uint32_t serial;
    uint16_t mfg_year;             /* 2000.. */
    uint8_t  mfg_month;            /* 1..12 */
    uint8_t  acmd41_polls;         /* ACMD41 answers "idle" this many times first (>= 1) */

    const sdemu_storage_t *storage;
    sdemu_clock_fn now_us;
    void *clock_ctx;
    sdemu_trace_fn trace;          /* may be NULL */
    void *trace_ctx;
} sdemu_config_t;

void sdemu_config_default(sdemu_config_t *cfg);

/* ------------------------------------------------------------------------ */
/* Engine                                                                    */
/* ------------------------------------------------------------------------ */

typedef struct {
    uint32_t commands;
    uint32_t cmd_crc_errors;
    uint32_t blocks_read;
    uint32_t blocks_written;
    uint32_t write_rejects;
    uint32_t faults_fired;
    uint32_t storage_busy;   /* tx() calls that waited for storage */
} sdemu_stats_t;

/* Engine state. Fields are private; it is declared here so it can be
 * allocated statically (in internal RAM on the S3). */
typedef struct sdemu {
    sdemu_config_t cfg;

    /* registers */
    uint8_t cid[16];
    uint8_t csd[16];
    uint8_t scr[8];
    uint8_t ssr[64];

    /* card state */
    uint8_t spi_mode;     /* CMD0 seen since power-up */
    uint8_t idle;         /* ACMD41 not completed yet */
    uint8_t crc_on;       /* CMD59 */
    uint8_t app_cmd;      /* previous command was CMD55 */
    uint8_t acmd41_count;
    uint8_t status_err;   /* latched for the next CMD13 */

    /* command assembler and split decode (see DESIGN.md 3.3) */
    uint8_t cmd_buf[6];
    uint8_t cmd_len;
    uint8_t cmd_stage;    /* 0 none, 1 frame complete, 2 decoded */
    uint8_t cmd_code;     /* 0..63 or 100+n */
    uint8_t cmd_crc_ok;
    uint32_t cmd_arg;

    /* output */
    uint8_t out_state;
    uint8_t resp[6];
    uint8_t resp_len;
    uint8_t resp_pos;
    uint16_t resp_delay;
    uint8_t after_resp;
    uint8_t busy_bytes;

    /* data phase */
    uint8_t  multi;
    uint8_t  blk[SDEMU_BLOCK_SIZE];
    uint16_t blk_len;
    int16_t  blk_pos;     /* RD_SEND: -1 = token */
    uint16_t crc16;
    uint16_t crc16_rx;
    const uint8_t *reg_src;   /* register read, NULL = storage block */
    uint32_t lba;
    uint32_t deadline_us;
    uint8_t  waiting;     /* deadline active */
    uint8_t  nac;         /* 0xFF bytes still owed before a read token */
    uint8_t  rd_fault;    /* previewed read-stage fault action */
    int8_t   rd_rule;     /* index of that rule, -1 none */
    uint32_t rd_param, rd_param2;
    uint8_t  wr_token;    /* data response decided at block start (0 = decide at end) */
    uint8_t  wr_guess;    /* token sent as a guess, corrected in rx() if needed */
    uint8_t  wr_drop;
    uint32_t wr_busy_ms;
    uint16_t commit_pos;
    uint8_t  committed;

    /* faults: double buffered, see DESIGN.md 3.7 */
    sdemu_fault_table_t ftab[2];
    volatile uint32_t factive;   /* index of the table the engine uses */
    volatile uint32_t fpending;  /* 0 idle, 1 published, 2 engine swapping */
    sdemu_fault_table_t fmaster; /* writer side master copy */

    sdemu_stats_t stats;
} sdemu_t;

void sdemu_init(sdemu_t *card, const sdemu_config_t *cfg);
void sdemu_power_cycle(sdemu_t *card);   /* data survives, protocol state does not */

void sdemu_select(sdemu_t *card);
void sdemu_deselect(sdemu_t *card);
uint8_t sdemu_tx(sdemu_t *card);
int sdemu_rx(sdemu_t *card, uint8_t byte);

/* Writer side (any thread, one writer at a time). The engine picks the new
 * table up at the next command. Adding or clearing resets all counters. */
bool sdemu_faults_add(sdemu_t *card, const sdemu_fault_rule_t *rule);
void sdemu_faults_clear(sdemu_t *card);
/* Copy of the table the engine currently uses (counters included). */
void sdemu_faults_get(const sdemu_t *card, sdemu_fault_table_t *out);

void sdemu_get_stats(const sdemu_t *card, sdemu_stats_t *out);
const char *sdemu_cmd_name(uint8_t cmd);

#ifdef __cplusplus
}
#endif
