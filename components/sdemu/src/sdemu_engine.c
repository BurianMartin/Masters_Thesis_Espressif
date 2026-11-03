/*
 * sdemu protocol engine: an SDHC card in SPI mode, one byte at a time.
 *
 * Read docs/DESIGN.md first. The short version:
 *
 *  - The transport asks for the next output byte (tx) in the middle of the
 *    current byte and delivers the received byte (rx) at its end, so a
 *    reaction to byte i appears at byte i+2 at the earliest.
 *  - Command handling is split over three calls so no single call is long:
 *      rx(last byte)  only marks the frame complete
 *      tx()           decodes it and checks CRC7      -> sends 0xFF
 *      tx()           executes it                     -> sends R1
 *    R1 therefore lands on the 3rd byte after the command (N_CR = 3), inside
 *    both the spec's 1..8 and ESP-IDF's 2nd..9th byte window.
 *  - Every call does bounded work: blocks are moved and CRC'd 8 bytes per
 *    tx(), and storage that is not ready answers BUSY, which keeps the host
 *    waiting with 0xFF (reads) or busy 0x00 (writes) like a slow card.
 *  - The write data response must be the byte right after the last CRC byte,
 *    which is one byte too early for a reaction. tx() sends a guess and rx()
 *    of the last CRC byte corrects it. All data response tokens are 000xxxx1,
 *    so bit 7, which has already left the chip, never changes.
 */
#include <string.h>
#include "sdemu_internal.h"

enum {
    OS_IDLE = 0,    /* nothing to say: 0xFF */
    OS_RESP,        /* sending resp[] (after resp_delay 0xFF bytes) */
    OS_RD_PREP,     /* preparing a block (copy + CRC in 8-byte steps): 0xFF */
    OS_RD_SEND,     /* token, data, CRC16 */
    OS_RD_ERRTOK,   /* one data error token instead of a block */
    OS_BUSY,        /* busy_bytes x 0x00, then busy_next */
    OS_WR_TOKEN,    /* waiting for a start/stop token on MOSI: 0xFF */
    OS_WR_DATA,     /* receiving a block: 0xFF, then the data response */
    OS_WR_COMMIT,   /* storing the block: busy 0x00 */
};

enum { AFTER_NONE = 0, AFTER_READ, AFTER_WRITE, AFTER_BUSY };

static uint8_t s_crc7_table[256];

static void crc7_table_init(void)
{
    for (int i = 0; i < 256; i++) {
        uint8_t crc = (uint8_t)i;
        for (int b = 0; b < 8; b++) {
            crc = (uint8_t)((crc & 0x80) ? ((crc << 1) ^ 0x12) : (crc << 1));
        }
        s_crc7_table[i] = crc;
    }
}

static inline uint8_t frame_crc_byte(const uint8_t *f)
{
    uint8_t crc = 0;   /* CRC7 kept in bits 7..1 */
    for (int i = 0; i < 5; i++) {
        crc = s_crc7_table[crc ^ f[i]];
    }
    return (uint8_t)(crc | 1);
}

static inline uint32_t now_us(sdemu_t *c)
{
    return c->cfg.now_us ? c->cfg.now_us(c->cfg.clock_ctx) : 0;
}

static inline bool time_reached(sdemu_t *c, uint32_t deadline)
{
    return (int32_t)(now_us(c) - deadline) >= 0;
}

static inline sdemu_fault_table_t *ftab(sdemu_t *c)
{
    return &c->ftab[c->factive];
}

static void trace(sdemu_t *c, uint8_t type, uint8_t cmd, uint32_t arg, uint8_t r1, uint8_t extra)
{
    if (!c->cfg.trace) {
        return;
    }
    sdemu_event_t ev = {
        .t_us = now_us(c), .arg = arg, .type = type, .cmd = cmd, .r1 = r1, .extra = extra,
    };
    c->cfg.trace(c->cfg.trace_ctx, &ev);
}

static inline uint8_t r1_base(const sdemu_t *c)
{
    return c->idle ? R1_IDLE : 0;
}

/* ------------------------------------------------------------------------ */
/* init                                                                      */
/* ------------------------------------------------------------------------ */

void sdemu_config_default(sdemu_config_t *cfg)
{
    memset(cfg, 0, sizeof(*cfg));
    cfg->capacity_blocks = 12288;    /* 6 MiB */
    cfg->manufacturer_id = 0x7E;
    memcpy(cfg->oem_id, "SE", 3);
    memcpy(cfg->product_name, "SDEMU", 6);
    cfg->product_rev = 0x10;
    cfg->serial = 0x00C0FFEE;
    cfg->mfg_year = 2026;
    cfg->mfg_month = 10;
    cfg->acmd41_polls = 2;
}

static void reset_protocol(sdemu_t *c)
{
    c->spi_mode = 0;
    c->idle = 1;
    c->crc_on = 0;
    c->app_cmd = 0;
    c->acmd41_count = 0;
    c->status_err = 0;
    c->cmd_len = 0;
    c->cmd_stage = 0;
    c->out_state = OS_IDLE;
    c->resp_delay = 0;
    c->waiting = 0;
}

void sdemu_init(sdemu_t *c, const sdemu_config_t *cfg)
{
    memset(c, 0, sizeof(*c));
    crc7_table_init();
    c->cfg = *cfg;
    if (c->cfg.capacity_blocks < 1024) {
        c->cfg.capacity_blocks = 1024;
    }
    c->cfg.capacity_blocks &= ~1023u;   /* CSD v2 granularity: 512 KiB */
    if (c->cfg.acmd41_polls == 0) {
        c->cfg.acmd41_polls = 1;
    }
    sdemu_build_registers(c);
    reset_protocol(c);
}

void sdemu_power_cycle(sdemu_t *c)
{
    reset_protocol(c);
    trace(c, SDEMU_EV_POWER, 0, 0, 0xFF, 0);
}

/* ------------------------------------------------------------------------ */
/* CS                                                                        */
/* ------------------------------------------------------------------------ */

void sdemu_select(sdemu_t *c)
{
    c->cmd_len = 0;
    c->cmd_stage = 0;
}

void sdemu_deselect(sdemu_t *c)
{
    c->cmd_len = 0;
    c->cmd_stage = 0;
    /* A card keeps storing a written block after CS goes high; everything
     * else is a transfer the host abandoned. */
    if (c->out_state != OS_WR_COMMIT) {
        c->out_state = OS_IDLE;
        c->resp_delay = 0;
    }
}

/* ------------------------------------------------------------------------ */
/* responses and data phases                                                 */
/* ------------------------------------------------------------------------ */

static void respond(sdemu_t *c, uint8_t after, uint8_t len, uint8_t b0, uint8_t b1,
                    uint8_t b2, uint8_t b3, uint8_t b4)
{
    c->resp[0] = b0; c->resp[1] = b1; c->resp[2] = b2; c->resp[3] = b3; c->resp[4] = b4;
    c->resp_len = len;
    c->resp_pos = 0;
    c->after_resp = after;
    c->out_state = OS_RESP;
}

static inline void respond_r1(sdemu_t *c, uint8_t r1)
{
    respond(c, AFTER_NONE, 1, r1, 0, 0, 0, 0);
}

static void read_start(sdemu_t *c)
{
    c->out_state = OS_RD_PREP;
    c->blk_pos = 0;
    c->crc16 = 0;
    c->waiting = 0;
    c->rd_fault = SDEMU_FA_NONE;
    c->rd_rule = -1;
    c->blk_len = c->reg_src ? c->blk_len : SDEMU_BLOCK_SIZE;
    c->committed = 0;   /* reused as "preview done" flag during RD_PREP */
    /* N_AC: never put the token inside the host's command transaction,
     * which continues for up to 14 bytes after R1 and is not scanned. */
    c->nac = 16;
}

static void read_error_token(sdemu_t *c, uint8_t token)
{
    c->rd_param = token;
    c->out_state = OS_RD_ERRTOK;
}

static uint8_t tx_read_prep(sdemu_t *c)
{
    if (c->nac) {
        c->nac--;
    }
    if (!c->reg_src && !c->committed) {
        /* first step of a storage block: range check and fault preview */
        c->committed = 1;
        if (c->lba >= c->cfg.capacity_blocks) {
            read_error_token(c, DE_OUT_OF_RANGE);
            return 0xFF;
        }
        int fi = sdemu_fault_preview(ftab(c), SDEMU_STAGE_READ, c->cmd_code, c->lba);
        c->rd_rule = (int8_t)fi;
        if (fi >= 0) {
            const sdemu_fault_rule_t *r = &ftab(c)->rules[fi];
            c->rd_fault = r->action;
            c->rd_param = r->param;
            c->rd_param2 = r->param2;
            if (r->action == SDEMU_FA_ERROR_TOKEN) {
                c->out_state = OS_RD_ERRTOK;
                return 0xFF;
            }
            if (r->action == SDEMU_FA_READ_DELAY) {
                c->deadline_us = now_us(c) + r->param * 1000u;
                c->waiting = 1;
            }
        }
        return 0xFF;
    }
    if (c->waiting) {
        if (!time_reached(c, c->deadline_us)) {
            return 0xFF;
        }
        c->waiting = 0;
    }

    uint16_t pos = (uint16_t)c->blk_pos;
    uint8_t *dst = &c->blk[pos];
    if (c->reg_src) {
        memcpy(dst, c->reg_src + pos, 8);
    } else {
        sdemu_io_t io = c->cfg.storage->read(c->cfg.storage->ctx, c->lba, pos, dst, 8);
        if (io == SDEMU_IO_BUSY) {
            c->stats.storage_busy++;
            return 0xFF;
        }
        if (io == SDEMU_IO_ERROR) {
            read_error_token(c, DE_ERROR);
            return 0xFF;
        }
        if (c->rd_fault == SDEMU_FA_FLIP_BYTE && c->rd_param >= pos && c->rd_param < pos + 8u) {
            c->blk[c->rd_param] ^= (uint8_t)c->rd_param2;   /* before the CRC: CRC stays valid */
        }
    }
    uint16_t crc = c->crc16;
    for (int i = 0; i < 8; i++) {
        crc = sdemu_crc16_byte(crc, dst[i]);
    }
    c->crc16 = crc;
    pos += 8;
    c->blk_pos = (int16_t)pos;
    if (pos >= c->blk_len) {
        if (c->rd_fault == SDEMU_FA_CORRUPT_CRC16) {
            c->crc16 ^= 0xFFFF;
        }
        c->out_state = OS_RD_SEND;
        c->blk_pos = -1;
    }
    return 0xFF;
}

static uint8_t tx_read_send(sdemu_t *c)
{
    int16_t pos = c->blk_pos;
    if (pos < 0) {
        if (c->nac) {
            c->nac--;
            return 0xFF;
        }
        /* the token really goes out now: count the block for fault rules */
        if (!c->reg_src) {
            sdemu_fault_commit(ftab(c), SDEMU_STAGE_READ, c->cmd_code, c->lba, c->rd_rule);
            if (c->rd_rule >= 0) {
                c->stats.faults_fired++;
                trace(c, SDEMU_EV_FAULT, c->cmd_code, c->lba, 0xFF, c->rd_fault);
            }
            trace(c, SDEMU_EV_READ_BLOCK, c->cmd_code, c->lba, 0xFF, TOKEN_START);
        }
        c->blk_pos = 0;
        return TOKEN_START;
    }
    if (pos < (int16_t)c->blk_len) {
        c->blk_pos = (int16_t)(pos + 1);
        return c->blk[pos];
    }
    if (pos == (int16_t)c->blk_len) {
        c->blk_pos = (int16_t)(pos + 1);
        return (uint8_t)(c->crc16 >> 8);
    }
    uint8_t last = (uint8_t)c->crc16;
    if (c->reg_src) {
        c->reg_src = NULL;
        c->out_state = OS_IDLE;
    } else {
        c->stats.blocks_read++;
        if (c->multi) {
            c->lba++;
            read_start(c);   /* prepare the next block; CMD12 may still cancel it */
        } else {
            c->out_state = OS_IDLE;
        }
    }
    return last;
}

static uint8_t tx_read_errtok(sdemu_t *c)
{
    if (c->nac) {
        c->nac--;
        return 0xFF;
    }
    if (c->rd_rule >= 0) {
        sdemu_fault_commit(ftab(c), SDEMU_STAGE_READ, c->cmd_code, c->lba, c->rd_rule);
        c->stats.faults_fired++;
        trace(c, SDEMU_EV_FAULT, c->cmd_code, c->lba, 0xFF, c->rd_fault);
    }
    trace(c, SDEMU_EV_READ_BLOCK, c->cmd_code, c->lba, 0xFF, (uint8_t)c->rd_param);
    c->out_state = OS_IDLE;
    return (uint8_t)c->rd_param;
}

static void busy_then(sdemu_t *c, uint8_t bytes, uint8_t next)
{
    c->busy_bytes = bytes;
    c->after_resp = next;   /* reused: state after OS_BUSY */
    c->out_state = OS_BUSY;
}

static uint8_t tx_write_commit(sdemu_t *c)
{
    const sdemu_storage_t *st = c->cfg.storage;
    if (!c->wr_drop && c->commit_pos < SDEMU_BLOCK_SIZE) {
        sdemu_io_t io = st->write(st->ctx, c->lba, c->commit_pos, &c->blk[c->commit_pos], 8);
        if (io == SDEMU_IO_BUSY) {
            c->stats.storage_busy++;
            return 0x00;
        }
        if (io == SDEMU_IO_ERROR) {
            c->status_err = 1;
            c->wr_drop = 1;
        }
        c->commit_pos += 8;
        return 0x00;
    }
    if (!c->committed) {
        if (!c->wr_drop) {
            sdemu_io_t io = st->commit(st->ctx, c->lba);
            if (io == SDEMU_IO_BUSY) {
                c->stats.storage_busy++;
                return 0x00;
            }
            if (io == SDEMU_IO_ERROR) {
                c->status_err = 1;   /* reported through the next CMD13 */
            }
        }
        c->committed = 1;
        if (c->wr_busy_ms) {
            c->deadline_us = now_us(c) + c->wr_busy_ms * 1000u;
            c->waiting = 1;
        }
        return 0x00;
    }
    if (c->waiting) {
        if (!time_reached(c, c->deadline_us)) {
            return 0x00;
        }
        c->waiting = 0;
    }
    c->stats.blocks_written++;
    if (c->multi) {
        c->lba++;
        c->out_state = OS_WR_TOKEN;
    } else {
        c->out_state = OS_IDLE;
    }
    return 0xFF;
}

static void write_block_begin(sdemu_t *c)
{
    c->out_state = OS_WR_DATA;
    c->blk_pos = 0;
    c->crc16 = 0;
    c->wr_token = 0;
    c->wr_guess = DR_ACCEPTED;
    c->wr_drop = 0;
    c->wr_busy_ms = 0;
    if (c->lba >= c->cfg.capacity_blocks) {
        c->wr_token = DR_WRITE_ERROR;
        return;
    }
    int fi = sdemu_fault_preview(ftab(c), SDEMU_STAGE_WRITE, c->cmd_code, c->lba);
    sdemu_fault_commit(ftab(c), SDEMU_STAGE_WRITE, c->cmd_code, c->lba, fi);
    if (fi < 0) {
        return;
    }
    const sdemu_fault_rule_t *r = &ftab(c)->rules[fi];
    switch (r->action) {
    case SDEMU_FA_WRITE_ERROR:     c->wr_token = DR_WRITE_ERROR; break;
    case SDEMU_FA_WRITE_CRC_ERROR: c->wr_token = DR_CRC_ERROR; break;
    case SDEMU_FA_BUSY:            c->wr_busy_ms = r->param; break;
    case SDEMU_FA_DROP_WRITE:      c->wr_drop = 1; break;
    default: break;
    }
    c->stats.faults_fired++;
    trace(c, SDEMU_EV_FAULT, c->cmd_code, c->lba, 0xFF, r->action);
}

/* last CRC byte of a written block arrived: final data response */
static int write_block_end(sdemu_t *c)
{
    uint8_t tok = c->wr_token;
    if (!tok) {
        tok = (c->crc_on && c->crc16_rx != c->crc16) ? DR_CRC_ERROR : DR_ACCEPTED;
    }
    trace(c, SDEMU_EV_WRITE_BLOCK, c->cmd_code, c->lba, 0xFF, tok);
    if (tok == DR_ACCEPTED) {
        c->out_state = OS_WR_COMMIT;
        c->commit_pos = 0;
        c->committed = 0;
        c->waiting = 0;
    } else {
        c->stats.write_rejects++;
        busy_then(c, 1, c->multi ? AFTER_WRITE : AFTER_NONE);
    }
    return tok != c->wr_guess ? tok : -1;
}

/* ------------------------------------------------------------------------ */
/* commands                                                                  */
/* ------------------------------------------------------------------------ */

static bool is_known_acmd(uint8_t idx)
{
    return idx == 13 || idx == 23 || idx == 41 || idx == 51;
}

static void swap_fault_table(sdemu_t *c)
{
    uint32_t one = 1;
    if (c->fpending == 1 &&
        __atomic_compare_exchange_n(&c->fpending, &one, 2, false, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)) {
        c->factive ^= 1;
        __atomic_store_n(&c->fpending, 0, __ATOMIC_RELEASE);
    }
}

/* stage 1: decode and check CRC (one tx() call) */
static void cmd_decode(sdemu_t *c)
{
    const uint8_t *f = c->cmd_buf;
    uint8_t idx = f[0] & 0x3F;
    c->cmd_arg = ((uint32_t)f[1] << 24) | ((uint32_t)f[2] << 16) | ((uint32_t)f[3] << 8) | f[4];
    c->cmd_crc_ok = frame_crc_byte(f) == f[5];
    c->cmd_code = (c->app_cmd && is_known_acmd(idx)) ? (uint8_t)SDEMU_ACMD(idx) : idx;
    c->cmd_stage = 2;

    /* a command stops any data the card is sending (CMD12 during CMD18) */
    if (c->out_state == OS_RD_PREP || c->out_state == OS_RD_SEND || c->out_state == OS_RD_ERRTOK ||
        c->out_state == OS_RESP) {
        c->out_state = OS_IDLE;
        c->reg_src = NULL;
    }
    swap_fault_table(c);
}

static void start_read(sdemu_t *c, const uint8_t *reg, uint16_t len, uint8_t r1, uint8_t r2, bool has_r2)
{
    c->reg_src = reg;
    c->blk_len = len;
    c->multi = 0;
    respond(c, AFTER_READ, has_r2 ? 2 : 1, r1, r2, 0, 0, 0);
}

/* stage 2: execute (one tx() call) */
static void cmd_execute(sdemu_t *c)
{
    uint8_t code = c->cmd_code;
    uint8_t idx = code >= 100 ? (uint8_t)(code - 100) : code;
    uint32_t arg = c->cmd_arg;
    c->cmd_stage = 0;
    c->resp_delay = 0;

    if (!c->spi_mode) {
        /* still in SD mode: only a valid CMD0 (with CS low) switches to SPI */
        if (code == 0 && c->cmd_crc_ok) {
            c->spi_mode = 1;
        } else {
            return;
        }
    }
    c->stats.commands++;

    if ((c->crc_on || idx == 0 || idx == 8) && !c->cmd_crc_ok) {
        c->stats.cmd_crc_errors++;
        c->app_cmd = 0;
        respond_r1(c, r1_base(c) | R1_COM_CRC);
        trace(c, SDEMU_EV_CMD_CRC, code, arg, r1_base(c) | R1_COM_CRC, 0);
        return;
    }

    bool block_cmd = code == 17 || code == 18 || code == 24 || code == 25;
    int fi = sdemu_fault_preview(ftab(c), SDEMU_STAGE_CMD, code, block_cmd ? arg : 0);
    sdemu_fault_commit(ftab(c), SDEMU_STAGE_CMD, code, block_cmd ? arg : 0, fi);
    if (fi >= 0) {
        const sdemu_fault_rule_t *r = &ftab(c)->rules[fi];
        c->stats.faults_fired++;
        trace(c, SDEMU_EV_FAULT, code, arg, 0xFF, r->action);
        if (r->action == SDEMU_FA_NO_RESPONSE) {
            c->app_cmd = 0;
            trace(c, SDEMU_EV_CMD, code, arg, 0xFF, 0);
            return;
        }
        if (r->action == SDEMU_FA_R1_ERROR) {
            c->app_cmd = 0;
            respond_r1(c, (uint8_t)(r1_base(c) | (r->param & 0x7F)));
            trace(c, SDEMU_EV_CMD, code, arg, c->resp[0], 0);
            return;
        }
        if (r->action == SDEMU_FA_DELAY_RESPONSE) {
            c->resp_delay = (uint16_t)r->param;
        }
    }
    c->app_cmd = 0;

    uint8_t base = r1_base(c);
    if (c->idle && !(code == 0 || code == 8 || code == 55 || code == 58 || code == 59 ||
                     code == SDEMU_ACMD(41))) {
        respond_r1(c, base | R1_ILLEGAL_CMD);
        trace(c, SDEMU_EV_CMD, code, arg, c->resp[0], 0);
        return;
    }

    switch (code) {
    case 0:   /* GO_IDLE_STATE */
        c->idle = 1;
        c->crc_on = 0;
        c->acmd41_count = 0;
        c->out_state = OS_IDLE;
        respond_r1(c, R1_IDLE);
        break;
    case 8:   /* SEND_IF_COND: echo voltage and check pattern (R7) */
        if (((arg >> 8) & 0x0F) != 0x01) {
            trace(c, SDEMU_EV_CMD, code, arg, 0xFF, 0);
            return;   /* voltage not supported: no response */
        }
        respond(c, AFTER_NONE, 5, base, 0x00, 0x00, (uint8_t)((arg >> 8) & 0x0F), (uint8_t)arg);
        break;
    case 58: { /* READ_OCR (R3) */
        uint32_t ocr = sdemu_ocr(c);
        respond(c, AFTER_NONE, 5, base, (uint8_t)(ocr >> 24), (uint8_t)(ocr >> 16),
                (uint8_t)(ocr >> 8), (uint8_t)ocr);
        break;
    }
    case 59:  /* CRC_ON_OFF */
        c->crc_on = arg & 1;
        respond_r1(c, base);
        break;
    case 55:  /* APP_CMD */
        c->app_cmd = 1;
        respond_r1(c, base);
        break;
    case SDEMU_ACMD(41): /* SD_SEND_OP_COND: needs HCS, SDHC never leaves idle without it */
        if ((arg & (1u << 30)) && ++c->acmd41_count >= c->cfg.acmd41_polls) {
            c->idle = 0;
        }
        respond_r1(c, r1_base(c));
        break;
    case 9:   /* SEND_CSD */
        start_read(c, c->csd, 16, base, 0, false);
        break;
    case 10:  /* SEND_CID */
        start_read(c, c->cid, 16, base, 0, false);
        break;
    case SDEMU_ACMD(51): /* SEND_SCR */
        start_read(c, c->scr, 8, base, 0, false);
        break;
    case SDEMU_ACMD(13): /* SD_STATUS: R2 + 64-byte block */
        start_read(c, c->ssr, 64, base, 0, true);
        break;
    case 13: { /* SEND_STATUS (R2) */
        uint8_t st = c->status_err ? 0x04 : 0x00;   /* "error" */
        c->status_err = 0;
        respond(c, AFTER_NONE, 2, base, st, 0, 0, 0);
        break;
    }
    case 16:  /* SET_BLOCKLEN: SDHC is fixed at 512 */
        respond_r1(c, arg == SDEMU_BLOCK_SIZE ? base : (uint8_t)(base | R1_PARAM));
        break;
    case SDEMU_ACMD(23): /* SET_WR_BLK_ERASE_COUNT: a hint, ignored */
        respond_r1(c, base);
        break;
    case 17:
    case 18:
        if (arg >= c->cfg.capacity_blocks) {
            respond_r1(c, base | R1_PARAM);
            break;
        }
        c->lba = arg;
        c->reg_src = NULL;
        c->multi = code == 18;
        respond(c, AFTER_READ, 1, base, 0, 0, 0, 0);
        break;
    case 24:
    case 25:
        if (arg >= c->cfg.capacity_blocks) {
            respond_r1(c, base | R1_PARAM);
            break;
        }
        c->lba = arg;
        c->multi = code == 25;
        respond(c, AFTER_WRITE, 1, base, 0, 0, 0, 0);
        break;
    case 12:  /* STOP_TRANSMISSION (R1b); the read was stopped in cmd_decode */
        respond(c, AFTER_BUSY, 1, base, 0, 0, 0, 0);
        break;
    default:  /* CMD5 (SDIO probe), CMD52, CMD6, erase, ...: illegal */
        respond_r1(c, base | R1_ILLEGAL_CMD);
        break;
    }
    trace(c, SDEMU_EV_CMD, code, arg, c->resp[0], 0);
}

/* ------------------------------------------------------------------------ */
/* byte interface                                                            */
/* ------------------------------------------------------------------------ */

uint8_t sdemu_tx(sdemu_t *c)
{
    if (c->cmd_stage == 1) {
        cmd_decode(c);
        return 0xFF;
    }
    if (c->cmd_stage == 2) {
        cmd_execute(c);
    }

    switch (c->out_state) {
    case OS_RESP: {
        if (c->resp_delay) {
            c->resp_delay--;
            return 0xFF;
        }
        uint8_t b = c->resp[c->resp_pos++];
        if (c->resp_pos >= c->resp_len) {
            switch (c->after_resp) {
            case AFTER_READ:  read_start(c); break;
            case AFTER_WRITE: c->out_state = OS_WR_TOKEN; break;
            case AFTER_BUSY:  busy_then(c, 1, AFTER_NONE); break;
            default:          c->out_state = OS_IDLE; break;
            }
        }
        return b;
    }
    case OS_RD_PREP:
        return tx_read_prep(c);
    case OS_RD_SEND:
        return tx_read_send(c);
    case OS_RD_ERRTOK:
        return tx_read_errtok(c);
    case OS_BUSY:
        if (c->busy_bytes) {
            c->busy_bytes--;
            return 0x00;
        }
        c->out_state = c->after_resp == AFTER_WRITE ? OS_WR_TOKEN : OS_IDLE;
        return 0xFF;
    case OS_WR_DATA:
        if (c->blk_pos == SDEMU_BLOCK_SIZE + 1) {
            /* this byte follows the last CRC byte, which hasn't arrived yet: guess */
            uint8_t g = c->wr_token;
            if (!g) {
                g = (c->crc_on && (c->crc16_rx >> 8) != (c->crc16 >> 8)) ? DR_CRC_ERROR : DR_ACCEPTED;
            }
            c->wr_guess = g;
            return g;
        }
        return 0xFF;
    case OS_WR_COMMIT:
        return tx_write_commit(c);
    default:
        return 0xFF;
    }
}

int sdemu_rx(sdemu_t *c, uint8_t b)
{
    if (c->out_state == OS_WR_DATA) {
        int16_t pos = c->blk_pos;
        if (pos < (int16_t)SDEMU_BLOCK_SIZE) {
            c->blk[pos] = b;
            c->crc16 = sdemu_crc16_byte(c->crc16, b);
            c->blk_pos = (int16_t)(pos + 1);
            return -1;
        }
        if (pos == (int16_t)SDEMU_BLOCK_SIZE) {
            c->crc16_rx = (uint16_t)(b << 8);
            c->blk_pos = (int16_t)(pos + 1);
            return -1;
        }
        c->crc16_rx |= b;
        c->blk_pos = (int16_t)(pos + 1);
        return write_block_end(c);
    }
    if (c->out_state == OS_WR_TOKEN) {
        if ((!c->multi && b == TOKEN_START) || (c->multi && b == TOKEN_START_MULTI_WR)) {
            write_block_begin(c);
            return -1;
        }
        if (c->multi && b == TOKEN_STOP_TRAN) {
            trace(c, SDEMU_EV_WRITE_STOP, c->cmd_code, c->lba, 0xFF, 0);
            busy_then(c, 1, AFTER_NONE);
            return -1;
        }
    }

    /* command assembler */
    if (c->cmd_len == 0) {
        if ((b & 0xC0) != 0x40 || c->cmd_stage != 0) {
            return -1;
        }
    }
    c->cmd_buf[c->cmd_len++] = b;
    if (c->cmd_len == 6) {
        c->cmd_len = 0;
        c->cmd_stage = 1;   /* the next tx() decodes it */
    }
    return -1;
}

/* ------------------------------------------------------------------------ */
/* faults (writer side) and stats                                            */
/* ------------------------------------------------------------------------ */

static void faults_publish(sdemu_t *c)
{
    for (;;) {
        uint32_t p = __atomic_load_n(&c->fpending, __ATOMIC_ACQUIRE);
        if (p == 1) {
            uint32_t one = 1;   /* take back a table the engine hasn't picked up yet */
            if (!__atomic_compare_exchange_n(&c->fpending, &one, 0, false, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE)) {
                continue;
            }
        } else if (p == 2) {
            continue;           /* engine is swapping right now */
        }
        break;
    }
    uint32_t inactive = __atomic_load_n(&c->factive, __ATOMIC_ACQUIRE) ^ 1u;
    c->ftab[inactive] = c->fmaster;
    __atomic_store_n(&c->fpending, 1, __ATOMIC_RELEASE);
}

bool sdemu_faults_add(sdemu_t *c, const sdemu_fault_rule_t *rule)
{
    if (c->fmaster.count >= SDEMU_MAX_RULES) {
        return false;
    }
    for (uint32_t i = 0; i < c->fmaster.count; i++) {
        c->fmaster.rules[i].matches = 0;
        c->fmaster.rules[i].fires = 0;
    }
    sdemu_fault_rule_t r = *rule;
    r.matches = 0;
    r.fires = 0;
    c->fmaster.rules[c->fmaster.count++] = r;
    faults_publish(c);
    return true;
}

void sdemu_faults_clear(sdemu_t *c)
{
    c->fmaster.count = 0;
    faults_publish(c);
}

void sdemu_faults_get(const sdemu_t *c, sdemu_fault_table_t *out)
{
    if (__atomic_load_n(&c->fpending, __ATOMIC_ACQUIRE) != 0) {
        *out = c->fmaster;   /* not picked up yet: counters are all zero */
    } else {
        *out = c->ftab[__atomic_load_n(&c->factive, __ATOMIC_ACQUIRE)];
    }
}

void sdemu_get_stats(const sdemu_t *c, sdemu_stats_t *out)
{
    *out = c->stats;
}

const char *sdemu_cmd_name(uint8_t cmd)
{
    switch (cmd) {
    case 0:  return "GO_IDLE_STATE";
    case 5:  return "IO_SEND_OP_COND";
    case 8:  return "SEND_IF_COND";
    case 9:  return "SEND_CSD";
    case 10: return "SEND_CID";
    case 12: return "STOP_TRANSMISSION";
    case 13: return "SEND_STATUS";
    case 16: return "SET_BLOCKLEN";
    case 17: return "READ_SINGLE_BLOCK";
    case 18: return "READ_MULTIPLE_BLOCK";
    case 24: return "WRITE_BLOCK";
    case 25: return "WRITE_MULTIPLE_BLOCK";
    case 52: return "IO_RW_DIRECT";
    case 55: return "APP_CMD";
    case 58: return "READ_OCR";
    case 59: return "CRC_ON_OFF";
    case SDEMU_ACMD(13): return "SD_STATUS";
    case SDEMU_ACMD(23): return "SET_WR_BLK_ERASE_COUNT";
    case SDEMU_ACMD(41): return "SD_SEND_OP_COND";
    case SDEMU_ACMD(51): return "SEND_SCR";
    default: return "?";
    }
}
