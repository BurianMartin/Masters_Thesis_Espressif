/*
 * Laptop model of the ESP-IDF SDSPI host. What it copies from the real host
 * (see docs/DESIGN.md section 1):
 *
 *  - command transactions of fixed size: 6 command bytes + 8 NCR + response
 *    (15 bytes for R1, 16 for R2, 19 for R3/R7), R1 searched in the 2nd..9th
 *    byte after the command (never the 1st); no retry when it isn't there
 *  - after CMD12 the 2nd byte is a stuff byte and ignored as well
 *  - data tokens polled in separate 8-byte transactions; only 0xFF and 0x00
 *    are allowed before 0xFE; 1 s timeout
 *  - writes: token + data in one transaction, then CRC16 + data response in a
 *    3-byte transaction (response = the byte right after the CRC), then busy
 *    polled 1 byte at a time
 *  - CMD13 after every read and twice after every write; failures there are
 *    only logged
 *  - init: CMD52, CMD0 x2, CMD8, CMD5, CMD59, ACMD41 loop, CMD58, CMD10, CMD9,
 *    ACMD51, ACMD13, ACMD51 again (compared)
 */
#include <stdio.h>
#include <string.h>
#include "sim_host.h"
#include "../../components/sdemu/src/sdemu_internal.h"

#define BYTE_US   20u    /* 8 bits at 400 kHz */
#define GAP_US    30u    /* host pause between transactions (CS stays low) */

#define LOG(h, ...) do { if ((h)->verbose) printf("    host: " __VA_ARGS__); } while (0)

uint32_t sim_clock(void *ctx)
{
    return ((sim_host_t *)ctx)->t_us;
}

void sim_host_init(sim_host_t *h, sdemu_t *card)
{
    memset(h, 0, sizeof(*h));
    h->card = card;
}

const char *sim_err_name(int err)
{
    switch (err) {
    case SIM_OK: return "ESP_OK";
    case SIM_ERR_TIMEOUT: return "ESP_ERR_TIMEOUT";
    case SIM_ERR_INVALID_CRC: return "ESP_ERR_INVALID_CRC";
    case SIM_ERR_INVALID_RESPONSE: return "ESP_ERR_INVALID_RESPONSE";
    case SIM_ERR_NOT_SUPPORTED: return "ESP_ERR_NOT_SUPPORTED";
    case SIM_ERR_INVALID_ARG: return "ESP_ERR_INVALID_ARG";
    case SIM_ERR_FAIL: return "ESP_FAIL";
    default: return "?";
    }
}

/* ---- the wire: same pipeline as the S3 transport ------------------------ */

void sim_delay_us(sim_host_t *h, uint32_t us)
{
    h->t_us += us;
}

void sim_cs(sim_host_t *h, bool low)
{
    if (low && !h->selected) {
        h->selected = true;
        sdemu_select(h->card);
        h->cur = sdemu_tx(h->card);
    } else if (!low && h->selected) {
        h->selected = false;
        sdemu_deselect(h->card);
    }
    h->t_us += 2;
}

uint8_t sim_byte(sim_host_t *h, uint8_t mosi)
{
    h->t_us += BYTE_US;
    h->bytes++;
    if (!h->selected) {
        return 0xFF;   /* pull-up: card is not driving MISO */
    }
    uint8_t miso = h->cur;
    uint8_t next = sdemu_tx(h->card);          /* middle of the byte */
    int fix = sdemu_rx(h->card, mosi);         /* end of the byte */
    if (fix >= 0) {
        if ((fix & 0x80) != (next & 0x80)) {
            printf("    PIPELINE VIOLATION: correction changes bit 7 (%02x -> %02x)\n", next, fix);
            h->pipeline_violations++;
        }
        h->corrections++;
        next = (uint8_t)fix;
    }
    h->cur = next;
    return miso;
}

static void xfer(sim_host_t *h, const uint8_t *tx, uint8_t *rx, size_t n)
{
    for (size_t i = 0; i < n; i++) {
        uint8_t b = sim_byte(h, tx ? tx[i] : 0xFF);
        if (rx) {
            rx[i] = b;
        }
    }
}

/* ---- commands ------------------------------------------------------------ */

static int r1_to_err(uint8_t r1)
{
    if (r1 & R1_COM_CRC) return SIM_ERR_INVALID_CRC;
    if (r1 & R1_ILLEGAL_CMD) return SIM_ERR_NOT_SUPPORTED;
    if (r1 & (R1_ADDRESS | R1_PARAM)) return SIM_ERR_INVALID_ARG;
    if (r1 & (R1_ERASE_RESET | R1_ERASE_SEQ)) return SIM_ERR_INVALID_RESPONSE;
    return SIM_OK;
}

int sim_cmd(sim_host_t *h, uint8_t idx, uint32_t arg, int resp_len, uint8_t *resp, bool bad_crc)
{
    uint8_t tx[19], rx[19];
    int size = 6 + 8 + resp_len;
    memset(tx, 0xFF, sizeof(tx));
    tx[0] = (uint8_t)(0x40 | idx);
    tx[1] = (uint8_t)(arg >> 24);
    tx[2] = (uint8_t)(arg >> 16);
    tx[3] = (uint8_t)(arg >> 8);
    tx[4] = (uint8_t)arg;
    tx[5] = (uint8_t)((sdemu_crc7(tx, 5) << 1) | 1);
    if (bad_crc) {
        tx[5] ^= 0x10;
    }
    xfer(h, tx, rx, (size_t)size);

    /* shift_cmd_response(): R1 = first byte with bit 7 clear, from the 2nd byte on */
    int first = 7;
    if (idx == 12) {
        first = 8;   /* stuff byte after CMD12 */
    }
    int pos = -1;
    for (int i = first; i < 6 + 1 + 8; i++) {
        if ((rx[i] & 0x80) == 0) {
            pos = i;
            break;
        }
    }
    if (pos < 0) {
        LOG(h, "CMD%u: no response\n", idx);
        h->last_r1 = 0xFF;
        return SIM_ERR_TIMEOUT;
    }
    if (pos + resp_len > size) {
        LOG(h, "CMD%u: response truncated\n", idx);
        return SIM_ERR_TIMEOUT;
    }
    memcpy(resp, &rx[pos], (size_t)resp_len);
    h->last_r1 = resp[0];
    LOG(h, "CMD%-2u arg=%08x -> R1 %02x at byte %d\n", idx, arg, resp[0], pos - 5);
    return r1_to_err(resp[0]);
}

static void pre_command_poll(sim_host_t *h)
{
    /* with CS high, read until two non-zero bytes (always 0xFF here) */
    sim_cs(h, false);
    xfer(h, NULL, NULL, 2);
}

static int cmd_txn(sim_host_t *h, uint8_t idx, uint32_t arg, int resp_len, uint8_t *resp)
{
    pre_command_poll(h);
    sim_cs(h, true);
    int err = sim_cmd(h, idx, arg, resp_len, resp, false);
    sim_cs(h, false);
    return err;
}

static int acmd_txn(sim_host_t *h, uint8_t idx, uint32_t arg, int resp_len, uint8_t *resp)
{
    uint8_t r[1];
    int err = cmd_txn(h, 55, 0, 1, r);
    if (err) {
        return err;
    }
    return cmd_txn(h, idx, arg, resp_len, resp);
}

/* ---- data phases ---------------------------------------------------------- */

static int poll_busy(sim_host_t *h, uint32_t timeout_ms)
{
    uint32_t t0 = h->t_us;
    for (;;) {
        sim_delay_us(h, GAP_US);
        if (sim_byte(h, 0xFF) != 0x00) {
            return SIM_OK;
        }
        if (h->t_us - t0 >= timeout_ms * 1000u) {
            return SIM_ERR_TIMEOUT;
        }
    }
}

/* poll_data_token() + the rest of the block */
static int read_data(sim_host_t *h, uint8_t *dst, size_t len)
{
    uint8_t rx[8];
    uint8_t extra[8];
    size_t n_extra = 0;
    uint32_t t0 = h->t_us;
    for (;;) {
        sim_delay_us(h, GAP_US);
        xfer(h, NULL, rx, 8);
        int found = -1;
        for (int i = 0; i < 8; i++) {
            if (rx[i] == TOKEN_START) {
                found = i;
                break;
            }
            if (rx[i] != 0xFF && rx[i] != 0x00) {
                LOG(h, "poll_data_token: received 0x%02x while waiting for data\n", rx[i]);
                return SIM_ERR_INVALID_RESPONSE;
            }
        }
        if (found >= 0) {
            n_extra = (size_t)(8 - found - 1);
            memcpy(extra, &rx[found + 1], n_extra);
            break;
        }
        if (h->t_us - t0 >= 1000000u) {
            return SIM_ERR_TIMEOUT;
        }
    }
    uint8_t blk[SDEMU_BLOCK_SIZE + 2];
    memcpy(blk, extra, n_extra);
    sim_delay_us(h, GAP_US);
    xfer(h, NULL, blk + n_extra, len + 2 - n_extra);
    uint16_t crc = (uint16_t)((blk[len] << 8) | blk[len + 1]);
    if (h->crc_enabled && crc != sdemu_crc16(blk, len)) {
        LOG(h, "data CRC mismatch\n");
        return SIM_ERR_INVALID_CRC;
    }
    memcpy(dst, blk, len);
    return SIM_OK;
}

static int read_register(sim_host_t *h, uint8_t idx, bool app, uint8_t *dst, size_t len, int resp_len)
{
    uint8_t r[2];
    if (app) {
        int err = cmd_txn(h, 55, 0, 1, r);
        if (err) {
            return err;
        }
    }
    pre_command_poll(h);
    sim_cs(h, true);
    int err = sim_cmd(h, idx, 0, resp_len, r, false);
    if (!err) {
        err = read_data(h, dst, len);
    }
    sim_cs(h, false);
    return err;
}

int sim_send_status(sim_host_t *h, uint8_t *r2)
{
    uint8_t tmp[2];
    int err = cmd_txn(h, 13, 0, 2, r2 ? r2 : tmp);
    if (!err && (r2 ? r2 : tmp)[1] != 0) {
        err = SIM_ERR_INVALID_RESPONSE;
    }
    return err;
}

/* ---- sdmmc_card_init() ---------------------------------------------------- */

int sim_card_init(sim_host_t *h, sim_card_info_t *info)
{
    uint8_t r[5];
    int err;
    memset(info, 0, sizeof(*info));
    h->crc_enabled = false;

    /* SDIO reset probe, sent while the card is still in SD mode; result ignored */
    cmd_txn(h, 52, 0x80000C08u, 1, r);

    /* go idle twice, 80 clocks with CS high before each, first answer ignored */
    for (int i = 0; i < 2; i++) {
        sim_cs(h, false);
        xfer(h, NULL, NULL, 10);
        sim_delay_us(h, 20000);
        sim_cs(h, true);
        err = sim_cmd(h, 0, 0, 1, r, false);
        sim_cs(h, false);
        if (i == 1 && (err || r[0] != R1_IDLE)) {
            LOG(h, "CMD0 failed\n");
            return err ? err : SIM_ERR_INVALID_RESPONSE;
        }
    }

    /* CMD8: a card that does not answer is treated as version 1 */
    bool v2 = true;
    err = cmd_txn(h, 8, 0x1AA, 5, r);
    if (err == SIM_ERR_TIMEOUT || err == SIM_ERR_NOT_SUPPORTED) {
        v2 = false;
        LOG(h, "CMD8 failed: assuming SD v1 card\n");
    } else if (err) {
        return err;
    } else if (r[4] != 0xAA) {
        return SIM_ERR_INVALID_RESPONSE;
    }

    /* CMD5: SDIO probe, the card must refuse */
    err = cmd_txn(h, 5, 0, 1, r);
    if (err == SIM_OK) {
        LOG(h, "card answered CMD5: treated as SDIO\n");
        return SIM_ERR_NOT_SUPPORTED;
    }

    /* CRC on by default */
    err = cmd_txn(h, 59, 1, 1, r);
    if (err) {
        return err;
    }
    h->crc_enabled = true;

    /* ACMD41 until ready: up to 300 tries, 10 ms apart */
    uint32_t hcs = v2 ? (1u << 30) : 0;
    bool ready = false;
    for (int i = 0; i < 300; i++) {
        err = acmd_txn(h, 41, hcs, 1, r);
        if (err) {
            return err;
        }
        if ((r[0] & R1_IDLE) == 0) {
            ready = true;
            break;
        }
        sim_delay_us(h, 10000);
    }
    if (!ready) {
        LOG(h, "ACMD41 timeout\n");
        return SIM_ERR_TIMEOUT;
    }

    err = cmd_txn(h, 58, 0, 5, r);
    if (err) {
        return err;
    }
    info->ocr = ((uint32_t)r[1] << 24) | ((uint32_t)r[2] << 16) | ((uint32_t)r[3] << 8) | r[4];
    if (info->ocr & (1u << 24)) {
        LOG(h, "S18A set: would start UHS-I voltage switch\n");
        return SIM_ERR_NOT_SUPPORTED;
    }
    info->sdhc = (info->ocr & (1u << 30)) != 0;

    if ((err = read_register(h, 10, false, info->cid, 16, 1))) return err;
    if ((err = read_register(h, 9, false, info->csd, 16, 1))) return err;
    uint8_t csd_structure = info->csd[0] >> 6;
    if (csd_structure > 1) {
        return SIM_ERR_NOT_SUPPORTED;
    }
    if (csd_structure == 1) {
        uint32_t c_size = ((uint32_t)(info->csd[7] & 0x3F) << 16) | ((uint32_t)info->csd[8] << 8) | info->csd[9];
        info->capacity_blocks = (c_size + 1) * 1024u;
    }
    if ((err = read_register(h, 51, true, info->scr, 8, 1))) return err;
    if ((info->scr[0] >> 4) != 0) {
        return SIM_ERR_NOT_SUPPORTED;
    }
    uint8_t ssr[64];
    if ((err = read_register(h, 13, true, ssr, 64, 2))) return err;
    /* (clock switch to the configured maximum would happen here) */
    uint8_t scr2[8];
    if ((err = read_register(h, 51, true, scr2, 8, 1))) return err;
    if (memcmp(scr2, info->scr, 8) != 0) {
        return SIM_ERR_INVALID_RESPONSE;
    }
    memcpy(info->name, &info->cid[3], 5);
    info->name[5] = '\0';
    return SIM_OK;
}

/* ---- sector access ------------------------------------------------------- */

static void status_after(sim_host_t *h, int times)
{
    for (int i = 0; i < times; i++) {
        int err = sim_send_status(h, NULL);
        if (err) {
            h->cmd13_failures++;
            LOG(h, "CMD13 after transfer failed (%s), only logged\n", sim_err_name(err));
        }
    }
}

int sim_read_blocks(sim_host_t *h, uint32_t lba, uint32_t n, uint8_t *buf)
{
    uint8_t r[1];
    bool multi = n > 1;
    pre_command_poll(h);
    sim_cs(h, true);
    int err = sim_cmd(h, multi ? 18 : 17, lba, 1, r, false);
    for (uint32_t i = 0; !err && i < n; i++) {
        err = read_data(h, buf + (size_t)i * SDEMU_BLOCK_SIZE, SDEMU_BLOCK_SIZE);
    }
    if (multi && h->last_r1 != 0xFF) {
        uint8_t r12[1];
        int e12 = sim_cmd(h, 12, 0, 1, r12, false);
        if (!e12) {
            e12 = poll_busy(h, 1000);
        }
        if (!err) {
            err = e12;
        }
    }
    sim_cs(h, false);
    if (!err) {
        status_after(h, 1);
    }
    return err;
}

int sim_write_blocks(sim_host_t *h, uint32_t lba, uint32_t n, const uint8_t *buf)
{
    uint8_t r[1];
    bool multi = n > 1;
    pre_command_poll(h);
    sim_cs(h, true);
    int err = sim_cmd(h, multi ? 25 : 24, lba, 1, r, false);
    for (uint32_t i = 0; !err && i < n; i++) {
        const uint8_t *src = buf + (size_t)i * SDEMU_BLOCK_SIZE;
        uint8_t tok = multi ? TOKEN_START_MULTI_WR : TOKEN_START;
        sim_delay_us(h, GAP_US);
        sim_byte(h, 0xFF);                       /* N_WR */
        sim_byte(h, tok);
        xfer(h, src, NULL, SDEMU_BLOCK_SIZE);
        uint16_t crc = (uint16_t)(sdemu_crc16(src, SDEMU_BLOCK_SIZE) ^ h->write_crc_xor);
        h->write_crc_xor = 0;
        uint8_t tx3[3] = { (uint8_t)(crc >> 8), (uint8_t)crc, 0xFF }, rx3[3];
        xfer(h, tx3, rx3, 3);
        uint8_t resp = rx3[2] & 0x1F;
        if (resp == DR_CRC_ERROR) {
            err = SIM_ERR_INVALID_CRC;
        } else if (resp == DR_WRITE_ERROR) {
            err = SIM_ERR_FAIL;
        } else if (resp != DR_ACCEPTED) {
            LOG(h, "bad data response 0x%02x\n", rx3[2]);
            err = SIM_ERR_INVALID_RESPONSE;
        }
        if (err) {
            LOG(h, "write block %u: data response 0x%02x\n", lba + i, rx3[2]);
            break;
        }
        err = poll_busy(h, 5000);
    }
    if (multi && !err) {
        sim_byte(h, TOKEN_STOP_TRAN);
        sim_byte(h, 0xFF);
        err = poll_busy(h, 5000);
    }
    sim_cs(h, false);
    if (!err) {
        status_after(h, 2);
    }
    return err;
}
