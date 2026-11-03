/*
 * Card registers (spec chapter 5): OCR, CID, CSD v2.0, SCR, SD status.
 * Only what an SDHC card in SPI mode needs.
 */
#include <string.h>
#include "sdemu_internal.h"

#define OCR_BUSY   (1u << 31)   /* 1 = power-up finished */
#define OCR_CCS    (1u << 30)   /* card capacity status: 1 = SDHC/SDXC */
#define OCR_VDD    0x00FF8000u  /* 2.7-3.6 V */
/* bit 24 (S18A) must stay 0: ESP-IDF would start the UHS-I voltage switch */

uint32_t sdemu_ocr(const sdemu_t *card)
{
    uint32_t ocr = OCR_VDD;
    if (!card->idle) {
        ocr |= OCR_BUSY | OCR_CCS;   /* CCS is only valid once the busy bit is set */
    }
    return ocr;
}

static void build_cid(sdemu_t *card)
{
    const sdemu_config_t *c = &card->cfg;
    uint8_t *r = card->cid;
    memset(r, 0, 16);
    r[0] = c->manufacturer_id;                    /* MID      [127:120] */
    r[1] = (uint8_t)c->oem_id[0];                 /* OID      [119:104] */
    r[2] = (uint8_t)c->oem_id[1];
    for (int i = 0; i < 5; i++) {                 /* PNM      [103:64]  */
        r[3 + i] = (uint8_t)(c->product_name[i] ? c->product_name[i] : ' ');
    }
    r[8] = c->product_rev;                        /* PRV      [63:56]   */
    r[9] = (uint8_t)(c->serial >> 24);            /* PSN      [55:24]   */
    r[10] = (uint8_t)(c->serial >> 16);
    r[11] = (uint8_t)(c->serial >> 8);
    r[12] = (uint8_t)c->serial;
    uint16_t y = (uint16_t)(c->mfg_year >= 2000 ? c->mfg_year - 2000 : 0);
    uint16_t mdt = (uint16_t)(((y & 0xFF) << 4) | (c->mfg_month & 0x0F));
    r[13] = (uint8_t)((mdt >> 8) & 0x0F);         /* reserved [23:20], MDT [19:8] */
    r[14] = (uint8_t)mdt;
    r[15] = (uint8_t)((sdemu_crc7(r, 15) << 1) | 1);
}

static void build_csd_v2(sdemu_t *card)
{
    /* CSD version 2.0 (spec 5.3.3). capacity = (C_SIZE + 1) * 512 KiB */
    uint32_t c_size = card->cfg.capacity_blocks / 1024u - 1u;
    uint8_t *r = card->csd;
    r[0] = 0x40;           /* CSD_STRUCTURE = 1 (v2.0) */
    r[1] = 0x0E;           /* TAAC: 1 ms */
    r[2] = 0x00;           /* NSAC */
    r[3] = 0x32;           /* TRAN_SPEED: 25 MHz */
    r[4] = 0x5B;           /* CCC [11:4] = 0x5B5: classes 0,2,4,5,7,8,10 */
    r[5] = 0x59;           /* CCC [3:0] = 5, READ_BL_LEN = 9 (512) */
    r[6] = 0x00;           /* READ_BL_PARTIAL, misalign, DSR_IMP = 0 */
    r[7] = (uint8_t)((c_size >> 16) & 0x3F);
    r[8] = (uint8_t)(c_size >> 8);
    r[9] = (uint8_t)c_size;
    r[10] = 0x7F;          /* ERASE_BLK_EN = 1, SECTOR_SIZE[6:1] */
    r[11] = 0x80;          /* SECTOR_SIZE[0] = 1, WP_GRP_SIZE = 0 */
    r[12] = 0x0A;          /* WP_GRP_ENABLE 0, R2W_FACTOR = 2, WRITE_BL_LEN[3:2] */
    r[13] = 0x40;          /* WRITE_BL_LEN[1:0] = 9, WRITE_BL_PARTIAL = 0 */
    r[14] = 0x00;          /* FILE_FORMAT_GRP, COPY, PERM/TMP_WRITE_PROTECT */
    r[15] = (uint8_t)((sdemu_crc7(r, 15) << 1) | 1);
}

static void build_scr(sdemu_t *card)
{
    uint8_t *r = card->scr;
    memset(r, 0, 8);
    r[0] = 0x02;   /* SCR_STRUCTURE = 0 (ESP-IDF checks), SD_SPEC = 2 (v2.00+) */
    r[1] = 0x35;   /* DATA_STAT_AFTER_ERASE 0, SD_SECURITY = 3 (SDHC), BUS_WIDTHS = 1|4 bit */
    r[2] = 0x80;   /* SD_SPEC3 = 1 */
}

static void build_ssr(sdemu_t *card)
{
    /* SD status (ACMD13, spec 4.10.2): 512 bits, MSB first. */
    uint8_t *r = card->ssr;
    memset(r, 0, 64);
    r[8] = 0x02;   /* SPEED_CLASS = 2 (class 4) */
    r[9] = 0x02;   /* PERFORMANCE_MOVE = 2 MB/s */
    r[10] = 0x90;  /* AU_SIZE = 9 (4 MiB) */
    r[11] = 0x00;  /* ERASE_SIZE = 1 AU */
    r[12] = 0x01;
    r[13] = 0x08;  /* ERASE_TIMEOUT = 2 s, ERASE_OFFSET = 0 */
}

void sdemu_build_registers(sdemu_t *card)
{
    build_cid(card);
    build_csd_v2(card);
    build_scr(card);
    build_ssr(card);
}
