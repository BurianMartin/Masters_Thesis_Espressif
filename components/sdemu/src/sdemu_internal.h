/* Internal helpers shared by the engine's source files. Not public API. */
#pragma once

#include "sdemu.h"

/* R1 bits (spec 7.3.2.1) */
#define R1_IDLE          0x01
#define R1_ERASE_RESET   0x02
#define R1_ILLEGAL_CMD   0x04
#define R1_COM_CRC       0x08
#define R1_ERASE_SEQ     0x10
#define R1_ADDRESS       0x20
#define R1_PARAM         0x40

/* tokens (spec 7.3.3) */
#define TOKEN_START          0xFE
#define TOKEN_START_MULTI_WR 0xFC
#define TOKEN_STOP_TRAN      0xFD
#define DR_ACCEPTED          0x05
#define DR_CRC_ERROR         0x0B
#define DR_WRITE_ERROR       0x0D
#define DE_ERROR             0x01
#define DE_OUT_OF_RANGE      0x08

/* CRC */
uint8_t sdemu_crc7(const uint8_t *data, size_t len);
extern const uint16_t sdemu_crc16_table[256];
static inline uint16_t sdemu_crc16_byte(uint16_t crc, uint8_t b)
{
    return (uint16_t)((crc << 8) ^ sdemu_crc16_table[((crc >> 8) ^ b) & 0xFF]);
}
uint16_t sdemu_crc16(const uint8_t *data, size_t len);

/* registers */
void sdemu_build_registers(sdemu_t *card);
uint32_t sdemu_ocr(const sdemu_t *card);

/*
 * Fault matching. preview() finds the rule that would fire for the next match
 * without counting it; commit() counts the match for every rule that matches
 * (and the fire for the one that was previewed).
 */
int sdemu_fault_preview(const sdemu_fault_table_t *t, sdemu_fault_stage_t stage, uint8_t cmd, uint32_t lba);
void sdemu_fault_commit(sdemu_fault_table_t *t, sdemu_fault_stage_t stage, uint8_t cmd, uint32_t lba, int fired);
