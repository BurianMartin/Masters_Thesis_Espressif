/*
 * Portable storage backends: a plain RAM disk and a pattern disk.
 * The ESP32-S3 port adds a staged PSRAM backend (port/esp32s3/sdemu_esp.c).
 */
#include <string.h>
#include "sdemu_internal.h"

/* ---- RAM disk ----------------------------------------------------------- */

static sdemu_io_t ram_read(void *ctx, uint32_t lba, uint32_t off, uint8_t *dst, uint32_t len)
{
    sdemu_ram_backend_t *be = ctx;
    if (lba >= be->blocks) {
        return SDEMU_IO_ERROR;
    }
    memcpy(dst, be->mem + (size_t)lba * SDEMU_BLOCK_SIZE + off, len);
    return SDEMU_IO_OK;
}

static sdemu_io_t ram_write(void *ctx, uint32_t lba, uint32_t off, const uint8_t *src, uint32_t len)
{
    sdemu_ram_backend_t *be = ctx;
    (void)lba;
    /* Stage into wbuf: the block only becomes visible on commit, like a real card. */
    memcpy(be->wbuf + off, src, len);
    return SDEMU_IO_OK;
}

static sdemu_io_t ram_commit(void *ctx, uint32_t lba)
{
    sdemu_ram_backend_t *be = ctx;
    if (lba >= be->blocks) {
        return SDEMU_IO_ERROR;
    }
    memcpy(be->mem + (size_t)lba * SDEMU_BLOCK_SIZE, be->wbuf, SDEMU_BLOCK_SIZE);
    return SDEMU_IO_OK;
}

void sdemu_ram_backend_init(sdemu_ram_backend_t *be, sdemu_storage_t *out, uint8_t *mem, uint32_t blocks)
{
    be->mem = mem;
    be->blocks = blocks;
    out->ctx = be;
    out->read = ram_read;
    out->write = ram_write;
    out->commit = ram_commit;
}

/* ---- pattern disk ------------------------------------------------------- */

uint8_t sdemu_pattern_byte(uint32_t lba, uint32_t off)
{
    uint32_t x = lba * 2654435761u + off * 40503u;
    return (uint8_t)(x >> 13);
}

static sdemu_io_t pat_read(void *ctx, uint32_t lba, uint32_t off, uint8_t *dst, uint32_t len)
{
    (void)ctx;
    for (uint32_t i = 0; i < len; i++) {
        dst[i] = sdemu_pattern_byte(lba, off + i);
    }
    return SDEMU_IO_OK;
}

static sdemu_io_t pat_write(void *ctx, uint32_t lba, uint32_t off, const uint8_t *src, uint32_t len)
{
    (void)ctx; (void)lba; (void)off; (void)src; (void)len;
    return SDEMU_IO_OK;
}

static sdemu_io_t pat_commit(void *ctx, uint32_t lba)
{
    (void)ctx; (void)lba;
    return SDEMU_IO_OK;
}

void sdemu_pattern_backend_init(sdemu_storage_t *out)
{
    out->ctx = NULL;
    out->read = pat_read;
    out->write = pat_write;
    out->commit = pat_commit;
}
