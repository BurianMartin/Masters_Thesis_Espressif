/*
 * Fault rules: names, parsing, formatting, and the matching used by the engine.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "sdemu_internal.h"

static const char *const s_action_names[SDEMU_FA__COUNT] = {
    [SDEMU_FA_NONE]            = "none",
    [SDEMU_FA_NO_RESPONSE]     = "no_response",
    [SDEMU_FA_DELAY_RESPONSE]  = "delay_response",
    [SDEMU_FA_R1_ERROR]        = "r1_error",
    [SDEMU_FA_READ_DELAY]      = "read_delay",
    [SDEMU_FA_ERROR_TOKEN]     = "error_token",
    [SDEMU_FA_CORRUPT_CRC16]   = "corrupt_crc16",
    [SDEMU_FA_FLIP_BYTE]       = "flip_byte",
    [SDEMU_FA_WRITE_ERROR]     = "write_error",
    [SDEMU_FA_WRITE_CRC_ERROR] = "write_crc_error",
    [SDEMU_FA_BUSY]            = "busy",
    [SDEMU_FA_DROP_WRITE]      = "drop_write",
};

const char *sdemu_fault_action_name(uint8_t action)
{
    return action < SDEMU_FA__COUNT ? s_action_names[action] : "?";
}

sdemu_fault_stage_t sdemu_fault_stage(uint8_t action)
{
    switch (action) {
    case SDEMU_FA_READ_DELAY:
    case SDEMU_FA_ERROR_TOKEN:
    case SDEMU_FA_CORRUPT_CRC16:
    case SDEMU_FA_FLIP_BYTE:
        return SDEMU_STAGE_READ;
    case SDEMU_FA_WRITE_ERROR:
    case SDEMU_FA_WRITE_CRC_ERROR:
    case SDEMU_FA_BUSY:
    case SDEMU_FA_DROP_WRITE:
        return SDEMU_STAGE_WRITE;
    default:
        return SDEMU_STAGE_CMD;
    }
}

void sdemu_fault_rule_default(sdemu_fault_rule_t *r)
{
    memset(r, 0, sizeof(*r));
    r->cmd = SDEMU_CMD_ANY;
    r->lba_first = 0;
    r->lba_last = 0xFFFFFFFFu;
    r->nth = 1;
}

/* ---- matching (hot path) ------------------------------------------------ */

static bool rule_matches(const sdemu_fault_rule_t *r, sdemu_fault_stage_t stage, uint8_t cmd, uint32_t lba)
{
    if (sdemu_fault_stage(r->action) != stage) {
        return false;
    }
    if (r->cmd != SDEMU_CMD_ANY && r->cmd != cmd) {
        return false;
    }
    return lba >= r->lba_first && lba <= r->lba_last;
}

static bool rule_would_fire(const sdemu_fault_rule_t *r)
{
    uint32_t m = r->matches + 1;       /* this match */
    uint32_t nth = r->nth ? r->nth : 1;
    if (r->max && r->fires >= r->max) {
        return false;
    }
    if (m < nth) {
        return false;
    }
    if (m == nth) {
        return true;
    }
    return r->every && ((m - nth) % r->every) == 0;
}

int sdemu_fault_preview(const sdemu_fault_table_t *t, sdemu_fault_stage_t stage, uint8_t cmd, uint32_t lba)
{
    for (uint32_t i = 0; i < t->count; i++) {
        const sdemu_fault_rule_t *r = &t->rules[i];
        if (rule_matches(r, stage, cmd, lba) && rule_would_fire(r)) {
            return (int)i;
        }
    }
    return -1;
}

void sdemu_fault_commit(sdemu_fault_table_t *t, sdemu_fault_stage_t stage, uint8_t cmd, uint32_t lba, int fired)
{
    for (uint32_t i = 0; i < t->count; i++) {
        sdemu_fault_rule_t *r = &t->rules[i];
        if (rule_matches(r, stage, cmd, lba)) {
            r->matches++;
            if ((int)i == fired) {
                r->fires++;
            }
        }
    }
}

/* ---- text form (control channel, not hot) ------------------------------- */

static bool parse_u32(const char *s, uint32_t *out)
{
    char *end;
    unsigned long v = strtoul(s, &end, 0);
    if (end == s || *end != '\0') {
        return false;
    }
    *out = (uint32_t)v;
    return true;
}

static bool parse_cmd(const char *s, uint8_t *out)
{
    uint32_t v;
    if (strcmp(s, "any") == 0) {
        *out = SDEMU_CMD_ANY;
        return true;
    }
    if ((s[0] == 'a' || s[0] == 'A') && parse_u32(s + 1, &v) && v < 64) {   /* a41 = ACMD41 */
        *out = (uint8_t)SDEMU_ACMD(v);
        return true;
    }
    if (strncmp(s, "acmd", 4) == 0 && parse_u32(s + 4, &v) && v < 64) {
        *out = (uint8_t)SDEMU_ACMD(v);
        return true;
    }
    if (strncmp(s, "cmd", 3) == 0) {
        s += 3;
    }
    if (!parse_u32(s, &v) || !(v < 64 || (v >= 100 && v < 164))) {
        return false;
    }
    *out = (uint8_t)v;
    return true;
}

bool sdemu_fault_parse(const char *text, sdemu_fault_rule_t *out, char *err, size_t err_len)
{
    char buf[160];
    sdemu_fault_rule_t r;
    sdemu_fault_rule_default(&r);
    bool have_action = false, have_mask = false;

    if (strlen(text) >= sizeof(buf)) {
        snprintf(err, err_len, "too long");
        return false;
    }
    strcpy(buf, text);

    for (char *tok = strtok(buf, " \t"); tok; tok = strtok(NULL, " \t")) {
        char *eq = strchr(tok, '=');
        if (!eq) {
            snprintf(err, err_len, "expected key=value: %s", tok);
            return false;
        }
        *eq = '\0';
        const char *key = tok, *val = eq + 1;
        bool ok = true;
        if (strcmp(key, "action") == 0) {
            ok = false;
            for (int a = 1; a < SDEMU_FA__COUNT; a++) {
                if (strcmp(val, s_action_names[a]) == 0) {
                    r.action = (uint8_t)a;
                    ok = have_action = true;
                }
            }
        } else if (strcmp(key, "cmd") == 0) {
            ok = parse_cmd(val, &r.cmd);
        } else if (strcmp(key, "lba") == 0) {
            char *dash = strchr(val, '-');
            if (dash) {
                *dash = '\0';
                ok = parse_u32(val, &r.lba_first) && parse_u32(dash + 1, &r.lba_last);
            } else {
                ok = parse_u32(val, &r.lba_first);
                r.lba_last = r.lba_first;
            }
        } else if (strcmp(key, "nth") == 0) {
            ok = parse_u32(val, &r.nth);
        } else if (strcmp(key, "every") == 0) {
            ok = parse_u32(val, &r.every);
        } else if (strcmp(key, "max") == 0) {
            ok = parse_u32(val, &r.max);
        } else if (strcmp(key, "param") == 0 || strcmp(key, "ms") == 0 || strcmp(key, "bytes") == 0 ||
                   strcmp(key, "offset") == 0 || strcmp(key, "token") == 0 || strcmp(key, "bits") == 0) {
            ok = parse_u32(val, &r.param);
        } else if (strcmp(key, "mask") == 0 || strcmp(key, "param2") == 0) {
            ok = parse_u32(val, &r.param2);
            have_mask = true;
        } else {
            snprintf(err, err_len, "unknown key: %s", key);
            return false;
        }
        if (!ok) {
            snprintf(err, err_len, "bad value for %s: %s", key, val);
            return false;
        }
    }
    if (!have_action) {
        snprintf(err, err_len, "action= missing");
        return false;
    }
    if (r.lba_last < r.lba_first) {
        snprintf(err, err_len, "empty lba range");
        return false;
    }
    if (r.action == SDEMU_FA_FLIP_BYTE) {
        if (!have_mask) {
            r.param2 = 0xFF;
        }
        if (r.param >= SDEMU_BLOCK_SIZE || (r.param2 & 0xFF) == 0) {
            snprintf(err, err_len, "flip_byte needs offset < 512 and a non-zero mask");
            return false;
        }
    }
    if (r.action == SDEMU_FA_ERROR_TOKEN) {
        if (r.param == 0) {
            r.param = DE_ERROR;
        }
        if (r.param > 0x0F) {
            snprintf(err, err_len, "error token must be 0x01..0x0F");
            return false;
        }
    }
    if (r.action == SDEMU_FA_R1_ERROR && (r.param == 0 || r.param > 0x7F)) {
        snprintf(err, err_len, "r1_error needs bits=0x01..0x7F");
        return false;
    }
    *out = r;
    return true;
}

int sdemu_fault_format(const sdemu_fault_rule_t *r, char *buf, size_t len)
{
    char cmd[12];
    if (r->cmd == SDEMU_CMD_ANY) {
        strcpy(cmd, "any");
    } else if (r->cmd >= 100) {
        snprintf(cmd, sizeof(cmd), "acmd%u", (unsigned)(r->cmd - 100));
    } else {
        snprintf(cmd, sizeof(cmd), "%u", (unsigned)r->cmd);
    }
    int n = snprintf(buf, len, "action=%s cmd=%s", sdemu_fault_action_name(r->action), cmd);
    if (r->lba_first != 0 || r->lba_last != 0xFFFFFFFFu) {
        n += snprintf(buf + n, len - (size_t)n, " lba=%lu-%lu",
                      (unsigned long)r->lba_first, (unsigned long)r->lba_last);
    }
    n += snprintf(buf + n, len - (size_t)n, " nth=%lu every=%lu max=%lu param=%lu mask=0x%02lx matches=%lu fires=%lu",
                  (unsigned long)r->nth, (unsigned long)r->every, (unsigned long)r->max,
                  (unsigned long)r->param, (unsigned long)r->param2,
                  (unsigned long)r->matches, (unsigned long)r->fires);
    return n;
}
