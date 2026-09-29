#include "glasses_core.h"
#include <string.h>

void glasses_rx_reset(GlassesRx *rx) { rx->used = 0; rx->next = 0; rx->total = 0; }
void glasses_rx_expire(GlassesRx *rx, uint32_t now) {
    if (rx->total && (uint32_t)(now - rx->last) >= GLASSES_RX_TIMEOUT_MS) {
        ++rx->rejected; glasses_rx_reset(rx);
    }
}
/* Decode whole UTF-8 messages, including code points spanning fragments.
 * The display font is ASCII: German umlauts are expanded, other glyphs use ?. */
static bool text_decode(const uint8_t *raw, size_t len, char *out, bool *truncated) {
    size_t i = 0, n = 0;
    while (i < len) {
        uint32_t cp = raw[i++], minimum = 0;
        unsigned extra = 0;
        const char *replacement = NULL;
        char ascii[2] = { '?', 0 };
        if (!cp) { /* Only the last fragment may contain zero padding. */
            while (i < len) if (raw[i++]) return false;
            break;
        }
        if (cp >= 0xC2 && cp <= 0xDF) { extra = 1; cp &= 31; minimum = 0x80; }
        else if (cp >= 0xE0 && cp <= 0xEF) { extra = 2; cp &= 15; minimum = 0x800; }
        else if (cp >= 0xF0 && cp <= 0xF4) { extra = 3; cp &= 7; minimum = 0x10000; }
        else if (cp >= 0x80) return false;
        if (i + extra > len) return false;
        while (extra--) {
            uint8_t c = raw[i++];
            if ((c & 0xC0) != 0x80) return false;
            cp = (cp << 6) | (c & 63);
        }
        if (cp < minimum || cp > 0x10FFFF || (cp >= 0xD800 && cp <= 0xDFFF)) return false;
        switch (cp) {
        case 0xE4: replacement = "ae"; break; case 0xF6: replacement = "oe"; break;
        case 0xFC: replacement = "ue"; break; case 0xC4: replacement = "Ae"; break;
        case 0xD6: replacement = "Oe"; break; case 0xDC: replacement = "Ue"; break;
        case 0xDF: replacement = "ss"; break;
        default: ascii[0] = cp >= 32 && cp <= 126 ? (char)cp : cp < 32 ? ' ' : '?';
        }
        if (!replacement) replacement = ascii;
        while (*replacement) {
            if (n + 1 < GLASSES_TEXT_SIZE) out[n++] = *replacement;
            else *truncated = true;
            ++replacement;
        }
    }
    out[n] = 0;
    return n != 0;
}
bool glasses_rx_push(GlassesRx *rx, const uint8_t *data, size_t len, uint32_t now) {
    bool truncated = false;
    uint8_t slot;
    glasses_rx_expire(rx, now);
    if (!data || len < 3 || len > 20 || data[1] == 0 || data[1] > GLASSES_RX_SIZE / 18 || data[0] >= data[1]) goto reject;
    if (data[0] == 0) { glasses_rx_reset(rx); rx->total = data[1]; }
    if (!rx->total || data[1] != rx->total || data[0] != rx->next ||
        (data[0] + 1 < data[1] && len != 20) || rx->used + len - 2 > GLASSES_RX_SIZE) goto reject;
    if (data[0] + 1 < data[1]) {
        size_t i; for (i = 2; i < len; ++i) if (data[i] == 0) goto reject;
    }
    memcpy(rx->raw + rx->used, data + 2, len - 2);
    rx->used += len - 2; ++rx->next; rx->last = now;
    if (rx->next != rx->total) return true;
    /* Keep the currently displayed message immutable; retain the newest four queued messages. */
    slot = (uint8_t)((rx->head + rx->count) % GLASSES_QUEUE_SIZE);
    {
        char decoded[GLASSES_TEXT_SIZE] = {0};
        if (!text_decode(rx->raw, rx->used, decoded, &truncated)) goto reject;
        if (rx->count == GLASSES_QUEUE_SIZE) {
            rx->head = (uint8_t)((rx->head + 1) % GLASSES_QUEUE_SIZE); --rx->count; ++rx->dropped;
        }
        memcpy(rx->queue[slot], decoded, strlen(decoded) + 1);
    }
    if (truncated) ++rx->truncated;
    ++rx->count; glasses_rx_reset(rx); return true;
reject:
    ++rx->rejected; glasses_rx_reset(rx); return false;
}
bool glasses_rx_pop(GlassesRx *rx, char out[GLASSES_TEXT_SIZE]) {
    if (!rx->count) return false;
    memcpy(out, rx->queue[rx->head], GLASSES_TEXT_SIZE);
    rx->head = (uint8_t)((rx->head + 1) % GLASSES_QUEUE_SIZE); --rx->count; return true;
}
static unsigned days(unsigned month, unsigned year) {
    static const uint8_t d[] = {31,28,31,30,31,30,31,31,30,31,30,31};
    return d[month - 1] + (month == 2 && (!year || (year % 4 == 0 && (year % 100 || year % 400 == 0))));
}
static unsigned decimal(const uint8_t *p, unsigned n) {
    unsigned value = 0; while (n--) value = value * 10 + *p++ - '0'; return value;
}
bool glasses_clock_set(GlassesClock *c, const uint8_t *p, size_t len, uint32_t now) {
    GlassesClock next = {0}; size_t i;
    if (!p || (len != 8 && len != 12)) return false;
    for (i = 0; i < len; ++i) if (p[i] < '0' || p[i] > '9') return false;
    next.hour = decimal(p, 2); next.minute = decimal(p + 2, 2);
    next.day = decimal(p + 4, 2); next.month = decimal(p + 6, 2);
    next.year = len == 12 ? decimal(p + 8, 4) : 0;
    if (next.hour > 23 || next.minute > 59 || next.month < 1 || next.month > 12 ||
        (len == 12 && (next.year < 2000 || next.year > 2099)) || next.day < 1 || next.day > days(next.month, next.year)) return false;
    next.valid = true; next.last = now; *c = next; return true;
}
void glasses_clock_tick(GlassesClock *c, uint32_t now) {
    uint32_t seconds, elapsed;
    if (!c->valid) return;
    elapsed = now - c->last; c->last = now;
    seconds = elapsed / 1000; c->remainder += elapsed % 1000;
    seconds += c->remainder / 1000; c->remainder %= 1000;
    seconds += c->second + 60 * c->minute + 3600 * c->hour;
    c->second = seconds % 60; c->minute = seconds / 60 % 60; c->hour = seconds / 3600 % 24;
    seconds /= 86400;
    while (seconds--) {
        if (++c->day > days(c->month, c->year)) { c->day = 1; if (++c->month > 12) { c->month = 1; if (c->year) ++c->year; } }
    }
}
GlassesTouchAction glasses_touch_poll(GlassesTouch *t, uint8_t pads, uint32_t now) {
    unsigned i; static const uint32_t hold[] = {1000,2240,1000};
    pads &= 7;
    if (pads != t->candidate) { t->candidate = pads; t->changed = now; }
    if (pads != t->stable && (uint32_t)(now - t->changed) >= 60) {
        for (i = 0; i < 3; ++i) if ((pads & (1u << i)) && !(t->stable & (1u << i))) t->pressed[i] = now;
        t->fired &= pads; t->stable = pads;
    }
    if ((t->stable & 5) == 5) {
        if (!t->chord_active) { t->chord_active = true; t->chord = now; }
        if (!t->chord_fired && (uint32_t)(now - t->chord) >= 3000) { t->chord_fired = true; t->fired |= 5; return GLASSES_PAIR; }
        return GLASSES_TOUCH_NONE;
    }
    t->chord_active = false; t->chord_fired = false;
    for (i = 0; i < 3; ++i) if ((t->stable & (1u << i)) && !(t->fired & (1u << i)) &&
        (uint32_t)(now - t->pressed[i]) >= hold[i]) {
        t->fired |= (uint8_t)(1u << i); return (GlassesTouchAction)(GLASSES_HOME + i);
    }
    return GLASSES_TOUCH_NONE;
}
uint32_t glasses_crc32(uint32_t crc, const uint8_t *p, size_t len) {
    unsigned i; while (len--) { crc ^= *p++; for (i = 0; i < 8; ++i) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u))); } return crc;
}
bool glasses_image_vectors_valid(uint32_t sp, uint32_t reset, uint32_t size) {
    /* CPU1 application uses SRAM1 only; reserve the first eight bytes. */
    return size >= 0x140 && size <= 0x30000 && !(sp & 7) && sp > 0x20000008 && sp <= 0x20008000 &&
           (reset & 1) && (reset & ~1u) >= 0x08010000 && (reset & ~1u) < 0x08010000 + size;
}
