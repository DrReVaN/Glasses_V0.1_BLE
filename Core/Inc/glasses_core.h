#ifndef GLASSES_CORE_H
#define GLASSES_CORE_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define GLASSES_TEXT_SIZE 128u
#define GLASSES_RX_SIZE 252u
#define GLASSES_QUEUE_SIZE 4u
#define GLASSES_RX_TIMEOUT_MS 3000u
/* Internal display text: one byte per cell, ASCII + printable Latin-1.
 * 0x80 is euro, 0x81 is a visible missing-glyph box. BLE remains UTF-8. */
#define GLASSES_GLYPH_EURO 0x80u
#define GLASSES_GLYPH_MISSING 0x81u
typedef struct {
    uint8_t raw[GLASSES_RX_SIZE];
    size_t used;
    uint8_t next, total;
    uint32_t last;
    char queue[GLASSES_QUEUE_SIZE][GLASSES_TEXT_SIZE];
    uint8_t head, count;
    uint32_t rejected, dropped, truncated;
} GlassesRx;
typedef struct {
    unsigned year, month, day, hour, minute, second;
    uint32_t last, remainder;
    bool valid;
} GlassesClock;
typedef enum { GLASSES_TOUCH_NONE, GLASSES_HOME, GLASSES_POWER,
               GLASSES_MESSAGE, GLASSES_PAIR } GlassesTouchAction;
typedef struct {
    uint8_t candidate, stable, fired;
    uint32_t changed, pressed[3], chord;
    bool chord_active, chord_fired;
} GlassesTouch;
void glasses_rx_reset(GlassesRx *rx);
bool glasses_rx_push(GlassesRx *rx, const uint8_t *data, size_t len, uint32_t now);
void glasses_rx_expire(GlassesRx *rx, uint32_t now);
bool glasses_rx_pop(GlassesRx *rx, char out[GLASSES_TEXT_SIZE]);
bool glasses_clock_set(GlassesClock *clock, const uint8_t *data, size_t len, uint32_t now);
void glasses_clock_tick(GlassesClock *clock, uint32_t now);
GlassesTouchAction glasses_touch_poll(GlassesTouch *touch, uint8_t pads, uint32_t now);
uint32_t glasses_crc32(uint32_t crc, const uint8_t *data, size_t len);
bool glasses_image_vectors_valid(uint32_t sp, uint32_t reset, uint32_t size);
#endif
