#include "glasses_display.h"
#include "ssd1306.h"
#include <string.h>

static void text(uint8_t x, uint8_t y, char *value, FontDef font) {
    ssd1306_SetCursor(x, y);
    ssd1306_WriteString(value, font, White);
}
static void number(uint8_t x, uint8_t y, unsigned value) {
    char digits[3] = {(char)('0' + value / 10), (char)('0' + value % 10), 0};
    text(x, y, digits, Font_6x8);
}
static void clock_row(uint8_t y, unsigned first, unsigned second, char separator, bool valid) {
    if (valid) number(6, y, first);
    else text(6, y, "--", Font_6x8);
    ssd1306_SetCursor(17, y);
    ssd1306_WriteChar(separator, Font_6x8, White);
    if (valid) number(21, y, second);
    else text(21, y, "--", Font_6x8);
}
static void triplet(uint8_t y, uint32_t value) {
    char digits[4] = {(char)('0' + value / 100),
                      (char)('0' + value / 10 % 10),
                      (char)('0' + value % 10), 0};
    text(10, y, digits, Font_7x10);
}
void glasses_display_render(const GlassesDisplay *view) {
    ssd1306_Fill(Black);
    if (view->pairing) {
        /* Read the first three digits on top, then the last three below.
         * Six digits on one line exceed the legacy status/clock footprint. */
        triplet(56, view->pair_value / 1000);
        triplet(66, view->pair_value % 1000);
    } else if (view->ota_requested) {
        text(7, 56, "OTA?", Font_7x10);
        text(7, 66, "1+3-", Font_7x10);
    } else if (view->message && view->message[0]) {
        char window[GLASSES_MESSAGE_COLUMNS + 1] = {0};
        size_t length = strlen(view->message);
        for (size_t i = 0; i < GLASSES_MESSAGE_COLUMNS && view->scroll < length && i < length - view->scroll; ++i)
            window[i] = view->message[view->scroll + i];
        /* Align with the clock's X=6. Six 7x10 cells end at X=47,
         * inside the previous five-cell message boundary at X=48. */
        text(6, 61, window, Font_7x10);
    } else if (view->bootloader) {
        text(10, 56, "OTA", Font_7x10);
        text(7, 66, view->connected ? "Link" : "Pair", Font_7x10);
    } else if (!view->connected) {
        text(10, 56, "BLE", Font_7x10);
        text(7, 66, "Wait", Font_7x10);
    } else {
        /* Keep even the original overlapping separator cells and draw order. */
        clock_row(58, view->clock->hour, view->clock->minute, ':', view->clock->valid);
        clock_row(66, view->clock->day, view->clock->month, '.', view->clock->valid);
    }
    ssd1306_UpdateScreen();
}
