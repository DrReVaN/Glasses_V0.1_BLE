/* Exercise the production renderer, SSD1306 driver and fonts. Capture the actual
 * SPI framebuffer; the legacy reference coordinates come from main:main.c. */
#include "glasses_display.h"
#include "ssd1306.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

SPI_HandleTypeDef hspi1;
static uint8_t frame[SSD1306_BUFFER_SIZE], reference[SSD1306_BUFFER_SIZE], page;
static bool data_mode;
static unsigned writes;
void HAL_GPIO_WritePin(void *port, uint16_t pin, GPIO_PinState state) {
    assert(port == GPIOA);
    if (pin == GPIO_PIN_3) data_mode = state == GPIO_PIN_SET;
}
void HAL_Delay(uint32_t delay) { (void)delay; }
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *spi, uint8_t *data, uint16_t size, uint32_t timeout) {
    assert(spi == &hspi1 && timeout > 0);
    if (data_mode) {
        assert(size == SSD1306_WIDTH && page < SSD1306_HEIGHT / 8);
        memcpy(frame + page * SSD1306_WIDTH, data, size);
        writes++;
    } else {
        assert(size == 1);
        if ((*data & 0xF8) == 0xB0) page = *data & 7;
    }
    return HAL_OK;
}
static void bounds(unsigned min_x, unsigned max_x) {
    unsigned lit = 0;
    for (unsigned x = 0; x < SSD1306_HEIGHT; ++x)
        for (unsigned y = 0; y < SSD1306_WIDTH; ++y)
            if (frame[y + (x / 8) * SSD1306_WIDTH] & (1u << (x % 8))) {
                assert(x >= min_x && x <= max_x && y >= 56 && y <= 75);
                lit++;
            }
    assert(lit > 0);
}
static void render(GlassesDisplay *view, bool message) {
    /* A previous large frame must be cleared, including outside the viewport. */
    ssd1306_Fill(White);
    writes = 0;
    glasses_display_render(view);
    assert(writes == SSD1306_HEIGHT / 8);
    bounds(message ? 0 : 6, message ? 48 : 34);
}
static void legacy_text(uint8_t x, uint8_t y, char *str, FontDef font) {
    ssd1306_SetCursor(x, y);
    assert(ssd1306_WriteString(str, font, White) == 0);
}
static void legacy_clock(const GlassesClock *clock) {
    char digits[16];
    ssd1306_Fill(Black);
    snprintf(digits, sizeof(digits), "%02u", clock->hour);
    legacy_text(6, 58, digits, Font_6x8);
    legacy_text(17, 58, ":", Font_6x8);
    snprintf(digits, sizeof(digits), "%02u", clock->minute);
    legacy_text(21, 58, digits, Font_6x8);
    snprintf(digits, sizeof(digits), "%02u", clock->day);
    legacy_text(6, 66, digits, Font_6x8);
    legacy_text(17, 66, ".", Font_6x8);
    snprintf(digits, sizeof(digits), "%02u", clock->month);
    legacy_text(21, 66, digits, Font_6x8);
    ssd1306_UpdateScreen();
    memcpy(reference, frame, sizeof(frame));
}
int main(void) {
    GlassesClock clock = {.valid = true, .day = 30, .month = 9};
    GlassesDisplay view = {.clock = &clock, .message = "", .connected = true};
    ssd1306_Init();
    for (unsigned minute = 0; minute < 24 * 60; ++minute) {
        clock.hour = minute / 60; clock.minute = minute % 60;
        legacy_clock(&clock);
        render(&view, false);
        assert(memcmp(frame, reference, sizeof(frame)) == 0);
    }
    for (clock.month = 1; clock.month <= 12; ++clock.month)
        for (clock.day = 1; clock.day <= 31; ++clock.day) {
            legacy_clock(&clock);
            render(&view, false);
            assert(memcmp(frame, reference, sizeof(frame)) == 0);
        }
    clock.valid = false;
    render(&view, false);
    view.connected = false;
    ssd1306_Fill(Black);
    legacy_text(7, 66, "Wait", Font_7x10);
    legacy_text(10, 56, "BLE", Font_7x10);
    ssd1306_UpdateScreen();
    memcpy(reference, frame, sizeof(frame));
    render(&view, false);
    assert(memcmp(frame, reference, sizeof(frame)) == 0);
    view.bootloader = true;
    render(&view, false);
    view.connected = true;
    render(&view, false);
    view.ota_requested = true;
    render(&view, false);
    view.pairing = true; /* Pairing takes priority over OTA. */
    for (unsigned value = 0; value < 1000; ++value) {
        /* Each possible three-digit group appears in both rows, including 000. */
        view.pair_value = value * 1000 + (999 - value);
        render(&view, false);
    }
    view.pair_value = 123456;
    render(&view, false);
    memcpy(reference, frame, sizeof(frame));
    ssd1306_Fill(Black);
    legacy_text(10, 56, "123", Font_7x10);
    legacy_text(10, 66, "456", Font_7x10);
    ssd1306_UpdateScreen();
    assert(memcmp(frame, reference, sizeof(frame)) == 0);
    view.pairing = view.ota_requested = view.bootloader = false;
    char message[GLASSES_TEXT_SIZE];
    view.message = message;
    for (unsigned ch = 33; ch <= 126; ++ch) {
        memset(message, ch, 5); message[5] = 0;
        ssd1306_Fill(Black);
        legacy_text(14, 61, message, Font_7x10);
        ssd1306_UpdateScreen();
        memcpy(reference, frame, sizeof(frame));
        render(&view, true);
        assert(memcmp(frame, reference, sizeof(frame)) == 0);
    }
    memset(message, 'M', sizeof(message) - 1); message[sizeof(message) - 1] = 0;
    for (view.scroll = 0; view.scroll < sizeof(message) - 1; ++view.scroll)
        render(&view, true);
    view.message = "";
    render(&view, false);
    puts("Display: legacy clock, BLE and message frames match pixel-for-pixel; all UI states stay within the legacy footprint.");
    return 0;
}
