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
    bounds(6, message ? 47 : 34);
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
        memset(message, ch, 6); message[6] = 0;
        ssd1306_Fill(Black);
        legacy_text(6, 61, message, Font_7x10);
        ssd1306_UpdateScreen();
        memcpy(reference, frame, sizeof(frame));
        render(&view, true);
        assert(memcmp(frame, reference, sizeof(frame)) == 0);
    }
    memset(message, 'M', sizeof(message) - 1); message[sizeof(message) - 1] = 0;
    for (view.scroll = 0; view.scroll < sizeof(message) - 1; ++view.scroll)
        render(&view, true);
    /* All extended glyphs really reach SPI; signed char must never index ASCII. */
    assert(ssd1306_Glyph7x10(0x80) != 0 && ssd1306_Glyph7x10(0x81) != 0);
    assert(memcmp(ssd1306_Glyph7x10(0x80), ssd1306_Glyph7x10('?'), 20) != 0);
    assert(ssd1306_Glyph7x10(0x82) == 0 && ssd1306_Glyph7x10(0xA0) == 0);
    view.scroll = 0;
    for (unsigned ch = 0x80; ch <= 0xFF; ++ch) {
        if (ch != 0x80 && ch != 0x81 && ch < 0xA1) continue;
        const uint16_t *glyph = ssd1306_Glyph7x10((uint8_t)ch);
        assert(glyph);
        memset(message,ch,6); message[6]=0;
        render(&view,true); memcpy(reference,frame,sizeof(frame));
        ssd1306_Fill(Black);
        /* Independent placement of six glyphs, including accents at row 9. */
        for (unsigned col=0;col<6;++col)
            for (unsigned dy=0;dy<10;++dy) for (unsigned dx=0;dx<7;++dx)
                if (glyph[dy] & (0x8000u >> dx))
                    ssd1306_DrawPixel(6+col*7+dx,61+dy,White);
        ssd1306_UpdateScreen(); assert(!memcmp(frame,reference,sizeof(frame)));
    }
    /* Exercise the wire decoder and renderer together, across a UTF-8 split. */
    GlassesRx rx = {0};
    const uint8_t sample[] = "12345678901234567\xE2\x82\xAC" "12,50\xC3\x84\xC3\x96\xC3\x9C";
    for (size_t i=0;i<sizeof(sample)-1;i+=18) {
        uint8_t packet[20]={(uint8_t)(i/18),(uint8_t)((sizeof(sample)-2)/18+1)};
        size_t n=sizeof(sample)-1-i; if(n>18)n=18;
        memcpy(packet+2,sample+i,n); assert(glasses_rx_push(&rx,packet,n+2,(uint32_t)i));
    }
    assert(glasses_rx_pop(&rx,message));
    assert(strlen(message)==26 && (uint8_t)message[17]==GLASSES_GLYPH_EURO);
    for (view.scroll=0;view.scroll<strlen(message);++view.scroll) render(&view,true);
    view.scroll=17; render(&view,true);
    FILE *preview=fopen("build/tests/display-unicode.pbm","wb");
    assert(preview); fprintf(preview,"P4\n64 128\n");
    for(unsigned y=0;y<128;++y) for(unsigned bx=0;bx<8;++bx) {
        uint8_t bits=0;
        for(unsigned bit=0;bit<8;++bit) {
            unsigned x=bx*8+bit;
            if(frame[y+(x/8)*SSD1306_WIDTH] & (1u<<(x%8))) bits|=(uint8_t)(0x80u>>bit);
        }
        assert(fputc(bits,preview)!=EOF);
    }
    assert(fclose(preview)==0);
    view.scroll=SIZE_MAX; glasses_display_render(&view);
    for(size_t i=0;i<sizeof(frame);++i) assert(frame[i]==0);
    view.message = "";
    render(&view, false);
    puts("Display: clock/BLE ASCII unchanged; six message cells align at X=6; all 97 new glyphs remain inside the optical footprint.");
    return 0;
}
