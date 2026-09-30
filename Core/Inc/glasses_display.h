#ifndef GLASSES_DISPLAY_H
#define GLASSES_DISPLAY_H
#include "glasses_core.h"

typedef struct {
    const GlassesClock *clock;
    const char *message;
    size_t scroll;
    uint32_t pair_value;
    bool connected, bootloader, pairing, ota_requested;
} GlassesDisplay;

/* V1 optical layout: preserve the coordinates and fonts of the working firmware.
 * Logical OLED RAM is 64 x 128; it is not the prism's usable image area. */
void glasses_display_render(const GlassesDisplay *view);
#endif
