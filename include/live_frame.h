#pragma once

#include <stddef.h>
#include <stdint.h>

// Draws a 1-bit PNG the size of the screen with live mode's partial waveform (lib/trmnl live_region.h): only the
// pixels that change move, so a live session needs a full refresh only every LIVE_REGION_MAX_PARTIALS frames.
// False when the image isn't such a PNG; the caller then draws it the usual way.
bool live_frame_draw(uint8_t *png, size_t size);
