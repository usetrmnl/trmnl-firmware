#pragma once

#include <stddef.h>
#include <stdint.h>

// Whether the TRMNL OG can draw this PNG with its fast live waveform: 1-bit gray or indexed, not interlaced, the size
// of the screen. Gray images are drawn the usual way.
bool liveFramePngDrawable(const uint8_t *png, size_t size, uint32_t screenWidth, uint32_t screenHeight);

// Copies a decoded 1-bit PNG row as the panel takes it (1 = white), flipped when the palette's first color is the
// brighter one. palette holds RGB triplets for indexed rows, nullptr for gray ones.
void liveFrameRow(const uint8_t *pixels, uint32_t width, const uint8_t *palette, uint8_t *out);

// Whether a drawable PNG of the changed area fits on the screen at x, y, with x and its width on byte boundaries.
// Its width and height go to width and height.
bool liveFramePatchFits(const uint8_t *png, size_t size, uint32_t x, uint32_t y, uint32_t screenWidth,
                        uint32_t screenHeight, uint32_t *width, uint32_t *height);
