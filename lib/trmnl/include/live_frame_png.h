#pragma once

#include <stddef.h>
#include <stdint.h>

// Whether the TRMNL OG can draw this PNG with its fast live waveform: 1- or 2-bit gray or indexed, not interlaced,
// the size of the screen.
bool liveFramePngDrawable(const uint8_t *png, size_t size, uint32_t screenWidth, uint32_t screenHeight);

// Copies a decoded 1- or 2-bit PNG row as 2-bit levels (0 = black .. 3 = white, 4 pixels a byte, MSB first). palette
// holds RGB triplets for indexed rows, nullptr for gray ones. True when the row has a gray level.
bool liveFrameRow(const uint8_t *pixels, uint32_t width, uint8_t bitDepth, const uint8_t *palette, uint8_t *out);

// One row of the panel's old (plane 1) or new (plane 2) image for pass 1 or 2 of the live draw, 1 = white. The pair
// picks each pixel's waveform: a gray frame takes both passes (to the nearer extreme, then to its gray).
void liveFramePlaneRow(const uint8_t *was, const uint8_t *now, uint32_t width, int pass, int plane, uint8_t *out);

// Packs a row of 2-bit levels as runs, a byte per run of up to 64 pixels (level in the top 2 bits). Returns the bytes
// written, 0 when they don't fit in capacity.
size_t liveFrameRowPack(const uint8_t *levels, uint32_t width, uint8_t *out, size_t capacity);

// Unpacks one packed row into 2-bit levels, returning the bytes it read.
size_t liveFrameRowUnpack(const uint8_t *in, uint32_t width, uint8_t *levels);
