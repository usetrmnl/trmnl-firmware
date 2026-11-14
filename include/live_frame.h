#pragma once

#include <stddef.h>
#include <stdint.h>

// Draws a live mode frame on the TRMNL OG, with the fast waveform when the last frame is still on screen.
// False when it can't (a gray image, another panel, no memory); draw it the usual way.
bool live_frame_draw(uint8_t *png, size_t size);
