#pragma once

#include <stddef.h>
#include <stdint.h>

// Draws a live mode frame on the TRMNL OG with the fast balanced waveform, keeping a copy of it for the next one.
// False when it can't (a gray image, another panel, a full refresh due, or the first frame); draw it the usual way.
bool live_frame_draw(uint8_t *png, size_t size);
