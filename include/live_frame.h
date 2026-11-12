#pragma once

#include <stddef.h>
#include <stdint.h>

// Draws a live mode frame on the TRMNL OG with the fast balanced waveform, keeping a copy of it, named name, for the
// next one. False when it can't (a gray image, another panel, a full refresh due, or the first frame); draw it the
// usual way.
bool live_frame_draw(uint8_t *png, size_t size, const char *name);

// Whether the screen shows the live frame named name, so a patch made against it can be drawn.
bool live_frame_shows(const char *name);

// Draws a PNG of the changed area at x, y over the frame on screen, which then takes the name name. False when it
// can't; the screen is unchanged and the full image must be drawn instead.
bool live_frame_draw_patch(uint8_t *png, size_t size, uint32_t x, uint32_t y, const char *name);
