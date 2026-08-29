#pragma once
#include <stdint.h>
#include <stdbool.h>
//
// The display is controlled by 8 unique EPD controllers in a 4 x 2 arrangement
// Instead of 2560x1440 divided evenly into 8 sections it is arranged as
// 0:800x720 1:800x720 2:800x720 3:160x720 (top half of display)
// 4:800x720 5:800x720 6:800x720 7:160x720 (bottom half of display)
//
// Download image (PNG or BMP auto-detected) and decode into the 8 section buffers.
// Allocates PSRAM buffers internally; call gen2_imgFree() after epdDisplay().
// Returns true on success; buffers filled with white on failure.
bool gen2_imgFetchUrl(const char *url);

// Decode an already-downloaded PNG or BMP from an in-memory buffer.
// Useful when the caller has already fetched the data (e.g. from the main firmware).
bool gen2_display_image(const uint8_t *buf, uint32_t len);
