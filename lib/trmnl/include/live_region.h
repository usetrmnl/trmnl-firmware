#pragma once

#include <stddef.h>
#include <stdint.h>

// Region pushes in a live mode session: the server sends a 1-bit PNG for part of the screen
// (POST /region?x=…&y=…), the device composes it onto its copy of the 1-bit frame on screen and
// redraws the panel with a partial refresh, so only the pixels that changed move. 1-bit frames
// here are packed 8 pixels a byte, most significant bit first, 1 = white, rows (width + 7) / 8 bytes.

// A full refresh, against ghosting, after this many partial ones in a row: a safety net for a server that never cleans
// up. With the OG's 200 Hz regions and gentle cleanups the user saw no ghosting after 20; how far it goes is being tested.
#ifndef LIVE_REGION_MAX_PARTIALS
#define LIVE_REGION_MAX_PARTIALS 500
#endif

// The OG's partial waveform for regions: frames (twice) that drive changing pixels, frames that hold the others
#define LIVE_REGION_DRIVE_FRAMES 100  // at 200 Hz: 0.5 s twice; 80 left blacks a little gray in places
#define LIVE_REGION_KEEP_FRAMES  1    // at 200 Hz the shortest hold: enough against darkening, no visible flash
#define LIVE_REGION_VCOM         0x1E // VDCS: -1.6 V, the OG panel's factory VCOM (-0.1 V fades black)
#define LIVE_REGION_PLL          0x0F // about 200 Hz (0x06 = 50 Hz, the panel's default)
#define LIVE_REGION_TCON         0x00 // no gate/source non-overlap: rows are only about 10 us at 200 Hz

// What a region's PNG says about itself
struct PngHeader {
  uint32_t width;
  uint32_t height;
  uint8_t bitDepth;
  uint8_t colorType; // 0 gray, 3 indexed, …
  uint8_t interlace;
};

// Reads the IHDR of a PNG; false unless it's there
bool readPngHeader(const uint8_t *data, size_t size, PngHeader &header);

// Whether a region can draw a PNG with this header: 1 or 2-bit gray or indexed, not interlaced.
// 2-bit images must hold only black and white to look the same as on the server.
bool liveRegionPngSupported(const PngHeader &header);

// Whether a width × height region at (x, y) lies on a screen of screenWidth × screenHeight
bool liveRegionFits(int32_t x, int32_t y, uint32_t width, uint32_t height, uint32_t screenWidth,
                    uint32_t screenHeight);

// Reads a query parameter as a coordinate: digits only, below 10000. False, leaving value alone, otherwise.
bool parseLiveRegionCoordinate(const char *text, int32_t &value);

// Converts one decoded PNG row to 1-bit (1 = white). 1-bit rows keep their pixels, flipped when the palette's first
// color is the brighter one; 2-bit rows make anything but white black, as the panel does when it draws them.
// palette holds RGB triplets for indexed rows, nullptr otherwise. False for other bit depths.
bool liveRegionRowTo1Bit(const uint8_t *pixels, uint32_t width, uint8_t bitDepth, const uint8_t *palette,
                         uint8_t *out);

// Copies a 1-bit row of width pixels into a 1-bit frame frameWidth pixels wide, at (x, y), any x
void liveRegionComposeRow(uint8_t *frame, uint32_t frameWidth, uint32_t x, uint32_t y, const uint8_t *row,
                          uint32_t width);

// How a region is refreshed
enum class LiveRegionRefresh {
  Partial,
  Full,
};

// Partial, unless the server asks for a full one, the device's settings want only full refreshes, or there have
// been LIVE_REGION_MAX_PARTIALS partial ones since the last full refresh
LiveRegionRefresh liveRegionRefresh(uint32_t partialsSinceFull, bool fullRequested, bool maximumCompatibility);

// Dithered hold: on each region, a sparse rotating subset of the pixels a region doesn't change, 1 in `every` (2, 4, 8
// or 16) in a 4×4 ordered-dither pattern, gets re-driven to its own color, so no refresh brightens the whole panel at
// once but every pixel is held every `every` regions. The subset for `phase` in `row`, as a byte to XOR into each
// byte of that row's old image (most significant bit = the leftmost pixel): 0 when `every` isn't one of those.
uint8_t liveRegionDitherMask(uint32_t row, uint32_t every, uint32_t phase);

