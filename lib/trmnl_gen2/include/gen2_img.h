#pragma once
#include <stdint.h>
#include <stdbool.h>

// One section buffer = 800 × 720 pixels @ 4 bpp (matches EPD_IMAGE_DATA_BUFFER)
#define GEN2_IMG_SECTIONS       8
#define GEN2_IMG_SECTION_W      800
#define GEN2_IMG_SECTION_H      720
#define GEN2_IMG_SECTION_BYTES  288000

// Total visible display width distributed across all 8 driver ICs.
// ICs 0-2: 800 active cols each (vis 0-2399)
// IC  3:   160 active + 640 dummy (vis 2400-2559)
// ICs 4-6: 800 active cols each (vis 2560-4959)
// IC  7:   160 active + 640 dummy (vis 4960-5119)
#define GEN2_DISPLAY_VISIBLE_W  5120

// Pointers to the 8 section buffers filled by gen2_imgFetchUrl().
// Pass gen2_imgSections[i] to epdWriteImage(i, ..., GEN2_IMG_SECTION_BYTES).
extern const uint8_t *gen2_imgSections[GEN2_IMG_SECTIONS];

// Download image (PNG or BMP auto-detected) and decode into the 8 section buffers.
// Allocates PSRAM buffers internally; call gen2_imgFree() after epdDisplay().
// Returns true on success; buffers filled with white on failure.
bool gen2_imgFetchUrl(const char *url);

// Decode an already-downloaded PNG or BMP from an in-memory buffer.
// Useful when the caller has already fetched the data (e.g. from the main firmware).
bool gen2_imgDecodeBuffer(const uint8_t *buf, uint32_t len);

// Free all 8 PSRAM section buffers.
void gen2_imgFree(void);

// Fill every pixel with a solid EPD color nibble (e.g. EPD_WHITE = 0x11).
bool gen2_imgSolid(uint8_t nib);

// Write one display pixel: dc = display column (0..5119), dr = row (0..719).
void gen2_imgWritePixel(uint32_t dc, uint32_t dr, uint8_t nib);

// Overlay ASCII text using an embedded 8×8 bitmap font.
// x, y: top-left in display coords.  scale: pixel multiplier (4 → 32×32 px/char).
void gen2_imgDrawText(uint32_t x, uint32_t y, const char *text,
                      uint8_t fg_nib, uint8_t bg_nib, uint8_t scale);
