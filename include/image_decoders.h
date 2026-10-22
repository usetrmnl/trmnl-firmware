#pragma once

#include <stdint.h>

// Decode an image into the panel's buffer; each returns the refresh mode to draw it with
int png_to_epd(const uint8_t *pPNG, int iDataSize, bool bPrevious);
int jpeg_to_epd(const uint8_t *pJPEG, int iDataSize);
