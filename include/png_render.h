#pragma once

#include <Arduino.h>

// Decode a PNG straight into the panel driver's buffer/RAM; return the refresh mode to use, or an error
int png_to_epd(const uint8_t *pPNG, int iDataSize, bool bPrevious);
