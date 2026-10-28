#pragma once

#include <Arduino.h>

// Decode a JPEG straight into the panel driver's buffer/RAM; return the refresh mode to use, or an error
int jpeg_to_epd(const uint8_t *pJPEG, int iDataSize);
