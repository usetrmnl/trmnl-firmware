#include "live_frame_png.h"

#include <string.h>

namespace {
  const uint8_t PNG_SIGNATURE[8] = {0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A};

  uint32_t bigEndian32(const uint8_t *p) {
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
  }

  uint32_t brightness(const uint8_t *rgb) { return rgb[0] + 2u * rgb[1] + rgb[2]; }
} // namespace

bool liveFramePngDrawable(const uint8_t *png, size_t size, uint32_t screenWidth, uint32_t screenHeight) {
  // 8 signature bytes, then the IHDR chunk: length, type, 13 bytes of data
  if (size < 29 || memcmp(png, PNG_SIGNATURE, 8) != 0 || memcmp(&png[12], "IHDR", 4) != 0) return false;
  const uint8_t bitDepth = png[24], colorType = png[25], interlace = png[28];
  return bigEndian32(&png[16]) == screenWidth && bigEndian32(&png[20]) == screenHeight && bitDepth == 1 &&
         (colorType == 0 || colorType == 3) && interlace == 0;
}

void liveFrameRow(const uint8_t *pixels, uint32_t width, const uint8_t *palette, uint8_t *out) {
  const bool flip = palette && brightness(&palette[0]) > brightness(&palette[3]);
  for (uint32_t i = 0; i < (width + 7) / 8; i++)
    out[i] = flip ? ~pixels[i] : pixels[i];
}
