#include "live_region.h"

#include <string.h>

namespace {
  const uint8_t PNG_SIGNATURE[8] = {0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A};
  constexpr int32_t MAX_COORDINATE = 9999;

  uint32_t bigEndian32(const uint8_t *p) {
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
  }

  uint32_t brightness(const uint8_t *rgb) { return rgb[0] + 2u * rgb[1] + rgb[2]; }

  void setPixel(uint8_t *row, uint32_t x, bool white) {
    uint8_t mask = 0x80 >> (x & 7);
    if (white)
      row[x / 8] |= mask;
    else
      row[x / 8] &= ~mask;
  }
} // namespace

bool readPngHeader(const uint8_t *data, size_t size, PngHeader &header) {
  // 8 signature bytes, then the IHDR chunk: length, type, 13 bytes of data
  if (size < 29 || memcmp(data, PNG_SIGNATURE, 8) != 0 || memcmp(&data[12], "IHDR", 4) != 0) return false;
  header.width = bigEndian32(&data[16]);
  header.height = bigEndian32(&data[20]);
  header.bitDepth = data[24];
  header.colorType = data[25];
  header.interlace = data[28];
  return true;
}

bool liveRegionPngSupported(const PngHeader &header) {
  bool grayOrIndexed = header.colorType == 0 || header.colorType == 3;
  bool oneOrTwoBit = header.bitDepth == 1 || header.bitDepth == 2;
  return grayOrIndexed && oneOrTwoBit && header.interlace == 0 && header.width > 0 && header.height > 0;
}

bool liveRegionFits(int32_t x, int32_t y, uint32_t width, uint32_t height, uint32_t screenWidth,
                    uint32_t screenHeight) {
  if (x < 0 || y < 0 || width == 0 || height == 0) return false;
  return width <= screenWidth && height <= screenHeight && (uint32_t)x <= screenWidth - width &&
         (uint32_t)y <= screenHeight - height;
}

bool parseLiveRegionCoordinate(const char *text, int32_t &value) {
  if (!text || !*text) return false;
  int32_t parsed = 0;
  for (const char *c = text; *c; c++) {
    if (*c < '0' || *c > '9') return false;
    parsed = parsed * 10 + (*c - '0');
    if (parsed > MAX_COORDINATE) return false;
  }
  value = parsed;
  return true;
}

bool liveRegionRowTo1Bit(const uint8_t *pixels, uint32_t width, uint8_t bitDepth, const uint8_t *palette,
                         uint8_t *out) {
  uint32_t bytes = (width + 7) / 8;
  if (bitDepth == 1) {
    bool flip = palette && brightness(&palette[0]) > brightness(&palette[3]);
    for (uint32_t i = 0; i < bytes; i++)
      out[i] = flip ? ~pixels[i] : pixels[i];
    return true;
  }
  if (bitDepth == 2) {
    memset(out, 0, bytes);
    for (uint32_t x = 0; x < width; x++) {
      uint8_t sample = (pixels[x / 4] >> (6 - 2 * (x & 3))) & 3;
      bool white;
      if (palette) {
        const uint8_t *rgb = &palette[sample * 3];
        white = rgb[0] == 0xff && rgb[1] == 0xff && rgb[2] == 0xff;
      } else {
        white = sample == 3;
      }
      setPixel(out, x, white);
    }
    return true;
  }
  return false;
}

void liveRegionComposeRow(uint8_t *frame, uint32_t frameWidth, uint32_t x, uint32_t y, const uint8_t *row,
                          uint32_t width) {
  uint8_t *line = frame + y * ((frameWidth + 7) / 8);
  if ((x & 7) == 0 && (width & 7) == 0) { // byte aligned: copy whole bytes
    memcpy(line + x / 8, row, width / 8);
    return;
  }
  for (uint32_t i = 0; i < width; i++) {
    setPixel(line, x + i, row[i / 8] & (0x80 >> (i & 7)));
  }
}

LiveRegionRefresh liveRegionRefresh(uint32_t partialsSinceFull, bool fullRequested, bool maximumCompatibility) {
  if (fullRequested || maximumCompatibility || partialsSinceFull >= LIVE_REGION_MAX_PARTIALS)
    return LiveRegionRefresh::Full;
  return LiveRegionRefresh::Partial;
}

uint8_t liveRegionDitherMask(uint32_t row, uint32_t every, uint32_t phase) {
  static const uint8_t bayer[4][4] = {{0, 8, 2, 10}, {12, 4, 14, 6}, {3, 11, 1, 9}, {15, 7, 13, 5}};
  if (every != 2 && every != 4 && every != 8 && every != 16) return 0;
  uint8_t mask = 0;
  for (uint32_t x = 0; x < 8; x++) {
    if (bayer[row & 3][x & 3] % every == phase % every) mask |= 0x80 >> x;
  }
  return mask;
}

