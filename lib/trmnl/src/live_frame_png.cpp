#include "live_frame_png.h"

#include <string.h>

namespace {
  const uint8_t PNG_SIGNATURE[8] = {0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A};
  enum Lut : uint8_t { BB = 0, BW = 1, WB = 2, WW = 3 }; // (plane 1 bit, plane 2 bit), as in the panel's LUT names

  uint32_t bigEndian32(const uint8_t *p) {
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
  }

  uint8_t level(const uint8_t *row, uint32_t x) { return (row[x / 4] >> (6 - 2 * (x % 4))) & 3; }

  uint8_t paletteLevel(const uint8_t *rgb) { return (3u * (rgb[0] + 2u * rgb[1] + rgb[2]) + 510) / 1020; }

  // Pass 1 takes a changed pixel to the extreme its new level is nearer; pass 2 pushes it from there to its gray.
  // Unchanged white holds in pass 1, unchanged black in the last pass (its BB table), unchanged grays rest.
  Lut lutFor(uint8_t was, uint8_t now, int pass) {
    if (pass == 1 && was != now) return now >= 2 ? BW : WB;
    if (pass == 1) return was == 3 ? WW : BB;
    if (was != now && (now == 1 || now == 2)) return now == 1 ? BW : WB;
    return was == 0 && now == 0 ? BB : WW;
  }
} // namespace

bool liveFramePngDrawable(const uint8_t *png, size_t size, uint32_t screenWidth, uint32_t screenHeight) {
  // 8 signature bytes, then the IHDR chunk: length, type, 13 bytes of data
  if (size < 29 || memcmp(png, PNG_SIGNATURE, 8) != 0 || memcmp(&png[12], "IHDR", 4) != 0) return false;
  const uint8_t bitDepth = png[24], colorType = png[25], interlace = png[28];
  return bigEndian32(&png[16]) == screenWidth && bigEndian32(&png[20]) == screenHeight &&
         (bitDepth == 1 || bitDepth == 2) && (colorType == 0 || colorType == 3) && interlace == 0;
}

bool liveFrameRow(const uint8_t *pixels, uint32_t width, uint8_t bitDepth, const uint8_t *palette, uint8_t *out) {
  bool gray = false;
  memset(out, 0, (width + 3) / 4);
  for (uint32_t x = 0; x < width; x++) {
    const uint8_t index = bitDepth == 1 ? (pixels[x / 8] >> (7 - x % 8)) & 1 : level(pixels, x);
    const uint8_t value = palette ? paletteLevel(&palette[3 * index]) : (bitDepth == 1 ? index * 3 : index);
    gray |= value == 1 || value == 2;
    out[x / 4] |= value << (6 - 2 * (x % 4));
  }
  return gray;
}

void liveFramePlaneRow(const uint8_t *was, const uint8_t *now, uint32_t width, int pass, int plane, uint8_t *out) {
  memset(out, 0, (width + 7) / 8);
  for (uint32_t x = 0; x < width; x++) {
    const uint8_t lut = lutFor(level(was, x), level(now, x), pass);
    if ((plane == 1 ? lut >> 1 : lut) & 1) out[x / 8] |= 0x80 >> (x % 8);
  }
}

size_t liveFrameRowPack(const uint8_t *levels, uint32_t width, uint8_t *out, size_t capacity) {
  size_t written = 0;
  for (uint32_t x = 0; x < width;) {
    const uint8_t value = level(levels, x);
    uint32_t run = 1;
    while (x + run < width && run < 64 && level(levels, x + run) == value)
      run++;
    if (written == capacity) return 0;
    out[written++] = value << 6 | (run - 1);
    x += run;
  }
  return written;
}

size_t liveFrameRowUnpack(const uint8_t *in, uint32_t width, uint8_t *levels) {
  size_t read = 0;
  memset(levels, 0, (width + 3) / 4);
  for (uint32_t x = 0; x < width; read++) {
    const uint8_t value = in[read] >> 6;
    for (uint32_t end = x + (in[read] & 63) + 1; x < end && x < width; x++)
      levels[x / 4] |= value << (6 - 2 * (x % 4));
  }
  return read;
}
