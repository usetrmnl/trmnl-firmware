#include <cstdint>

enum bmp_err_e {
  BMP_NO_ERR,
  BMP_NOT_BMP,
  BMP_BAD_SIZE,
  BMP_COLOR_SCHEME_FAILED,
  BMP_INVALID_OFFSET,
};

bmp_err_e parseBMPHeader(uint8_t *data, bool &reserved);

// True if color table entry 0 is brighter than entry 1 (i.e. 0 bits are white)
bool bmpIsPaletteReversed(const uint8_t *data);

// Invert the pixels of a 1-bpp bitmap whose palette is reversed so that 0 bits are black
void bmpNormalizePolarity(const uint8_t *data, uint8_t *pixels, uint32_t pixelBytes);
