#include <cstdint>
#if !defined(TRMNL_IMAGE_H)
#define TRMN_IMAGE_H

#include <ctype.h>

// TODO: refcount ?
// TODO: palette
// TODO: adapt for bitdepth > 1

typedef struct TrmnlImage {
  uint16_t width;
  uint16_t height;
  uint8_t bpp;
  uint16_t stride; // size of one row in bytes
  bool owndata;
  uint8_t *data;
} TrmnlImage;

// creates a fully owned image
TrmnlImage *TrmnlNewImage(uint16_t width, uint16_t height, uint8_t bpp,
                          uint16_t stride);

// Create a copy of an image, owns the image, padding is one byte
// can flip image vertically
TrmnlImage *TrmnlFromImage(TrmnlImage *image, bool flip = false);

// creates a proxy image using an already existing image
// all parameters are copied, but does not own the data
TrmnlImage *TrmnlUseImage(TrmnlImage *image);

// Deletes an image
void TrmnlDeleteImage(TrmnlImage *image);

// utilities
// PNG, Screen buffer
uint16_t StrideOneByte(uint16_t width, uint8_t bpp);

// PNG, Screen buffer
uint16_t StrideFourByte(uint16_t width, uint8_t bpp);

uint8_t *TrmnlGetRowData(TrmnlImage *image, uint16_t row);

// takes care of different strides
void TrmnlCopyRowData(TrmnlImage *dstimage, TrmnlImage *srcimage,
                      uint16_t dstrow, uint16_t srcrow);

#endif