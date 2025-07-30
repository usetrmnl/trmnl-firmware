#include <trmnl_image.h>
#include <cstring>
#include <cstdlib>

TrmnlImage *TrmnlNewImage(uint16_t width, uint16_t height, uint8_t bpp,
                          uint16_t stride) {
  uint8_t *data = (uint8_t*)malloc(stride * height);
  TrmnlImage *image = (TrmnlImage*)malloc(sizeof(TrmnlImage));
  image->width = width;
  image->height = height;
  image->bpp = bpp;
  image->stride = stride;
  image->data = data;
  image->owndata = true;
  return image;
  }

  TrmnlImage *TrmnlFromImage(TrmnlImage *srcimage, bool flip) {
    uint16_t stride = StrideOneByte(srcimage->width, srcimage->bpp);
    TrmnlImage *image = TrmnlNewImage(srcimage->width, srcimage->height,
                                      srcimage->bpp, stride);
    for (uint16_t h = 0; h < image->height; ++h) {
      TrmnlCopyRowData(image, srcimage, h, flip ? (image->height - h - 1) : h);
    }
    return image;               
}

TrmnlImage *TrmnlUseImage(TrmnlImage *srcimage) {
   TrmnlImage *image = (TrmnlImage *)malloc(sizeof(TrmnlImage));
  image->width = srcimage->width;
  image->height = srcimage->height;
  image->bpp = srcimage->bpp;
  image->stride = srcimage->stride;
  image->data = srcimage->data;
  image->owndata = false;
  return image;
}

void TrmnlDeleteImage(TrmnlImage *image) {
  if (image->owndata)
    free(image->data);
  free(image);
}

uint16_t StrideOneByte(uint16_t width, uint8_t bpp) {
  return ((bpp * width) + 7) / 8;
}

uint16_t StrideFourByte(uint16_t width, uint8_t bpp) {
  return (((bpp * width) + 31) / 32) * 4;
}

uint8_t *TrmnlGetRowData(TrmnlImage *image, uint16_t row) {
  return image->data + row * image->stride;
}

// images must have same parameters except stride
void TrmnlCopyRowData(TrmnlImage *dstimage, TrmnlImage *srcimage, uint16_t dstrow, uint16_t srcrow) {
  if ((dstimage->width != srcimage->width) ||
      (dstimage->height != srcimage->height) ||
      (dstimage->bpp != srcimage->bpp))
    
    return;
  uint16_t smallerstride = dstimage->stride;
  if (srcimage->stride < smallerstride) {
    smallerstride = srcimage->stride;
  }
  memcpy((char *)TrmnlGetRowData(dstimage, dstrow),
         (const char *)TrmnlGetRowData(srcimage, srcrow), smallerstride);
}