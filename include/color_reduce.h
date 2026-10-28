#pragma once

#include <Arduino.h>

// Reduces a line of PNG pixels (any type and bit depth) to 1, 2 or 4-bpp gray
void ReduceBpp(int iDestBpp, int iPixelType, uint8_t *pPalette, uint8_t *pSrc, uint8_t *pDest, int w, int iSrcBpp);

#ifndef PARALLEL_EPD
// Closest bb_epaper color for an RGB pixel on black/white/red and black/white/yellow/red panels
unsigned char GetBWRPixel(int r, int g, int b);
unsigned char GetBWYRPixel(int r, int g, int b);

// Builds the RGB333 -> Spectra6 lookup used by GetSpectraPixel()
void CreateSpectra6Pal(void);
uint8_t GetSpectraPixel(int r, int g, int b);
#endif // !PARALLEL_EPD
