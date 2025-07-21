//
// bb_epaper
// Copyright (c) 2024 BitBank Software, Inc.
// Written by Larry Bank (bitbank@pobox.com)
// Project started 9/11/2024
//
// Use of this software is governed by the Business Source License
// included in the file ./LICENSE.
//
// As of the Change Date specified in that file, in accordance with
// the Business Source License, use of this software will be governed
// by the Apache License, Version 2.0, included in the file
// ./APL.txt.
//
// bb_ei_gfx.inl - graphics functions
//
// For constrained CPU environments, the NO_RAM macro will be defined
// to reduce codespace used by each of the functions
//
#ifndef __BB_EP_GFX__
#define __BB_EP_GFX__
#include "Group5.h"
#include "g5dec.inl"

static G5DECIMAGE g5dec;
// forward declarations
void InvertBytes(uint8_t *pData, uint8_t bLen);
void bbepUnicodeString(const char *szMsg, uint8_t *szExtMsg);
const uint8_t ucFont[]PROGMEM = {
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x06,0x5f,0x5f,0x06,0x00,
    0x00,0x07,0x07,0x00,0x07,0x07,0x00,0x14,0x7f,0x7f,0x14,0x7f,0x7f,0x14,
    0x24,0x2e,0x2a,0x6b,0x6b,0x3a,0x12,0x46,0x66,0x30,0x18,0x0c,0x66,0x62,
    0x30,0x7a,0x4f,0x5d,0x37,0x7a,0x48,0x00,0x04,0x07,0x03,0x00,0x00,0x00,
    0x00,0x1c,0x3e,0x63,0x41,0x00,0x00,0x00,0x41,0x63,0x3e,0x1c,0x00,0x00,
    0x08,0x2a,0x3e,0x1c,0x3e,0x2a,0x08,0x00,0x08,0x08,0x3e,0x3e,0x08,0x08,
    0x00,0x00,0x80,0xe0,0x60,0x00,0x00,0x00,0x08,0x08,0x08,0x08,0x08,0x08,
    0x00,0x00,0x00,0x60,0x60,0x00,0x00,0x60,0x30,0x18,0x0c,0x06,0x03,0x01,
    0x3e,0x7f,0x59,0x4d,0x47,0x7f,0x3e,0x40,0x42,0x7f,0x7f,0x40,0x40,0x00,
    0x62,0x73,0x59,0x49,0x6f,0x66,0x00,0x22,0x63,0x49,0x49,0x7f,0x36,0x00,
    0x18,0x1c,0x16,0x53,0x7f,0x7f,0x50,0x27,0x67,0x45,0x45,0x7d,0x39,0x00,
    0x3c,0x7e,0x4b,0x49,0x79,0x30,0x00,0x03,0x03,0x71,0x79,0x0f,0x07,0x00,
    0x36,0x7f,0x49,0x49,0x7f,0x36,0x00,0x06,0x4f,0x49,0x69,0x3f,0x1e,0x00,
    0x00,0x00,0x00,0x66,0x66,0x00,0x00,0x00,0x00,0x80,0xe6,0x66,0x00,0x00,
    0x08,0x1c,0x36,0x63,0x41,0x00,0x00,0x00,0x14,0x14,0x14,0x14,0x14,0x14,
    0x00,0x41,0x63,0x36,0x1c,0x08,0x00,0x00,0x02,0x03,0x59,0x5d,0x07,0x02,
    0x3e,0x7f,0x41,0x5d,0x5d,0x5f,0x0e,0x7c,0x7e,0x13,0x13,0x7e,0x7c,0x00,
    0x41,0x7f,0x7f,0x49,0x49,0x7f,0x36,0x1c,0x3e,0x63,0x41,0x41,0x63,0x22,
    0x41,0x7f,0x7f,0x41,0x63,0x3e,0x1c,0x41,0x7f,0x7f,0x49,0x5d,0x41,0x63,
    0x41,0x7f,0x7f,0x49,0x1d,0x01,0x03,0x1c,0x3e,0x63,0x41,0x51,0x33,0x72,
    0x7f,0x7f,0x08,0x08,0x7f,0x7f,0x00,0x00,0x41,0x7f,0x7f,0x41,0x00,0x00,
    0x30,0x70,0x40,0x41,0x7f,0x3f,0x01,0x41,0x7f,0x7f,0x08,0x1c,0x77,0x63,
    0x41,0x7f,0x7f,0x41,0x40,0x60,0x70,0x7f,0x7f,0x0e,0x1c,0x0e,0x7f,0x7f,
    0x7f,0x7f,0x06,0x0c,0x18,0x7f,0x7f,0x1c,0x3e,0x63,0x41,0x63,0x3e,0x1c,
    0x41,0x7f,0x7f,0x49,0x09,0x0f,0x06,0x1e,0x3f,0x21,0x31,0x61,0x7f,0x5e,
    0x41,0x7f,0x7f,0x09,0x19,0x7f,0x66,0x26,0x6f,0x4d,0x49,0x59,0x73,0x32,
    0x03,0x41,0x7f,0x7f,0x41,0x03,0x00,0x7f,0x7f,0x40,0x40,0x7f,0x7f,0x00,
    0x1f,0x3f,0x60,0x60,0x3f,0x1f,0x00,0x3f,0x7f,0x60,0x30,0x60,0x7f,0x3f,
    0x63,0x77,0x1c,0x08,0x1c,0x77,0x63,0x07,0x4f,0x78,0x78,0x4f,0x07,0x00,
    0x47,0x63,0x71,0x59,0x4d,0x67,0x73,0x00,0x7f,0x7f,0x41,0x41,0x00,0x00,
    0x01,0x03,0x06,0x0c,0x18,0x30,0x60,0x00,0x41,0x41,0x7f,0x7f,0x00,0x00,
    0x08,0x0c,0x06,0x03,0x06,0x0c,0x08,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
    0x00,0x00,0x03,0x07,0x04,0x00,0x00,0x20,0x74,0x54,0x54,0x3c,0x78,0x40,
    0x41,0x7f,0x3f,0x48,0x48,0x78,0x30,0x38,0x7c,0x44,0x44,0x6c,0x28,0x00,
    0x30,0x78,0x48,0x49,0x3f,0x7f,0x40,0x38,0x7c,0x54,0x54,0x5c,0x18,0x00,
    0x48,0x7e,0x7f,0x49,0x03,0x06,0x00,0x98,0xbc,0xa4,0xa4,0xf8,0x7c,0x04,
    0x41,0x7f,0x7f,0x08,0x04,0x7c,0x78,0x00,0x44,0x7d,0x7d,0x40,0x00,0x00,
    0x60,0xe0,0x80,0x84,0xfd,0x7d,0x00,0x41,0x7f,0x7f,0x10,0x38,0x6c,0x44,
    0x00,0x41,0x7f,0x7f,0x40,0x00,0x00,0x7c,0x7c,0x18,0x78,0x1c,0x7c,0x78,
    0x7c,0x78,0x04,0x04,0x7c,0x78,0x00,0x38,0x7c,0x44,0x44,0x7c,0x38,0x00,
    0x84,0xfc,0xf8,0xa4,0x24,0x3c,0x18,0x18,0x3c,0x24,0xa4,0xf8,0xfc,0x84,
    0x44,0x7c,0x78,0x4c,0x04,0x0c,0x18,0x48,0x5c,0x54,0x74,0x64,0x24,0x00,
    0x04,0x04,0x3e,0x7f,0x44,0x24,0x00,0x3c,0x7c,0x40,0x40,0x3c,0x7c,0x40,
    0x1c,0x3c,0x60,0x60,0x3c,0x1c,0x00,0x3c,0x7c,0x60,0x30,0x60,0x7c,0x3c,
    0x44,0x6c,0x38,0x10,0x38,0x6c,0x44,0x9c,0xbc,0xa0,0xa0,0xfc,0x7c,0x00,
    0x4c,0x64,0x74,0x5c,0x4c,0x64,0x00,0x08,0x08,0x3e,0x77,0x41,0x41,0x00,
    0x00,0x00,0x00,0x77,0x77,0x00,0x00,0x41,0x41,0x77,0x3e,0x08,0x08,0x00,
    0x02,0x03,0x01,0x03,0x02,0x03,0x01,0x70,0x78,0x4c,0x46,0x4c,0x78,0x70};

// 5x7 font (in 6x8 cell)
const uint8_t ucSmallFont[] PROGMEM = {
    0x00,0x00,0x00,0x00,0x00,
    0x00,0x06,0x5f,0x06,0x00,
    0x07,0x03,0x00,0x07,0x03,
    0x24,0x7e,0x24,0x7e,0x24,
    0x24,0x2b,0x6a,0x12,0x00,
    0x63,0x13,0x08,0x64,0x63,
    0x36,0x49,0x56,0x20,0x50,
    0x00,0x07,0x03,0x00,0x00,
    0x00,0x3e,0x41,0x00,0x00,
    0x00,0x41,0x3e,0x00,0x00,
    0x08,0x3e,0x1c,0x3e,0x08,
    0x08,0x08,0x3e,0x08,0x08,
    0x00,0xe0,0x60,0x00,0x00,
    0x08,0x08,0x08,0x08,0x08,
    0x00,0x60,0x60,0x00,0x00,
    0x20,0x10,0x08,0x04,0x02,
    0x3e,0x51,0x49,0x45,0x3e,
    0x00,0x42,0x7f,0x40,0x00,
    0x62,0x51,0x49,0x49,0x46,
    0x22,0x49,0x49,0x49,0x36,
    0x18,0x14,0x12,0x7f,0x10,
    0x2f,0x49,0x49,0x49,0x31,
    0x3c,0x4a,0x49,0x49,0x30,
    0x01,0x71,0x09,0x05,0x03,
    0x36,0x49,0x49,0x49,0x36,
    0x06,0x49,0x49,0x29,0x1e,
    0x00,0x6c,0x6c,0x00,0x00,
    0x00,0xec,0x6c,0x00,0x00,
    0x08,0x14,0x22,0x41,0x00,
    0x24,0x24,0x24,0x24,0x24,
    0x00,0x41,0x22,0x14,0x08,
    0x02,0x01,0x59,0x09,0x06,
    0x3e,0x41,0x5d,0x55,0x1e,
    0x7e,0x11,0x11,0x11,0x7e,
    0x7f,0x49,0x49,0x49,0x36,
    0x3e,0x41,0x41,0x41,0x22,
    0x7f,0x41,0x41,0x41,0x3e,
    0x7f,0x49,0x49,0x49,0x41,
    0x7f,0x09,0x09,0x09,0x01,
    0x3e,0x41,0x49,0x49,0x7a,
    0x7f,0x08,0x08,0x08,0x7f,
    0x00,0x41,0x7f,0x41,0x00,
    0x30,0x40,0x40,0x40,0x3f,
    0x7f,0x08,0x14,0x22,0x41,
    0x7f,0x40,0x40,0x40,0x40,
    0x7f,0x02,0x04,0x02,0x7f,
    0x7f,0x02,0x04,0x08,0x7f,
    0x3e,0x41,0x41,0x41,0x3e,
    0x7f,0x09,0x09,0x09,0x06,
    0x3e,0x41,0x51,0x21,0x5e,
    0x7f,0x09,0x09,0x19,0x66,
    0x26,0x49,0x49,0x49,0x32,
    0x01,0x01,0x7f,0x01,0x01,
    0x3f,0x40,0x40,0x40,0x3f,
    0x1f,0x20,0x40,0x20,0x1f,
    0x3f,0x40,0x3c,0x40,0x3f,
    0x63,0x14,0x08,0x14,0x63,
    0x07,0x08,0x70,0x08,0x07,
    0x71,0x49,0x45,0x43,0x00,
    0x00,0x7f,0x41,0x41,0x00,
    0x02,0x04,0x08,0x10,0x20,
    0x00,0x41,0x41,0x7f,0x00,
    0x04,0x02,0x01,0x02,0x04,
    0x80,0x80,0x80,0x80,0x80,
    0x00,0x03,0x07,0x00,0x00,
    0x20,0x54,0x54,0x54,0x78,
    0x7f,0x44,0x44,0x44,0x38,
    0x38,0x44,0x44,0x44,0x28,
    0x38,0x44,0x44,0x44,0x7f,
    0x38,0x54,0x54,0x54,0x08,
    0x08,0x7e,0x09,0x09,0x00,
    0x18,0xa4,0xa4,0xa4,0x7c,
    0x7f,0x04,0x04,0x78,0x00,
    0x00,0x00,0x7d,0x40,0x00,
    0x40,0x80,0x84,0x7d,0x00,
    0x7f,0x10,0x28,0x44,0x00,
    0x00,0x00,0x7f,0x40,0x00,
    0x7c,0x04,0x18,0x04,0x78,
    0x7c,0x04,0x04,0x78,0x00,
    0x38,0x44,0x44,0x44,0x38,
    0xfc,0x44,0x44,0x44,0x38,
    0x38,0x44,0x44,0x44,0xfc,
    0x44,0x78,0x44,0x04,0x08,
    0x08,0x54,0x54,0x54,0x20,
    0x04,0x3e,0x44,0x24,0x00,
    0x3c,0x40,0x20,0x7c,0x00,
    0x1c,0x20,0x40,0x20,0x1c,
    0x3c,0x60,0x30,0x60,0x3c,
    0x6c,0x10,0x10,0x6c,0x00,
    0x9c,0xa0,0x60,0x3c,0x00,
    0x64,0x54,0x54,0x4c,0x00,
    0x08,0x3e,0x41,0x41,0x00,
    0x00,0x00,0x77,0x00,0x00,
    0x00,0x41,0x41,0x3e,0x08,
    0x02,0x01,0x02,0x01,0x00,
    0x3c,0x26,0x23,0x26,0x3c};
//
// Get the size of text in a custom font area
//
void bbepGetStringBox(BBEPDISP *pBBEP, const char *szMsg, BB_RECT *pRect)
{
int cx = 0;
unsigned int c, i = 0;
BB_FONT *pBBF;
BB_FONT_SMALL *pBBFS;
BB_GLYPH *pGlyph;
BB_GLYPH_SMALL *pSmallGlyph;
int miny, maxy;
uint8_t szExtMsg[80];

   miny = 4000; maxy = 0;
   if (pBBEP == NULL || pRect == NULL || szMsg == NULL) return; // bad pointers

   if (pBBEP->pFont == NULL) { // built-in font
        miny = 0;
        switch (pBBEP->iFont) {
            case FONT_6x8:
                cx = 6;
                maxy = 8;
                break;
            case FONT_8x8:
                cx = 8;
                maxy = 8;
                break;
            case FONT_12x16:
                cx = 12;
                maxy = 16;
                break;
            case FONT_16x16:
                cx = 16;
                maxy = 16;
                break;
        }
        cx *= strlen(szMsg);
   } else { // proportional fonts
       bbepUnicodeString(szMsg, szExtMsg); // convert to extended ASCII
       if (pgm_read_word(pBBEP->pFont) == BB_FONT_MARKER) {
           pBBF = (BB_FONT *)pBBEP->pFont; pBBFS = NULL;
       } else { // small font
           pBBFS = (BB_FONT_SMALL *)pBBEP->pFont; pBBF = NULL;
       }
       while (szExtMsg[i]) {
           c = szExtMsg[i++];
           if (pBBF) { // big font
               if (c < pgm_read_byte(&pBBF->first) || c > pgm_read_byte(&pBBF->last)) // undefined character
                   continue; // skip it
               c -= pgm_read_byte(&pBBF->first); // first char of font defined
               pGlyph = &pBBF->glyphs[c];
               cx += pgm_read_word(&pGlyph->xAdvance);
               if ((int16_t)pgm_read_word(&pGlyph->yOffset) < miny) miny = pgm_read_word(&pGlyph->yOffset);
               if (pgm_read_word(&pGlyph->height)+(int16_t)pgm_read_word(&pGlyph->yOffset) > maxy) maxy = pgm_read_word(&pGlyph->height)+(int16_t)pgm_read_word(&pGlyph->yOffset);
           } else {  // small font
               if (c < pgm_read_byte(&pBBFS->first) || c > pgm_read_byte(&pBBFS->last)) // undefined character
                   continue; // skip it
               c -= pgm_read_byte(&pBBFS->first); // first char of font defined
               pSmallGlyph = &pBBFS->glyphs[c];
               cx += pgm_read_byte(&pSmallGlyph->xAdvance);
               if ((int8_t)pgm_read_byte(&pSmallGlyph->yOffset) < miny) miny = (int8_t)pgm_read_byte(&pSmallGlyph->yOffset);
               if (pgm_read_byte(&pSmallGlyph->height)+(int8_t)pgm_read_byte(&pSmallGlyph->yOffset) > maxy) maxy = pgm_read_byte(&pSmallGlyph->height)+(int8_t)pgm_read_byte(&pSmallGlyph->yOffset);
           } // small
       }
   } // prop fonts
   pRect->w = cx;
   pRect->x = pBBEP->iCursorX;
   pRect->y = pBBEP->iCursorY + miny;
   pRect->h = maxy - miny + 1;
} /* bbepGetStringBox() */

//
// Draw a sprite of any size in any position
// If it goes beyond the left/right or top/bottom edges
// it's trimmed to show the valid parts
// This function requires a back buffer to be defined
// The priority color (0 or 1) determines which color is painted
// when a 1 is encountered in the source image.
//
void bbepDrawSprite(BBEPDISP *pBBEP, const uint8_t *pSprite, int cx, int cy, int iPitch, int x, int y, uint8_t iColor)
{
    int tx, ty, dx, dy, iStartX;
    uint8_t *s, pix, ucSrcMask;
    
    if (pBBEP == NULL) return;
    if (x+cx < 0 || y+cy < 0 || x >= pBBEP->native_width || y >= pBBEP->native_height) {
        pBBEP->last_error = BBEP_ERROR_BAD_PARAMETER;
        return; // out of bounds
    }
    iColor = pBBEP->pColorLookup[iColor & 0xf]; // translate the color for this display type

    dy = y; // destination y
    if (y < 0) // skip the invisible parts
    {
        cy += y;
        y = -y;
        pSprite += (y * iPitch);
        dy = 0;
    }
    if (dy + cy > pBBEP->native_height)
        cy = pBBEP->native_height - y;
    if (pBBEP->ucScreen == NULL) { // no backbuffer, draw it direct to EPD
        uint8_t u8CMD, u8CMD1, u8CMD2, bInvert = 0;
        int iDestPitch;
        // start writing into the correct plane
        if (pBBEP->chip_type == BBEP_CHIP_UC81xx) {
            if (pBBEP->iFlags & BBEP_RED_SWAPPED) {
                u8CMD1 = UC8151_DTM1;
                u8CMD2 = UC8151_DTM2;
            } else {
                u8CMD1 = UC8151_DTM2;
                u8CMD2 = UC8151_DTM1;
            }
        } else {
            u8CMD1 = SSD1608_WRITE_RAM;
            u8CMD2 = SSD1608_WRITE_ALTRAM;
        }
        u8CMD = u8CMD1;
        if (iColor == BBEP_BLACK) bInvert = 1;
        else if (iColor == BBEP_RED && (pBBEP->iFlags & BBEP_3COLOR)) {
            u8CMD = u8CMD2; // second plane is red (inverted)
        }
        s = (uint8_t *)pSprite;
        // set the memory window for this character
        cx += (x & 7); // add parital byte
        bbepSetAddrWindow(pBBEP, x, y, cx, cy);
        iDestPitch = (cx+7)/8;
        bbepWriteCmd(pBBEP, u8CMD); // memory write command
        for (int ty=dy; ty<dy+cy; ty++) {
            memcpy(u8Cache, s, iPitch);
            s += iPitch;
            if (x & 7) { // need to shift it over by 1-7 bits
                uint8_t *s = u8Cache, uc1, uc0 = 0; // last shifted byte
                uint8_t n = x & 7; // shift amount
                for (int j=0; j<cx+7; j+= 8) {
                    uc1 = *s;
                    uc0 |= (uc1 >> n);
                    *s++ = uc0;
                    uc0 = uc1 << (8-n);
                }
                *s++ = uc0; // store final byte
                *s++ = 0; // and a zero for good measure
            }
            if (bInvert) InvertBytes(u8Cache, iDestPitch);
            bbepWriteData(pBBEP, u8Cache, iDestPitch); // write each row into the EPD framebuffer
        } // for y
    } else {
#ifndef NO_RAM
        iStartX = 0;
        dx = x;
        if (x < 0)
        {
            cx += x;
            x = -x;
            iStartX = x;
            dx = 0;
        }
        if (x + cx > pBBEP->native_width)
            cx = pBBEP->native_width - x;
        for (ty=0; ty<cy; ty++)
        {
            s = (uint8_t *)&pSprite[(iStartX >> 3)];
            ucSrcMask = 0x80 >> (iStartX & 7);
            pix = *s++;
            for (tx=0; tx<cx; tx++) {
                if (pix & ucSrcMask) { // set pixel in source, set it in dest
                    (*pBBEP->pfnSetPixelFast)(pBBEP, dx+tx, dy+ty, iColor);
                }
                ucSrcMask >>= 1;
                if (ucSrcMask == 0) { // read next byte
                    ucSrcMask = 0x80;
                    pix = *s++;
                }
            } // for tx
            pSprite += iPitch;
        } // for ty
#endif // NO_RAM
    }
} /* bbepDrawSprite() */
//
// Set (or clear) an individual pixel
// These functions only works with a back buffer defined. A bufferless version
// is possible, but would be too impractical.
//
int bbepSetPixel4Clr(void *pb, int x, int y, unsigned char ucColor)
{
int i;
int iPitch;
uint8_t u8, ucMask;
BBEPDISP *pBBEP = (BBEPDISP *)pb;

    // only available for local buffer operations
    if (!pBBEP || !pBBEP->ucScreen) return BBEP_ERROR_BAD_PARAMETER;
    ucColor = pBBEP->pColorLookup[ucColor & 0xf]; // translate the color for this display type

    ucMask = 0xc0 >> ((x & 3)*2);
    iPitch = (pBBEP->width+3)>>2;
    i = (x >> 2) + (y * iPitch);
    if (i < 0 || i >= (iPitch * pBBEP->height)) return BBEP_ERROR_BAD_PARAMETER;
    u8 = pBBEP->ucScreen[i];
    u8 &= ~ucMask;
    u8 |= ucColor << ((3-(x & 3))*2);
    pBBEP->ucScreen[i] = u8;
    return BBEP_SUCCESS;

} /* bbepSetPixel4Clr() */
// Fast version (no pointer verification + no boundary checking)
void bbepSetPixelFast4Clr(void *pb, int x, int y, unsigned char ucColor)
{
int i;
int iPitch;
uint8_t u8, ucMask;
BBEPDISP *pBBEP = (BBEPDISP *)pb;
    
    ucMask = 0xc0 >> ((x & 3)*2);
    iPitch = (pBBEP->width+3)>>2;
    i = (x >> 2) + (y * iPitch);
    u8 = pBBEP->ucScreen[i];
    u8 &= ~ucMask;
    u8 |= ucColor << ((3-(x & 3))*2);
    pBBEP->ucScreen[i] = u8;
} /* bbepSetPixelFast4Clr() */

int bbepSetPixel4Gray(void *pb, int x, int y, unsigned char ucColor)
{
int i;
int iPitch, iSize;
BBEPDISP *pBBEP = (BBEPDISP *)pb;
const uint8_t u8Mask = 0x80 >> (x & 7);

    // only available for local buffer operations
    if (!pBBEP || !pBBEP->ucScreen) return BBEP_ERROR_BAD_PARAMETER;
    ucColor = pBBEP->pColorLookup[ucColor & 0xf]; // translate the color for this display type

    iPitch = (pBBEP->width+7)>>3;
    iSize = ((pBBEP->native_width+7)>>3) * pBBEP->native_height;

    i = (x >> 3) + (y * iPitch);
    if (x < 0 || x >= pBBEP->width || i < 0 || i > iSize-1) { // off the screen
        pBBEP->last_error = BBEP_ERROR_BAD_PARAMETER;
        return BBEP_ERROR_BAD_PARAMETER;
    }
    if (ucColor & 1) { // first plane
        pBBEP->ucScreen[i] |= u8Mask;
    } else {
        pBBEP->ucScreen[i] &= ~u8Mask;
    }
    if (ucColor & 2) { // second plane
        pBBEP->ucScreen[iSize + i] |= u8Mask;
    } else {
        pBBEP->ucScreen[iSize + i] &= ~u8Mask;
    }
    return BBEP_SUCCESS;
} /* bbepSetPixel4Gray() */

void bbepSetPixelFast4Gray(void *pb, int x, int y, unsigned char ucColor)
{
int i;
int iPitch, iSize;
BBEPDISP *pBBEP = (BBEPDISP *)pb;
const uint8_t u8Mask = 0x80 >> (x & 7);

    iPitch = (pBBEP->width+7)>>3;
    iSize = ((pBBEP->native_width+7)>>3) * pBBEP->native_height;
    i = (x >> 3) + (y * iPitch);

    if (ucColor & 1) { // first plane
        pBBEP->ucScreen[i] |= u8Mask;
    } else {
        pBBEP->ucScreen[i] &= ~u8Mask;
    }
    if (ucColor & 2) { // second plane
        pBBEP->ucScreen[iSize + i] |= u8Mask;
    } else {
        pBBEP->ucScreen[iSize + i] &= ~u8Mask;
    }
} /* bbepSetPixelFast4Gray() */

int bbepSetPixel3Clr(void *pb, int x, int y, unsigned char ucColor)
{
int i;
int iPitch, iSize;
BBEPDISP *pBBEP = (BBEPDISP *)pb;
    
    // only available for local buffer operations
    if (!pBBEP || !pBBEP->ucScreen) return BBEP_ERROR_BAD_PARAMETER;
    ucColor = pBBEP->pColorLookup[ucColor & 0xf]; // translate the color for this display type

    iPitch = (pBBEP->width+7)>>3;
    iSize = ((pBBEP->native_width+7)>>3) * pBBEP->native_height;
    
    i = (x >> 3) + (y * iPitch);
    if (x < 0 || x >= pBBEP->width || i < 0 || i > iSize-1) { // off the screen
        pBBEP->last_error = BBEP_ERROR_BAD_PARAMETER;
        return BBEP_ERROR_BAD_PARAMETER;
    }
    if (ucColor >= BBEP_YELLOW) { // yellow/red has priority
        pBBEP->ucScreen[iSize + i] |= (0x80 >> (x & 7));
    } else {
        pBBEP->ucScreen[iSize + i] &= ~(0x80 >> (x & 7)); // clear red plane bit
        if (ucColor == BBEP_WHITE) {
            pBBEP->ucScreen[i] |= (0x80 >> (x & 7));
        } else { // must be black
            pBBEP->ucScreen[i] &= ~(0x80 >> (x & 7));
        }
    }
    return BBEP_SUCCESS;
} /* bbepSetPixel3Clr() */

void bbepSetPixelFast3Clr(void *pb, int x, int y, unsigned char ucColor)
{
int i;
int iPitch, iSize;
BBEPDISP *pBBEP = (BBEPDISP *)pb;
    
    iPitch = (pBBEP->width+7)>>3;
    iSize = ((pBBEP->native_width+7)>>3) * pBBEP->native_height;
    
    i = (x >> 3) + (y * iPitch);
    if (ucColor >= BBEP_YELLOW) { // yellow/red has priority
        pBBEP->ucScreen[iSize + i] |= (0x80 >> (x & 7));
    } else {
        pBBEP->ucScreen[iSize + i] &= ~(0x80 >> (x & 7)); // clear red plane bit
        if (ucColor == BBEP_WHITE) {
            pBBEP->ucScreen[i] |= (0x80 >> (x & 7));
        } else { // must be black
            pBBEP->ucScreen[i] &= ~(0x80 >> (x & 7));
        }
    }
} /* bbepSetPixelFast3Clr() */

int bbepSetPixel2Clr(void *pb, int x, int y, unsigned char ucColor)
{
    int i;
    int iPitch, iSize;
    uint8_t u8;
    BBEPDISP *pBBEP = (BBEPDISP *)pb;
    
    // only available for local buffer operations
    if (!pBBEP || !pBBEP->ucScreen) return BBEP_ERROR_BAD_PARAMETER;
    ucColor = pBBEP->pColorLookup[ucColor & 0xf]; // translate the color for this display type

    iPitch = (pBBEP->width+7)>>3;
    iSize = ((pBBEP->native_width+7)>>3) * pBBEP->native_height;
    
    i = (x >> 3) + (y * iPitch);
    if (x < 0 || x >= pBBEP->width || i < 0 || i > iSize-1) { // off the screen
        pBBEP->last_error = BBEP_ERROR_BAD_PARAMETER;
        return BBEP_ERROR_BAD_PARAMETER;
    }
    if (pBBEP->iPlane == PLANE_1) {
        i += iSize;
    }
    u8 = pBBEP->ucScreen[i];
    if (ucColor == BBEP_WHITE) {
        u8 |= (0x80 >> (x & 7));
    } else { // must be black
        u8 &= ~(0x80 >> (x & 7));
    }
    pBBEP->ucScreen[i] = u8;
    return BBEP_SUCCESS;
} /* bbepSetPixel2Clr() */

void bbepSetPixelFast2Clr(void *pb, int x, int y, unsigned char ucColor)
{
    int i;
    int iPitch, iSize;
    uint8_t u8;
    BBEPDISP *pBBEP = (BBEPDISP *)pb;
    
    iPitch = (pBBEP->width+7)>>3;
    iSize = ((pBBEP->native_width+7)>>3) * pBBEP->native_height;
    
    i = (x >> 3) + (y * iPitch);
    if (pBBEP->iPlane == PLANE_1) {
        i += iSize;
    }
    u8 = pBBEP->ucScreen[i];
    if (ucColor == BBEP_WHITE) {
        u8 |= (0x80 >> (x & 7));
    } else { // must be black
        u8 &= ~(0x80 >> (x & 7));
    }
    pBBEP->ucScreen[i] = u8;
} /* bbepSetPixelFast2Clr() */

int bbepSetPixel16Clr(void *pb, int x, int y, unsigned char ucColor)
{
    int i;
    int iPitch, iSize;
    uint8_t u8;
    BBEPDISP *pBBEP = (BBEPDISP *)pb;
    
    // only available for local buffer operations
    if (!pBBEP || !pBBEP->ucScreen) return BBEP_ERROR_BAD_PARAMETER;
    ucColor = pBBEP->pColorLookup[ucColor & 0xf]; // translate the color for this display type

    iPitch = pBBEP->width >> 1;
    iSize = (pBBEP->native_width >> 1) * pBBEP->native_height;
    
    i = (x >> 1) + (y * iPitch);
    if (x < 0 || x >= pBBEP->width || i < 0 || i > iSize-1) { // off the screen
        pBBEP->last_error = BBEP_ERROR_BAD_PARAMETER;
        return BBEP_ERROR_BAD_PARAMETER;
    }
    u8 = pBBEP->ucScreen[i];
    if (x & 1) {
        u8 &= 0xf0;
        u8 |= ucColor;
    } else {
        u8 &= 0x0f;
        u8 |= (ucColor << 4);
    }
    pBBEP->ucScreen[i] = u8;
    return BBEP_SUCCESS;
} /* bbepSetPixel16Clr() */

void bbepSetPixelFast16Clr(void *pb, int x, int y, unsigned char ucColor)
{
    int i;
    int iPitch;
    uint8_t u8;
    BBEPDISP *pBBEP = (BBEPDISP *)pb;
    
    iPitch = pBBEP->width >> 1;
    i = (x >> 1) + (y * iPitch);
    u8 = pBBEP->ucScreen[i];
    if (x & 1) {
        u8 &= 0xf0;
        u8 |= ucColor;
    } else {
        u8 &= 0x0f;
        u8 |= (ucColor << 4);
    }
    pBBEP->ucScreen[i] = u8;
} /* bbepSetPixelFast16Clr() */

//
// Invert font data
//
void InvertBytes(uint8_t *pData, uint8_t bLen)
{
    uint8_t i;
    for (i=0; i<bLen; i++)
    {
        *pData = ~(*pData);
        pData++;
    }
} /* InvertBytes() */
//
// Load a 1-bpp Group5 compressed bitmap
// Pass the pointer to the beginning of the G5 file
// If the FG == BG color, and there is a back buffer, it will
// draw the 1's bits as the FG color and leave
// the background (0 pixels) unchanged - aka transparent.
//
int bbepLoadG5(BBEPDISP *pBBEP, const uint8_t *pG5, int x, int y, int iFG, int iBG, float fScale)
{
    uint16_t rc, tx, ty, cx, cy, dx, dy, size;
    int width, height;
    BB_BITMAP *pbbb;
    uint32_t u32Frac, u32XAcc, u32YAcc; // integer fraction vars

    if (pBBEP == NULL || pG5 == NULL || fScale < 0.01) return BBEP_ERROR_BAD_PARAMETER;
    if (iFG != BBEP_TRANSPARENT) {
        iFG = pBBEP->pColorLookup[iFG & 0xf]; // translate the color for this display type
    }
    if (iBG != BBEP_TRANSPARENT) {
        iBG = pBBEP->pColorLookup[iBG & 0xf];
    }
    pbbb = (BB_BITMAP *)pG5;
    if (pgm_read_word(&pbbb->u16Marker) != BB_BITMAP_MARKER) return BBEP_ERROR_BAD_DATA;
    u32Frac = (uint32_t)(65536.0f / fScale); // calculate the fraction to advance the destination x/y
    cx = pgm_read_word(&pbbb->width);
    cy = pgm_read_word(&pbbb->height);
    width = pBBEP->width;
    height = pBBEP->height;
    // Calculate scaled destination size
    dx = (int)(fScale * (float)cx);
    dy = (int)(fScale * (float)cy);
    size = pgm_read_word(&pbbb->size);
    if (iFG == -1) iFG = BBEP_WHITE;
    if (iBG == -1) iBG = BBEP_BLACK;
    rc = g5_decode_init(&g5dec, cx, cy, (uint8_t *)&pbbb[1], size);
    if (rc != G5_SUCCESS) return BBEP_ERROR_BAD_DATA; // corrupt data?
    if (!pBBEP->ucScreen) { // no back buffer
        bbepSetAddrWindow(pBBEP, x, y, cx+(x&7), cy);
        bbepStartWrite(pBBEP, pBBEP->iPlane); // get ready to write
        dy = cy; // scaling is only supported on internal framebuffers
        u32Frac = 65536; // force to 1.0 scale
    }
    u32YAcc = 65536; // force first line to get decoded
    for (ty=y; ty<y+dy && ty < height; ty++) {
        uint8_t u8, *s, src_mask;
        while (u32YAcc >= 65536) { // advance to next source line
            g5_decode_line(&g5dec, u8Cache);
            u32YAcc -= 65536;
        }
        if (!pBBEP->ucScreen) {
            if (x & 7) { // need to shift it over by 1-7 bits
                uint8_t *d = s = u8Cache, uc1, uc0; // last shifted byte
                if (iFG == BBEP_WHITE) {
                    uc0 = 0;
                } else {
                    uc0 = 0xff << (7-(x & 7));
                }
                uint8_t n = x & 7; // shift amount
                for (int j=0; j<cx+7; j+= 8) {
                    uc1 = *s++;
                    uc0 |= (uc1 >> n);
                    *d++ = uc0;
                    uc0 = uc1 << (8-n);
                }
                *d++ = uc0; // store final byte
                *d++ = 0; // and a zero for good measure
            }
            if (iFG == BBEP_WHITE) { // inverted
                InvertBytes(u8Cache, (cx+(x&7)+7)>>3);
            }
            bbepWriteData(pBBEP, u8Cache, (cx+(x&7)+7)>>3);
        } else { // use the setPixel function for more features
#ifndef NO_RAM
            s = u8Cache;
            u32XAcc = 0;
            u8 = *s++; // grab first source byte (8 pixels)
            src_mask = 0x80;
            for (tx=x; tx<x+dx && tx < width; tx++) {
                if (u8 & src_mask) {
                    if (iFG != BBEP_TRANSPARENT)
                        (*pBBEP->pfnSetPixelFast)(pBBEP, tx, ty, (uint8_t)iFG);
                } else {
                    if (iBG != BBEP_TRANSPARENT)
                        (*pBBEP->pfnSetPixelFast)(pBBEP, tx, ty, (uint8_t)iBG);
                }
                u32XAcc += u32Frac;
                while (u32XAcc >= 65536) {
                    u32XAcc -= 65536; // whole source pixel horizontal movement
                    src_mask >>= 1;
                    if (src_mask == 0) { // need to load the next byte
                        u8 = *s++;
                        src_mask = 0x80;
                    }
                }
            } // for tx
#endif // NO_RAM
        }
        u32YAcc += u32Frac;
    } // for y
    return BBEP_SUCCESS;
} /* bbepLoadG5() */
//
// Load a 1-bpp Windows bitmap
// Pass the pointer to the beginning of the BMP file
// If the FG == BG color, it will
// draw the 1's bits as the FG color and leave
// the background (0 pixels) unchanged - aka transparent.
//
int bbepLoadBMP(BBEPDISP *pBBEP, const uint8_t *pBMP, int dx, int dy, int iFG, int iBG)
{
    int16_t i16, cx, cy;
    int iOffBits; // offset to bitmap data
    int x, y, iPitch;
    uint8_t b=0, *s;
    uint8_t src_mask;
    uint8_t bFlipped = 0;
    
    if (pBBEP == NULL || pBMP == NULL) return BBEP_ERROR_BAD_PARAMETER;
    iFG = pBBEP->pColorLookup[iFG & 0xf]; // translate the color for this display type
    iBG = pBBEP->pColorLookup[iBG & 0xf];
    // Don't use pgm_read_word because it can cause an unaligned
    // access on RP2040 for odd addresses
    i16 = pgm_read_byte(pBMP);
    i16 += (pgm_read_byte(&pBMP[1]) << 8);
    if (i16 != 0x4d42) { // must start with 'BM'
        pBBEP->last_error = BBEP_ERROR_BAD_DATA;
        return BBEP_ERROR_BAD_DATA; // not a BMP file
    }
    cx = pgm_read_byte(pBMP + 18);
    cx += (pgm_read_byte(pBMP+19)<<8);
    if (cx + dx > pBBEP->width) { // must fit on the display
        pBBEP->last_error = BBEP_ERROR_BAD_PARAMETER;
        return BBEP_ERROR_BAD_PARAMETER;
    }
    cy = pgm_read_byte(pBMP + 22);
    cy += (pgm_read_byte(pBMP+23)<<8);
    if (cy < 0) cy = -cy;
    else bFlipped = 1;
    if (cy + dy > pBBEP->height) { // must fit on the display
        pBBEP->last_error = BBEP_ERROR_BAD_PARAMETER;
        return BBEP_ERROR_BAD_PARAMETER;
    }
    if (pBBEP->ucScreen == NULL) {
        // no BG specified or no back buffer, BG color is opposite of FG
        if (iFG == -1) iFG = BBEP_WHITE;
        if (iBG == -1) iBG = BBEP_BLACK;
        bbepSetAddrWindow(pBBEP, dx, dy, cx+(dx&7), cy);
        bbepStartWrite(pBBEP, pBBEP->iPlane); // get ready to write the data
    }
    
    i16 = pgm_read_byte(pBMP + 28);
    i16 += (pgm_read_byte(pBMP+29)<<8);
    if (i16 != 1) { // must be 1 bit per pixel
        pBBEP->last_error = BBEP_ERROR_BAD_DATA;
        return BBEP_ERROR_BAD_DATA;
    }
    iOffBits = pgm_read_byte(pBMP + 10);
    iOffBits += (pgm_read_byte(pBMP+11));
    iPitch = (((cx+7)>>3) + 3) & 0xfffc; // must be DWORD aligned
    if (bFlipped)
    {
        iOffBits += ((cy-1) * iPitch); // start from bottom
        iPitch = -iPitch;
    }
    if (!pBBEP->ucScreen || iFG >= BBEP_YELLOW) { // this will override the B/W plane, so invert things
        x = iFG;
        iFG = iBG;
        iBG = x; // swap colors
    }
    for (y=0; y<cy; y++) {
        s = (uint8_t *)&pBMP[iOffBits + (y*iPitch)];
        if (!pBBEP->ucScreen) {
            if (dx & 7) { // need to shift it over by 1-7 bits
                uint8_t *d = u8Cache, uc1, uc0 = 0; // last shifted byte
                uint8_t n = dx & 7; // shift amount
                for (int j=0; j<cx+7; j+= 8) {
                    uc1 = pgm_read_byte(s++);
                    uc0 |= (uc1 >> n);
                    *d++ = uc0;
                    uc0 = uc1 << (8-n);
                }
                *d++ = uc0; // store final byte
                *d++ = 0; // and a zero for good measure
            } else {
                memcpy_P(u8Cache, s, (cx+7)>>3);
            }
            bbepWriteData(pBBEP, u8Cache, (cx+(dx&7)+7)>>3);
        } else { // use the setPixel function for more features
#ifndef NO_RAM
            src_mask = 0; // make it read a byte to start
            for (x=0; x<cx; x++) {
                if (src_mask == 0) { // need to load the next byte
                    b = pgm_read_byte(s++);
                    src_mask = 0x80; // MSB on left
                }
                if (b & src_mask) {
                    if (iFG != BBEP_TRANSPARENT)
                        (*pBBEP->pfnSetPixelFast)(pBBEP, dx+x, dy+y, (uint8_t)iFG);
                } else {
                    if (iBG != BBEP_TRANSPARENT)
                        (*pBBEP->pfnSetPixelFast)(pBBEP, dx+x, dy+y, (uint8_t)iBG);
                }
                src_mask >>= 1;
            } // for x
#endif // NO_RAM
        }
    } // for y
    return BBEP_SUCCESS;
} /* bbepLoadBMP() */
//
// Load a 4-bpp Windows bitmap for a 3-color bitmap
// Pass the pointer to the beginning of the BMP file
// First pass version assumes a full screen bitmap
//
int bbepLoadBMP3(BBEPDISP *pBBEP, const uint8_t *pBMP, int dx, int dy)
{
    int16_t i16, cx, cy, bpp;
    int x, y, iOffBits; // offset to bitmap data
    int iPitch, iDestPitch;
    int iRedOff, iColors, iPalOff;
    uint8_t uc, b, *s, *d;
    uint8_t dst_mask;
    uint8_t bFlipped = 0;
    uint8_t ucColorMap[16];
    
    if (pBBEP == NULL || pBMP == NULL) return BBEP_ERROR_BAD_PARAMETER;
    if (!(pBBEP->iFlags & BBEP_3COLOR) || pBBEP->ucScreen == 0) {
        pBBEP->last_error = BBEP_ERROR_NOT_SUPPORTED;
        return BBEP_ERROR_NOT_SUPPORTED; // if not 3-color EPD or no back buffer
    }
    iDestPitch = pBBEP->width;
    iRedOff = ((pBBEP->height+7)>>3) * iDestPitch;
    // Need to avoid pgm_read_word because it can cause an
    // unaligned address exception on the RP2040 for odd addresses
    i16 = pgm_read_byte(pBMP);
    i16 += (pgm_read_byte(pBMP+1)<<8);
    if (i16 != 0x4d42) { // must start with 'BM'
        pBBEP->last_error = BBEP_ERROR_BAD_DATA;
        return BBEP_ERROR_BAD_DATA; // not a BMP file
    }
    cx = pgm_read_byte(&pBMP[18]);
    cx += (pgm_read_byte(&pBMP[19]) << 8);
    if (cx + dx > pBBEP->width) { // must fit on the display
        pBBEP->last_error = BBEP_ERROR_BAD_PARAMETER;
        return BBEP_ERROR_BAD_PARAMETER;
    }
    cy = pgm_read_byte(&pBMP[22]);
    cy += (pgm_read_byte(&pBMP[23])<<8);
    if (cy < 0)
        cy = -cy;
    else
        bFlipped = 1;
    if (cy + dy > pBBEP->height) { // must fit on the display
        pBBEP->last_error = BBEP_ERROR_BAD_PARAMETER;
        return BBEP_ERROR_BAD_PARAMETER;
    }
    if (pgm_read_byte(&pBMP[30]) != 0) { // compression must be NONE
        pBBEP->last_error = BBEP_ERROR_BAD_DATA;
        return BBEP_ERROR_BAD_DATA;
    }
    bpp = pgm_read_byte(&pBMP[28]);
    if (bpp != 4) { // must be 4 bits per pixel
        pBBEP->last_error = BBEP_ERROR_BAD_DATA;
        return BBEP_ERROR_BAD_DATA;
    }
    iOffBits = pgm_read_byte(&pBMP[10]);
    iOffBits += (pgm_read_byte(&pBMP[11])<<8);
    iColors = pgm_read_byte(&pBMP[46]); // colors used BMP field
    if (iColors == 0 || iColors > (1<<bpp))
        iColors = (1 << bpp); // full palette
    iPalOff = iOffBits - (4 * iColors); // start of color palette
    iPitch = (((cx+1)>>1) + 3) & 0xfffc; // must be DWORD aligned
    if (bFlipped)
    {
        iOffBits += ((cy-1) * iPitch); // start from bottom
        iPitch = -iPitch;
    }
    // Map the colors to white/black/red with a simple quantization. Convert colors to G3R3B2 and find the closest value (red in the middle)
    // white = 0xff, red = 0x1c, black = 0x00
    for (x=0; x<iColors; x++) {
        uint8_t r, g, b, uc;
        b = pgm_read_byte(&pBMP[iPalOff+(x*4)]);
        g = pgm_read_byte(&pBMP[iPalOff+1+(x*4)]);
        r = pgm_read_byte(&pBMP[iPalOff+2+(x*4)]);
        uc = (b >> 6) | ((r >> 5) << 2) | ((g >> 5) << 5);
        if (uc >= 0x1c) { // check for red/white
            ucColorMap[x] = ((0xff - uc) < (uc - 0x1c)) ? BBEP_WHITE : BBEP_RED;
        } else {
            ucColorMap[x] = ((0x1c - uc) < uc) ? BBEP_RED : BBEP_BLACK;
        }
    }
    for (y=0; y<cy; y++)
    {
        dst_mask = 1 << ((y+dy) & 7);
        d = &pBBEP->ucScreen[(((y+dy)>>3)*iDestPitch)+dx];
        s = (uint8_t *)&pBMP[iOffBits+(y*iPitch)];
        for (x=0; x<cx; x+=2) // work with pixel pairs
        {
            b = pgm_read_byte(s++);
            d[x] &= ~dst_mask; // clear b/w & red planes to start as white
            d[x+1] &= ~dst_mask;
            d[x+iRedOff] &= ~dst_mask;
            d[x+1+iRedOff] &= ~dst_mask;
            uc = ucColorMap[b >> 4]; // left pixel
            if (uc == BBEP_BLACK)
                d[x] |= dst_mask;
            else if (uc == BBEP_RED)
                d[x+iRedOff] |= dst_mask; // we made it white already
            uc = ucColorMap[b & 0xf]; // right pixel
            if (uc == BBEP_BLACK)
                d[x+1] |= dst_mask;
            else if (uc == BBEP_RED)
                d[x+1+iRedOff] |= dst_mask;
        } // for x
    } // for y
    return BBEP_SUCCESS;
} /* bbepLoadBMP3() */
//
// Set the current cursor position
// The column represents the pixel column (0-127)
// The row represents the text row (0-7)
//
void bbepSetCursor(BBEPDISP *pBBEP, int x, int y)
{
    pBBEP->iCursorX = x;
    pBBEP->iCursorY = y;
} /* bbepSetCursor() */
//
// Turn text wrap on or off for the bbepWriteString() function
//
void bbepSetTextWrap(BBEPDISP *pBBEP, int bWrap)
{
    pBBEP->wrap = bWrap;
} /* bbepSetTextWrap() */
//
// Convert a single Unicode character into codepage 1252 (extended ASCII)
//
uint8_t bbepUnicodeTo1252(uint16_t u16CP)
{
            // convert supported Unicode values to codepage 1252
            switch(u16CP) { // they're all over the place, so check each
                case 0x20ac:
                    u16CP = 0x80; // euro sign
                    break;
                case 0x201a: // single low quote
                    u16CP = 0x82;
                    break;
                case 0x192: // small F with hook
                    u16CP = 0x83;
                    break;
                case 0x201e: // double low quote
                    u16CP = 0x84;
                    break;
                case 0x2026: // hor ellipsis
                    u16CP = 0x85;
                    break;
                case 0x2020: // dagger
                    u16CP = 0x86;
                    break;
                case 0x2021: // double dagger
                    u16CP = 0x87;
                    break;
                case 0x2c6: // circumflex
                    u16CP = 0x88;
                    break;
                case 0x2030: // per mille
                    u16CP = 0x89;
                    break;
                case 0x160: // capital S with caron
                    u16CP = 0x8a;
                    break;
                case 0x2039: // single left pointing quote
                    u16CP = 0x8b;
                    break;
                case 0x152: // capital ligature OE
                    u16CP = 0x8c;
                    break;
                case 0x17d: // captial Z with caron
                    u16CP = 0x8e;
                    break;
                case 0x2018: // left single quote
                    u16CP = 0x91;
                    break;
                case 0x2019: // right single quote
                    u16CP = 0x92;
                    break;
                case 0x201c: // left double quote
                    u16CP = 0x93;
                    break;
                case 0x201d: // right double quote
                    u16CP = 0x94;
                    break;
                case 0x2022: // bullet
                    u16CP = 0x95;
                    break;
                case 0x2013: // en dash
                    u16CP = 0x96;
                    break;
                case 0x2014: // em dash
                    u16CP = 0x97;
                    break;
                case 0x2dc: // small tilde
                    u16CP = 0x98;
                    break;
                case 0x2122: // trademark
                    u16CP = 0x99;
                    break;
                case 0x161: // small s with caron
                    u16CP = 0x9a;
                    break;
                case 0x203a: // single right quote
                    u16CP = 0x9b;
                    break;
                case 0x153: // small ligature oe
                    u16CP = 0x9c;
                    break;
                case 0x17e: // small z with caron
                    u16CP = 0x9e;
                    break;
                case 0x178: // capital Y with diaeresis
                    u16CP = 0x9f;
                    break;
                default:
                    if (u16CP > 0xff) u16CP = 32; // something went wrong
                    break;
            } // switch on character
    return (uint8_t)u16CP;
} /* bbepUnicodeTo1252() */
//
// Convert a Unicode string into our extended ASCII set (codepage 1252)
//
void bbepUnicodeString(const char *szMsg, uint8_t *szExtMsg)
{
int i, j;
uint8_t c;
uint16_t u16CP; // 16-bit codepoint encoded by the multi-byte sequence

    i = j = 0;
    while (szMsg[i]) {
        c = szMsg[i++];
        if (c < 0x80) { // normal 7-bit ASCII
             u16CP = c;
        } else { // multibyte
             if (c < 0xe0) { // first 0x800 characters
                  u16CP = (c & 0x3f) << 6;
                  u16CP += (szMsg[i++] & 0x3f);
             } else if (c < 0xf0) { // 0x800 to 0x10000
                  u16CP = (c & 0x3f) << 12;
                  u16CP += ((szMsg[i++] & 0x3f)<<6);
                  u16CP += (szMsg[i++] & 0x3f);
             } else { // 0x10001 to 0x20000
                  u16CP = 32; // convert to spaces (nothing supported here)
             }
        } // multibyte
        szExtMsg[j++] = bbepUnicodeTo1252(u16CP);
    } // while szMsg[i]
    szExtMsg[j++] = 0; // zero terminate it
} /* bbepUnicodeString() */
//
// Draw a string of BB_FONT characters directly into the EPD framebuffer
//
int bbepWriteStringCustom(BBEPDISP *pBBEP, void *pFont, int x, int y, char *szMsg, int iColor, uint8_t iPlane)
{
    int rc, i, h, w, j, end_y, dx, dy, tx, ty, tw, iSrcPitch, iPitch, iBG;
    signed int n;
    unsigned int c, bInvert = 0;
    uint8_t *s, uc0, uc1;
    BB_FONT *pBBF;
    BB_FONT_SMALL *pBBFS;
    BB_GLYPH *pGlyph;
    BB_GLYPH_SMALL *pSmallGlyph;
    uint8_t *pBits, u8CMD1, u8CMD2, u8CMD, u8EndMask;
    uint8_t szExtMsg[256]; // translated extended ASCII message text
    uint8_t first, last;
    
    if (pBBEP == NULL) return BBEP_ERROR_BAD_PARAMETER;
    if (pFont == NULL) {
        pBBEP->last_error = BBEP_ERROR_BAD_PARAMETER;
        return BBEP_ERROR_BAD_PARAMETER; // invalid param
    }
// Determine if we're using a small or large font
    if (pgm_read_word(pFont) == BB_FONT_MARKER) {
        pBBF = (BB_FONT *)pFont;
        pBBFS = NULL;
    } else {
        pBBFS = (BB_FONT_SMALL *)pFont;
        pBBF = NULL;
    }

    if (iColor != BBEP_TRANSPARENT) {
        iColor = pBBEP->pColorLookup[iColor & 0xf]; // translate the color for this display type
    }
    if (szMsg[1] == 0 && (szMsg[0] & 0x80)) { // single byte means we're coming from the Arduino write() method with pre-converted extended ASCII
        szExtMsg[0] = szMsg[0]; szExtMsg[1] = 0;
    } else {
        bbepUnicodeString(szMsg, szExtMsg); // convert to extended ASCII
    }
    iBG = pBBEP->iBG;
    if (iBG == -1) iBG = BBEP_TRANSPARENT; // -1 = don't care
    if (iBG != BBEP_TRANSPARENT) {
        iBG = pBBEP->pColorLookup[iBG & 0xf];
    }
    if (x == -1)
        x = pBBEP->iCursorX;
    if (y == -1)
        y = pBBEP->iCursorY;
    if (pBBF) {
        first = pgm_read_byte(&pBBF->first);
        last = pgm_read_byte(&pBBF->last);
    } else {
        first = pgm_read_byte(&pBBFS->first);
        last = pgm_read_byte(&pBBFS->last);
    }
    if (x == CENTER_X) { // center the string on the e-paper
        dx = i = 0;
        while (szExtMsg[i]) {
            c = szExtMsg[i++];
            if (c < first || c > last) // undefined character
                continue; // skip it
            c -= first; // first char of font defined
            if (pBBF) {
                pGlyph = &pBBF->glyphs[c]; // glyph info for this character
                dx += pgm_read_word(&pGlyph->xAdvance);
            } else { // small font
                pSmallGlyph = &pBBFS->glyphs[c]; // glyph info for this character
                dx += pgm_read_byte(&pSmallGlyph->xAdvance);
            }
        }
        x = (pBBEP->width - dx)/2;
        if (x < 0) x = 0;
    }
    // Point to the start of the compressed data
    if (pBBF) {
        pBits = (uint8_t *)pBBF;
        pBits += sizeof(BB_FONT);
        pBits += (last - first + 1) * sizeof(BB_GLYPH);
    } else {
        pBits = (uint8_t *)pBBFS;
        pBits += sizeof(BB_FONT_SMALL);
        pBits += (last - first + 1) * sizeof(BB_GLYPH_SMALL);
    }
    i = 0;
    while (szExtMsg[i] && x < pBBEP->width && y < pBBEP->height) {
        int16_t xOffset, yOffset;
        uint32_t u32Offset, u32Rot, xAdvance;
        c = szExtMsg[i++];
        if (c < first || c > last) // undefined character
            continue; // skip it
        c -= first; // first char of font defined
        if (pBBF) {
            pGlyph = &pBBF->glyphs[c]; // glyph info for this character
            w = pgm_read_word(&pGlyph->width);
            h = pgm_read_word(&pGlyph->height);
            u32Offset = pgm_read_word(&pGlyph->bitmapOffset);
            u32Rot = pgm_read_dword(&pBBF->rotation);
            xAdvance = pgm_read_word(&pGlyph->xAdvance);
            xOffset = (int16_t)pgm_read_word(&pGlyph->xOffset);
            yOffset = (int16_t)pgm_read_word(&pGlyph->yOffset);
        } else { // small font
            pSmallGlyph = &pBBFS->glyphs[c]; // glyph info for this character
            w = pgm_read_byte(&pSmallGlyph->width);
            h = pgm_read_byte(&pSmallGlyph->height);
            u32Offset = pgm_read_word(&pSmallGlyph->bitmapOffset);
            u32Rot = pgm_read_dword(&pBBFS->rotation);
            xAdvance = pgm_read_byte(&pSmallGlyph->xAdvance);
            xOffset = (int8_t)pgm_read_byte(&pSmallGlyph->xOffset);
            yOffset = (int8_t)pgm_read_byte(&pSmallGlyph->yOffset);
        }
        if (w > 1) { // skip this if drawing a space
            s = pBits + u32Offset; // start of compressed bitmap data
            if (u32Rot == 0 || u32Rot == 180) {
                dx = x + xOffset; // offset from character UL to start drawing
                dy = y + yOffset;
            } else { // rotated
                dx = w; // swap height/width
                w = h;
                h = dx;
                n = yOffset; // offset from character UL to start drawing
                dx = x;
                if (-n < w) dx -= (w+n); // since we draw from the baseline
                dy = y + xOffset;
            }
            if ((dy + h) > pBBEP->height) { // trim it
                h = pBBEP->height - dy;
            }
            u8EndMask = 0xff;
            if (w & 7) { // width ends on a partial byte
                u8EndMask <<= (8-(w & 7));
            }
            end_y = dy + h;
            if (pBBF) {
                ty = (pgm_read_word(&pGlyph[1].bitmapOffset) - (intptr_t)(s - pBits)); // compressed size
            } else {
                ty = (pgm_read_word(&pSmallGlyph[1].bitmapOffset) - (intptr_t)(s - pBits)); // compressed size
            }
            if (ty < 0 || ty > 4096) ty = 4096; // DEBUG
            rc = g5_decode_init(&g5dec, w, h, s, ty);
            if (rc != G5_SUCCESS) {
                pBBEP->last_error = BBEP_ERROR_BAD_DATA;
                 return BBEP_ERROR_BAD_DATA; // corrupt data?
            }
            if (pBBEP->ucScreen) { // backbuffer, draw pixels
#ifndef NO_RAM
                tw = w;
                if (x+tw > pBBEP->width) tw = pBBEP->width - x; // clip to right edge
                for (ty=dy; ty<end_y && ty < pBBEP->height; ty++) {
                    uint8_t u8, u8Count;
                    g5_decode_line(&g5dec, u8Cache);
                    s = u8Cache;
                    u8 = *s++;
                    u8Count = 8;
                    if (ty >= 0) {
                        for (tx=x; tx<x+tw; tx++) {
                            if (u8 & 0x80) {
                                if (iColor != BBEP_TRANSPARENT) {
                                    (*pBBEP->pfnSetPixelFast)(pBBEP, tx, ty, iColor);
                                }
                            } else if (iBG != BBEP_TRANSPARENT) {
                                (*pBBEP->pfnSetPixelFast)(pBBEP, tx, ty, iBG);
                            }
                            u8 <<= 1;
                            u8Count--;
                            if (u8Count == 0) {
                                u8Count = 8;
                                u8 = *s++;
                            }
                        }
                    } // on the screen
                }
#endif // NO_RAM
            } else { // draw directly into EPD memory
                // set the memory window for this character
                iSrcPitch = (w+7)/8;
                w += (dx & 7); // add parital byte
                bbepSetAddrWindow(pBBEP, dx, dy, w, h);
                iPitch = (w+7)/8;
                // start writing into the correct plane
                if (pBBEP->chip_type == BBEP_CHIP_UC81xx) {
                    if (pBBEP->iFlags & BBEP_RED_SWAPPED) {
                        u8CMD1 = UC8151_DTM1;
                        u8CMD2 = UC8151_DTM2;
                    } else {
                        u8CMD1 = UC8151_DTM2;
                        u8CMD2 = UC8151_DTM1;
                    }
                } else {
                    u8CMD1 = SSD1608_WRITE_RAM;
                    u8CMD2 = SSD1608_WRITE_ALTRAM;
                }
                u8CMD = u8CMD1;
                if (iColor == BBEP_BLACK) bInvert = 1;
                else if (iColor == BBEP_RED && (pBBEP->iFlags & BBEP_3COLOR)) {
                    u8CMD = u8CMD2; // second plane is red (inverted)
                }
                if (!(pBBEP->iFlags & BBEP_3COLOR) && pBBEP->iPlane == 1) {
                    u8CMD = u8CMD2; // set second plane for partial updates
                }
                bbepWriteCmd(pBBEP, u8CMD); // memory write command
                for (ty=dy; ty<end_y && rc == G5_SUCCESS && ty < pBBEP->native_height; ty++) {
                    rc = g5_decode_line(&g5dec, u8Cache);
                    u8Cache[iSrcPitch-1] &= u8EndMask; // clean pixels beyond character width
                    u8Cache[iSrcPitch] = 0;
                    if (dx & 7) { // need to shift it over by 1-7 bits
                        s = u8Cache;
                        uc0 = 0; // last shifted byte
                        n = dx & 7; // shift amount
                        for (j=0; j<w+7; j+= 8) {
                            uc1 = *s;
                            uc0 |= (uc1 >> n);
                            *s++ = uc0;
                            uc0 = uc1 << (8-n);
                        }
                        *s++ = uc0; // store final byte
                        *s++ = 0; // and a zero for good measure
                    }
                    if (bInvert) InvertBytes(u8Cache, iPitch);
                    bbepWriteData(pBBEP, u8Cache, iPitch); // write each row into the EPD framebuffer
                } // for y
            }
        } // if not drawing a space
        if (u32Rot == 0 || u32Rot == 180) {
            x += xAdvance; // width of this character
        } else {
            y += xAdvance;
        }
    } // while drawing characters
    pBBEP->iCursorX = x;
    pBBEP->iCursorY = y;
    return BBEP_SUCCESS;
} /* EPDWriteStringCustom() */
//
// Rotate an 8x8 pixel block (8 bytes) by 90 degrees
// Used to draw the built-in font at 2 angles
//
void RotateCharBox(uint8_t *pSrc)
{
    int x, y;
    uint8_t ucDest[8], uc, ucSrcMask, ucDstMask;
    
    for (y=0; y<8; y++) {
        ucSrcMask = 1<<y;
        ucDstMask = 0x80;
        uc = 0;
        for (x=0; x<8; x++) {
            if (pSrc[x] & ucSrcMask) uc |= ucDstMask;
            ucDstMask >>= 1;
        }
        ucDest[y] = uc;
    }
    memcpy(pSrc, ucDest, 8); // rotate "in-place"
} /* RotateCharBox() */
//
// Double the size of a 1-bpp image and smooth the jaggies
//
void bbepStretchAndSmooth(uint8_t *pSrc, uint8_t *pDest, int w, int h, int iSmooth)
{
uint8_t c, uc1, uc2, *s, *d, ucMask;
int tx, ty, iPitch, iDestPitch;

    iPitch = (w+7)>>3;
    iDestPitch = ((w*2)+7)>>3;
// First, double the size of the source bitmap into the destination
    uc1 = uc2 = 0;
    memset(pDest, 0, iDestPitch * h*2);
    for (ty=0; ty<h; ty++) {
        s = &pSrc[ty * iPitch];
        d = &pDest[(ty*2) * iDestPitch];
        c = *s++;
        ucMask = 0xc0; // destination mask
        for (tx=0; tx<w; tx++) {
            if (c & 0x80) // a left-nibble bit is set
                uc1 |= ucMask;
            if (c & 0x08) // a right-nibble bit is set
                uc2 |= ucMask;
            ucMask >>= 2;
            c <<= 1;
            if (ucMask == 0) {
                c = *s++;
                d[0] = d[iDestPitch] = uc1;
                d[1] = d[iDestPitch+1] = uc2;
                d += 2;
                ucMask = 0xc0;
                tx += 4;
                uc1 = uc2 = 0;
            }
        } // for tx
    } // for ty
    if (iSmooth == BBEP_SMOOTH_NONE) return;
    // Now that the image is stretched, go through it and fill in corners
    // to smooth the blockiness
    for (ty = 0; ty < h; ty ++) {
        s = &pSrc[ty * iPitch];
        d = &pDest[ty*2 * iDestPitch];
        for (tx=0; tx<w-1; tx++) {
            uint8_t c00, c10, c01, c11; // 4 pixel group
            int dx = (tx*2)+1;
            c00 = s[tx>>3] & (0x80 >> (tx & 7));
            c10 = s[(tx+1)>>3] & (0x80 >> ((tx+1) & 7));
            c01 = s[iPitch + (tx>>3)] & (0x80 >> (tx & 7));
            c11 = s[iPitch + ((tx+1)>>3)] & (0x80 >> ((tx+1) & 7));
            if (iSmooth == BBEP_SMOOTH_HEAVY) {
                if (c00 && c11 && (!c10 || !c01)) {
                    // fill in the 'hole'
                    d[(dx>>3)+iDestPitch*2] |= (0x80 >> (dx & 7));
                    d[((dx+1)>>3) + iDestPitch] |= (0x80 >> ((dx+1) & 7));
                }
                if (c10 && c01 && (!c11 || !c00)) {
                    d[(dx>>3)+iDestPitch] |= (0x80 >> (dx & 7));
                    d[((dx+1)>>3)+iDestPitch*2] |= (0x80 >> ((dx+1) & 7));
                }
            } else { // BBEP_SMOOTH_LIGHT
                if ((c00 && !c10 && !c01 && c11) || (!c00 && c10 && c01 && !c11)) {
                    d[(dx>>3)+iDestPitch*2] |= (0x80 >> (dx & 7));
                    d[((dx+1)>>3) + iDestPitch] |= (0x80 >> ((dx+1) & 7));
                    d[(dx>>3)+iDestPitch] |= (0x80 >> (dx & 7));
                    d[((dx+1)>>3)+iDestPitch*2] |= (0x80 >> ((dx+1) & 7));
                }
            }
        } // for tx
    } // for ty
} /* bbepStretchAndSmooth() */
//
// Draw a string of normal (8x8), small (6x8) or large (16x32) characters
// At the given col+row
//
int bbepWriteString(BBEPDISP *pBBEP, int x, int y, char *szMsg, int iSize, int iColor, int iBG)
{
    int i, iFontOff, iLen;
    uint8_t c, *s, ucCMD, ucCMD1, ucCMD2;
    uint8_t u8Temp[40];
    
    if (pBBEP == NULL) {
        return BBEP_ERROR_BAD_PARAMETER;
    }
    if (iColor != BBEP_TRANSPARENT) {
        iColor = pBBEP->pColorLookup[iColor & 0xf];
    }
    if (iBG == -1) iBG = BBEP_TRANSPARENT; // -1 = don't care
    if (iBG != BBEP_TRANSPARENT) {
        iBG = pBBEP->pColorLookup[iBG & 0xf];
    }
    if (pBBEP->chip_type == BBEP_CHIP_UC81xx) {
        if (pBBEP->iFlags & BBEP_RED_SWAPPED) {
            ucCMD1 = UC8151_DTM1;
            ucCMD2 = UC8151_DTM2;
        } else {
            ucCMD1 = UC8151_DTM2;
            ucCMD2 = UC8151_DTM1;
        }
    } else {
        ucCMD1 = SSD1608_WRITE_RAM;
        ucCMD2 = SSD1608_WRITE_ALTRAM;
    }
    ucCMD = ucCMD1;
    if (((pBBEP->iFlags & BBEP_3COLOR) && iColor == BBEP_RED) || pBBEP->iPlane == 1) {
        ucCMD = ucCMD2; // write to second plane for red
    }
    if (x == -1 || y == -1) // use the cursor position
    {
        x = pBBEP->iCursorX; y = pBBEP->iCursorY;
    } else {
        pBBEP->iCursorX = x; pBBEP->iCursorY = y;
    }
    if (pBBEP->iCursorX >= pBBEP->width || pBBEP->iCursorY >= pBBEP->height) {
        pBBEP->last_error = BBEP_ERROR_BAD_PARAMETER;
        return BBEP_ERROR_BAD_PARAMETER; // can't draw off the display
    }
    
    if (iSize == FONT_8x8 || iSize == FONT_16x16) // 8x8 font (and stretched)
    {
        int iCount = (iSize == FONT_8x8) ? 8 : 16;
        i = 0;
        while (x < pBBEP->width && szMsg[i] != 0 && y < pBBEP->height)
        {
            c = (unsigned char)szMsg[i];
            iFontOff = (int)(c-32) * 7;
            // we can't directly use the pointer to FLASH memory, so copy to a local buffer
            u8Temp[0] = 0; // first column is blank
            memcpy_P(&u8Temp[1], &ucFont[iFontOff], 7); // only needed on AVR
            if (iColor == BBEP_BLACK && !pBBEP->ucScreen) InvertBytes(u8Temp, 8);
            iLen = 8;
            if (x + iLen > pBBEP->width) { // clip right edge
                iLen = pBBEP->width - x;
            }
            if (!pBBEP->ucScreen) { // bufferless mode, rotate the coordinate system to fit the situation
                if (pBBEP->iOrientation == 0) { // need to rotate the font and bounding box
                    RotateCharBox(u8Temp);
                    bbepSetAddrWindow(pBBEP, x, y, iCount, iCount);
                } else {
                    bbepSetAddrWindow(pBBEP, pBBEP->native_width-iCount-y, x, iCount, pBBEP->native_height-x);
                }
                bbepWriteCmd(pBBEP, ucCMD); // write to "new" plane
                if (iCount == 8) { // non-stretched
                    bbepWriteData(pBBEP, u8Temp, iCount);
                } else { // stretch the pixels 2x
                    uint8_t u8SrcMask, u8DstMask, u8_0, u8_1;
                    uint8_t *d = u8Cache;
                    for (int ty=0; ty<8; ty++) {
                        u8_0 = u8_1 = 0;
                        u8SrcMask = 0x80;
                        u8DstMask = 0xc0;
                        for (int tx=0; tx<4; tx++) {
                            if (u8Temp[ty] & u8SrcMask) {
                                u8_0 |= u8DstMask;
                            }
                            if (u8Temp[ty] & (u8SrcMask >> 4)) {
                                u8_1 |= u8DstMask;
                            }
                            u8SrcMask >>= 1;
                            u8DstMask >>= 2;
                        } // for tx
                        d[0] = d[2] = u8_0;
                        d[1] = d[3] = u8_1;
                        d += 4;
                    } // for ty
                    bbepWriteData(pBBEP, u8Cache, 32);
                } // stretched 2x
            } else { // draw in memory
#ifndef NO_RAM
                uint8_t *s, u8Mask;
                if (iCount == 8) {
                    for (int ty=0; ty<8; ty++) {
                        u8Mask = 1<<ty;
                        for (int tx = 0; tx<iLen; tx++) {
                            if (u8Temp[tx] & u8Mask) {
                                if (iColor != BBEP_TRANSPARENT) {
                                    (*pBBEP->pfnSetPixelFast)(pBBEP, x+tx, y+ty, iColor);
                                }
                            } else if (iBG != BBEP_TRANSPARENT) {
                                (*pBBEP->pfnSetPixelFast)(pBBEP, x+tx, y+ty, iBG);
                            }
                        }
                    }
                } else { // 16x16
                    bbepStretchAndSmooth(u8Temp, u8Cache, 8, 8, 1); // smooth too
                    for (int ty=0; ty<16; ty++) {
                        s = &u8Cache[2*ty];
                        u8Mask = 0x80;
                        for (int tx = 7; tx>=0; tx--) {
                            if (s[0] & u8Mask) {
                                if (iColor != BBEP_TRANSPARENT) {
                                    (*pBBEP->pfnSetPixelFast)(pBBEP, x+ty, y+tx+8, iColor);
                                }
                            } else if (iBG != BBEP_TRANSPARENT) {
                                (*pBBEP->pfnSetPixelFast)(pBBEP, x+ty, y+tx+8, iBG);
                            }
                            if (s[1] & u8Mask) {
                                if (iColor != BBEP_TRANSPARENT) {
                                    (*pBBEP->pfnSetPixelFast)(pBBEP, x+ty, y+tx, iColor);
                                }
                            } else if (iBG != BBEP_TRANSPARENT) {
                                (*pBBEP->pfnSetPixelFast)(pBBEP, x+ty, y+tx, iBG);
                            }
                            u8Mask >>= 1;
                        }
                    }
                }
#endif
            }
            x += iCount;
            if (x >= pBBEP->width-iCount-1 && pBBEP->wrap) { // word wrap enabled?
                x = 0; // start at the beginning of the next line
                y += iCount;
            }
            i++;
        } // while
        pBBEP->iCursorX = x;
        pBBEP->iCursorY = y;
        return BBEP_SUCCESS;
    } else if (iSize == FONT_12x16) { // 6x8 stretched to 12x16
        i = 0;
        while (pBBEP->iCursorX < pBBEP->width && pBBEP->iCursorY < pBBEP->height && szMsg[i] != 0) {
            // stretch the 'normal' font instead of using the big font
            int tx, ty;
            c = szMsg[i] - 32;
            unsigned char uc1, uc2, ucMask, *pDest;
            s = (unsigned char *)&ucSmallFont[(int)c*5];
            u8Temp[0] = 0; // first column is blank
            memcpy_P(&u8Temp[1], s, 6);
            // Stretch the font to double width + double height
            memset(&u8Temp[6], 0, 24); // write 24 new bytes
            for (tx=0; tx<6; tx++)
            {
                ucMask = 3;
                pDest = &u8Temp[6+tx*2];
                uc1 = uc2 = 0;
                c = u8Temp[tx];
                for (ty=0; ty<4; ty++)
                {
                    if (c & (1 << ty)) // a bit is set
                        uc1 |= ucMask;
                    if (c & (1 << (ty + 4)))
                        uc2 |= ucMask;
                    ucMask <<= 2;
                }
                pDest[0] = uc1;
                pDest[1] = uc1; // double width
                pDest[12] = uc2;
                pDest[13] = uc2;
            }
            // smooth the diagonal lines
            for (tx=0; tx<5; tx++)
            {
                uint8_t c0, c1, ucMask2;
                c0 = u8Temp[tx];
                c1 = u8Temp[tx+1];
                pDest = &u8Temp[6+tx*2];
                ucMask = 1;
                ucMask2 = 2;
                for (ty=0; ty<7; ty++)
                {
                    if (((c0 & ucMask) && !(c1 & ucMask) && !(c0 & ucMask2) && (c1 & ucMask2)) || (!(c0 & ucMask) && (c1 & ucMask) && (c0 & ucMask2) && !(c1 & ucMask2)))
                    {
                        if (ty < 3) // top half
                        {
                            if (iColor == BBEP_WHITE) {
                                pDest[1] &= ~(1 << ((ty * 2)+1));
                                pDest[2] &= ~(1 << ((ty * 2)+1));
                                pDest[1] &= ~(1 << ((ty+1) * 2));
                                pDest[2] &= ~(1 << ((ty+1) * 2));
                            } else {
                                pDest[1] |= (1 << ((ty * 2)+1));
                                pDest[2] |= (1 << ((ty * 2)+1));
                                pDest[1] |= (1 << ((ty+1) * 2));
                                pDest[2] |= (1 << ((ty+1) * 2));
                            }
                        }
                        else if (ty == 3) // on the border
                        {
                            if (iColor == BBEP_WHITE) {
                                pDest[1] &= ~0x80; pDest[2] &= ~0x80;
                                pDest[13] &= ~1; pDest[14] &= ~1;
                            } else {
                                pDest[1] |= 0x80; pDest[2] |= 0x80;
                                pDest[13] |= 1; pDest[14] |= 1;
                            }
                        }
                        else // bottom half
                        {
                            if (iColor == BBEP_WHITE) {
                                pDest[13] &= ~(1 << (2*(ty-4)+1));
                                pDest[14] &= ~(1 << (2*(ty-4)+1));
                                pDest[13] &= ~(1 << ((ty-3) * 2));
                                pDest[14] &= ~(1 << ((ty-3) * 2));
                            } else {
                                pDest[13] |= (1 << (2*(ty-4)+1));
                                pDest[14] |= (1 << (2*(ty-4)+1));
                                pDest[13] |= (1 << ((ty-3) * 2));
                                pDest[14] |= (1 << ((ty-3) * 2));
                            }
                        }
                    }
                    else if (!(c0 & ucMask) && (c1 & ucMask) && (c0 & ucMask2) && !(c1 & ucMask2))
                    {
                        if (ty < 4) // top half
                        {
                            if (iColor == BBEP_WHITE) {
                                pDest[1] &= ~(1 << ((ty * 2)+1));
                                pDest[2] &= ~(1 << ((ty+1) * 2));
                            } else {
                                pDest[1] |= (1 << ((ty * 2)+1));
                                pDest[2] |= (1 << ((ty+1) * 2));
                            }
                        }
                        else
                        {
                            if (iColor == BBEP_WHITE) {
                                pDest[13] &= ~(1 << (2*(ty-4)+1));
                                pDest[14] &= ~(1 << ((ty-3) * 2));
                            } else {
                                pDest[13] |= (1 << (2*(ty-4)+1));
                                pDest[14] |= (1 << ((ty-3) * 2));
                            }
                        }
                    }
                    ucMask <<= 1; ucMask2 <<= 1;
                }
            }
            iLen = 12;
            if (pBBEP->iCursorX + iLen > pBBEP->width) // clip right edge
                iLen = pBBEP->width - pBBEP->iCursorX;
            if (!pBBEP->ucScreen) { // bufferless mode
                bbepSetAddrWindow(pBBEP, pBBEP->native_width-8-pBBEP->iCursorY, pBBEP->iCursorX, 8, iLen);
                bbepWriteCmd(pBBEP, ucCMD); // write to "new" plane
                if (iColor == BBEP_BLACK) {
                    InvertBytes(&u8Temp[6], iLen);
                }
                bbepWriteData(pBBEP, &u8Temp[6], iLen);
                bbepSetAddrWindow(pBBEP, pBBEP->native_width-16-pBBEP->iCursorY, pBBEP->iCursorX, 8, iLen);
                bbepWriteCmd(pBBEP, ucCMD); // write to "new" plane
                if (iColor == BBEP_BLACK) {
                    InvertBytes(&u8Temp[18], iLen);
                }
                bbepWriteData(pBBEP, &u8Temp[18], iLen);
            } else { // write to RAM
#ifndef NO_RAM
                uint8_t u8Mask;
                for (int ty=0; ty<8; ty++) {
                    u8Mask = 1<<ty;
                    for (int tx = 0; tx<iLen; tx++) {
                        if (u8Temp[6+tx] & u8Mask) {
                            if (iColor != BBEP_TRANSPARENT) {
                                (*pBBEP->pfnSetPixelFast)(pBBEP, x+tx, y+ty, iColor);
                            }
                        } else if (iBG != BBEP_TRANSPARENT) {
                            (*pBBEP->pfnSetPixelFast)(pBBEP, x+tx, y+ty, iBG);
                        }
                        if (u8Temp[18+tx] & u8Mask) {
                            if (iColor != BBEP_TRANSPARENT) {
                                (*pBBEP->pfnSetPixelFast)(pBBEP, x+tx, y+ty+8, iColor);
                            }
                        } else if (iBG != BBEP_TRANSPARENT) {
                            (*pBBEP->pfnSetPixelFast)(pBBEP, x+tx, y+ty+8, iBG);
                        }
                    }
                }
#endif
            }
            x = pBBEP->iCursorX += iLen;
            if (pBBEP->iCursorX >= pBBEP->width-11 && pBBEP->wrap) // word wrap enabled?
            {
                x = pBBEP->iCursorX = 0; // start at the beginning of the next line
                y = pBBEP->iCursorY = y+16;
            }
            i++;
        } // while
        return BBEP_SUCCESS;
    } // 12x16
    else if (iSize == FONT_6x8)
    {
        i = 0;
        while (pBBEP->iCursorX < pBBEP->width && pBBEP->iCursorY < pBBEP->height && szMsg[i] != 0)
        {
            c = szMsg[i] - 32;
            // we can't directly use the pointer to FLASH memory, so copy to a local buffer
            u8Temp[0] = 0; // first column is blank
            memcpy_P(&u8Temp[1], &ucSmallFont[(int)c*5], 5);
            iLen = 6;
            if (pBBEP->iCursorX + iLen > pBBEP->width) // clip right edge
                iLen = pBBEP->width - pBBEP->iCursorX;
            if (!pBBEP->ucScreen) { // bufferless mode
                if (iColor == BBEP_BLACK) InvertBytes(u8Temp, 6);
                bbepSetAddrWindow(pBBEP, pBBEP->iCursorX, pBBEP->iCursorY, 8, iLen);
                bbepWriteCmd(pBBEP, ucCMD); // write to "new" plane
                bbepWriteData(pBBEP, u8Temp, iLen);
            } else { // write to RAM
#ifndef NO_RAM
                uint8_t u8Mask;
                for (int ty=0; ty<8; ty++) {
                    u8Mask = 1<<ty;
                    for (int tx = 0; tx<iLen; tx++) {
                        if (u8Temp[tx] & u8Mask) {
                            if (iColor != BBEP_TRANSPARENT) {
                                (*pBBEP->pfnSetPixelFast)(pBBEP, x+tx, ty+y, iColor);
                            }
                        } else if (iBG != BBEP_TRANSPARENT) {
                            (*pBBEP->pfnSetPixelFast)(pBBEP, x+tx, ty+y, iBG);
                        }
                    }
                }
#endif
            }
            pBBEP->iCursorX += iLen;
            if (pBBEP->iCursorX >= pBBEP->width-5 && pBBEP->wrap) // word wrap enabled?
            {
                x = pBBEP->iCursorX = 0; // start at the beginning of the next line
                y = pBBEP->iCursorY = y+8;
            }
            i++;
        }
        return BBEP_SUCCESS;
    } // 6x8
    pBBEP->last_error = BBEP_ERROR_BAD_PARAMETER;
    return BBEP_ERROR_BAD_PARAMETER; // invalid size
} /* bbepWriteString() */
//
// Get the width of text in a custom font
//
void bbepGetStringBox(BBEPDISP *pBBEP, BB_FONT *pFont, const char *szMsg, int *width, int *top, int *bottom)
{
    int cx = 0;
    unsigned int c, i = 0;
    BB_GLYPH *pBBG;
    int miny, maxy;
    
    if (!pBBEP) return;
    
    if (width == NULL || top == NULL || bottom == NULL || pFont == NULL || szMsg == NULL) {
        pBBEP->last_error = BBEP_ERROR_BAD_PARAMETER;
        return; // bad pointers
    }
    miny = 1000; maxy = 0;
    while (szMsg[i]) {
        c = szMsg[i++];
        if (c < pFont->first || c > pFont->last) // undefined character
            continue; // skip it
        c -= pFont->first; // first char of font defined
        pBBG = &pFont->glyphs[c];
        cx += pBBG->xAdvance;
        if (pBBG->yOffset < miny) miny = pBBG->yOffset;
        if (pBBG->height+pBBG->yOffset > maxy) maxy = pBBG->height+pBBG->yOffset;
    }
    *width = cx;
    *top = miny;
    *bottom = maxy;
} /* bbepGetStringBox() */

int bbepAllocBuffer(BBEPDISP *pBBEP, int bDoubleSize)
{
#ifndef NO_RAM
    int iSize;
    if (pBBEP->iFlags & (BBEP_7COLOR | BBEP_16GRAY)) { // 4-bpp
        iSize = (pBBEP->native_width >> 1) * pBBEP->native_height;
    } else { // B/W or B/W/R or B/W/R/Y
        iSize = ((pBBEP->native_width+7)>>3) * pBBEP->native_height;
        if (bDoubleSize) {
            iSize *= 2; // 2 bit planes
            pBBEP->iFlags |= BBEP_HAS_SECOND_PLANE;
        }
    }
#if defined (HAL_ESP32_HAL_H_) && !defined(__riscv)
    if (iSize > 98000) { // need to use PSRAM
//#ifndef BOARD_HAS_PSRAM
//#error "Please enable PSRAM!"
//#endif
        pBBEP->ucScreen = (uint8_t *)ps_malloc(iSize);
    } else {
        pBBEP->ucScreen = (uint8_t *)malloc(iSize);
    }
#else // not ESP32 or no PSRAM
    pBBEP->ucScreen = (uint8_t *)malloc(iSize);
#endif // ESP32
    if (pBBEP->ucScreen != NULL) {
        return BBEP_SUCCESS;
    }
#endif
    return BBEP_ERROR_NO_MEMORY; // failed
} /* bbepAllocBuffer() */
//
// Draw a line from x1,y1 to x2,y2 in the given color
// This function supports both buffered and bufferless drawing
// (bufferless is barely functional for 1-bit displays and should not be
// relied upon for full functionality).
//
void bbepDrawLine(BBEPDISP *pBBEP, int x1, int y1, int x2, int y2, uint8_t ucColor)
{
    int temp;
    int dx = x2 - x1;
    int dy = y2 - y1;
    int error;
    int xinc, yinc;
    
    if (pBBEP == NULL) {
        return;
    }
    if (x1 < 0 || x2 < 0 || y1 < 0 || y2 < 0 || x1 >= pBBEP->width || x2 >= pBBEP->width || y1 >= pBBEP->height || y2 >= pBBEP->height) {
        pBBEP->last_error = BBEP_ERROR_BAD_PARAMETER;
        return;
    }
    ucColor = pBBEP->pColorLookup[ucColor & 0xf];
    if(abs(dx) > abs(dy)) {
        // X major case
        if(x2 < x1) {
            dx = -dx;
            temp = x1;
            x1 = x2;
            x2 = temp;
            temp = y1;
            y1 = y2;
            y2 = temp;
        }
        
        dy = (y2 - y1);
        error = dx >> 1;
        yinc = 1;
        if (dy < 0)
        {
            dy = -dy;
            yinc = -1;
        }
        for(; x1 <= x2; x1++) {
            (*pBBEP->pfnSetPixelFast)(pBBEP, x1, y1, ucColor);
            error -= dy;
            if (error < 0) {
                error += dx;
                y1 += yinc;
            }
        } // for x1
    } else {
        // Y major case
        if(y1 > y2) {
            dy = -dy;
            temp = x1;
            x1 = x2;
            x2 = temp;
            temp = y1;
            y1 = y2;
            y2 = temp;
        }
        dx = (x2 - x1);
        error = dy >> 1;
        xinc = 1;
        if (dx < 0) {
            dx = -dx;
            xinc = -1;
        }
        for(; y1 <= y2; y1++) {
            (*pBBEP->pfnSetPixelFast)(pBBEP, x1, y1, ucColor);
            error -= dx;
            if (error < 0) {
                error += dy;
                x1 += xinc;
            }
        } // for y
    } // y major case
} /* bbepDrawLine() */

//
// For drawing ellipses, a circle is drawn and the x and y pixels are scaled by a 16-bit integer fraction
// This function draws a single pixel and scales its position based on the x/y fraction of the ellipse
//
static void DrawScaledPixel(BBEPDISP *pBBEP, int iCX, int iCY, int x, int y, int32_t iXFrac, int32_t iYFrac, uint8_t ucColor)
{
    if (iXFrac != 0x10000) x = ((x * iXFrac) >> 16);
    if (iYFrac != 0x10000) y = ((y * iYFrac) >> 16);
    x += iCX; y += iCY;
    if (x < 0 || x >= pBBEP->width || y < 0 || y >= pBBEP->height) {
        pBBEP->last_error = BBEP_ERROR_BAD_PARAMETER;
        return; // off the screen
    }
    (*pBBEP->pfnSetPixelFast)(pBBEP, x, y, ucColor);
} /* DrawScaledPixel() */
//
// For drawing filled ellipses
//
static void DrawScaledLine(BBEPDISP *pBBEP, int iCX, int iCY, int x, int y, int32_t iXFrac, int32_t iYFrac, uint8_t ucColor)
{
    int iLen, x2;
    
    if (iXFrac != 0x10000) x = ((x * iXFrac) >> 16);
    if (iYFrac != 0x10000) y = ((y * iYFrac) >> 16);
    iLen = x*2;
    x = iCX - x; y += iCY;
    x2 = x + iLen;
    if (y < 0 || y >= pBBEP->height) {
        pBBEP->last_error = BBEP_ERROR_BAD_PARAMETER;
        return; // completely off the screen
    }
    if (x < 0) x = 0;
    if (x2 >= pBBEP->width) x2 = pBBEP->width-1;
    bbepDrawLine(pBBEP, x, y, x2, y, ucColor);
} /* DrawScaledLine() */
//
// Draw the 8 pixels around the Bresenham circle
// (scaled to make an ellipse)
//
static void BresenhamCircle(BBEPDISP *pBBEP, int iCX, int iCY, int x, int y, int32_t iXFrac, int32_t iYFrac, uint8_t ucColor, uint8_t u8Parts, uint8_t bFill)
{
    if (bFill) // draw a filled ellipse
    {
        // for a filled ellipse, draw 4 lines instead of 8 pixels
        DrawScaledLine(pBBEP, iCX, iCY, x, y, iXFrac, iYFrac, ucColor);
        DrawScaledLine(pBBEP, iCX, iCY, x, -y, iXFrac, iYFrac, ucColor);
        DrawScaledLine(pBBEP, iCX, iCY, y, x, iXFrac, iYFrac, ucColor);
        DrawScaledLine(pBBEP, iCX, iCY, y, -x, iXFrac, iYFrac, ucColor);
    }
    else // draw 8 pixels around the edges
    {
        if (u8Parts & 1) {
            DrawScaledPixel(pBBEP, iCX, iCY, -x, -y, iXFrac, iYFrac, ucColor);
            DrawScaledPixel(pBBEP, iCX, iCY, -y, -x, iXFrac, iYFrac, ucColor);
        }
        if (u8Parts & 2) {
            DrawScaledPixel(pBBEP, iCX, iCY, x, -y, iXFrac, iYFrac, ucColor);
            DrawScaledPixel(pBBEP, iCX, iCY, y, -x, iXFrac, iYFrac, ucColor);
        }
        if (u8Parts & 4) {
            DrawScaledPixel(pBBEP, iCX, iCY, x, y, iXFrac, iYFrac, ucColor);
            DrawScaledPixel(pBBEP, iCX, iCY, y, x, iXFrac, iYFrac, ucColor);
        }
        if (u8Parts & 8) {
            DrawScaledPixel(pBBEP, iCX, iCY, -x, y, iXFrac, iYFrac, ucColor);
            DrawScaledPixel(pBBEP, iCX, iCY, -y, x, iXFrac, iYFrac, ucColor);
        }
    }
} /* BresenhamCircle() */

//
// Draw an outline or filled ellipse
//
void bbepEllipse(BBEPDISP *pBBEP, int iCenterX, int iCenterY, int32_t iRadiusX, int32_t iRadiusY, uint8_t u8Parts, uint8_t ucColor, uint8_t bFilled)
{
    int32_t iXFrac, iYFrac;
    int iRadius, iDelta, x, y;
    
    if (pBBEP == NULL) return;
    if (pBBEP->ucScreen == NULL && !bFilled) {
        pBBEP->last_error = BBEP_ERROR_BAD_PARAMETER;
        return; // must have back buffer defined for outline mode
    }
    
    if (iRadiusX <= 0 || iRadiusY <= 0) {
        pBBEP->last_error = BBEP_ERROR_BAD_PARAMETER;
        return; // invalid radii
    }
    ucColor = pBBEP->pColorLookup[ucColor & 0xf];
    if (iRadiusX > iRadiusY) {// use X as the primary radius
        iRadius = iRadiusX;
        iXFrac = 65536;
        iYFrac = (iRadiusY * 65536) / iRadiusX;
    } else {
        iRadius = iRadiusY;
        iXFrac = (iRadiusX * 65536) / iRadiusY;
        iYFrac = 65536;
    }
    iDelta = 3 - (2 * iRadius);
    x = 0; y = iRadius;
    while (x <= y) {
        BresenhamCircle(pBBEP, iCenterX, iCenterY, x, y, iXFrac, iYFrac, ucColor, u8Parts, bFilled);
        x++;
        if (iDelta < 0) {
            iDelta += (4*x) + 6;
        } else {
            iDelta += 4 * (x-y) + 10;
            y--;
        }
    }
} /* bbepEllipse() */
//
// Draw an outline or filled rectangle
//
void bbepRectangle(BBEPDISP *pBBEP, int x1, int y1, int x2, int y2, uint8_t ucColor, uint8_t bFilled)
{
    int tmp;
    
    if (pBBEP == NULL) {
        return; // invalid - must have BBEPDISP structure
    }
    ucColor = pBBEP->pColorLookup[ucColor & 0xf];

    if (x1 < 0 || y1 < 0 || x2 < 0 || y2 < 0 ||
        x1 >= pBBEP->width || y1 >= pBBEP->height || x2 >= pBBEP->width || y2 >= pBBEP->height) {
        pBBEP->last_error = BBEP_ERROR_BAD_PARAMETER;
        return; // invalid coordinates
    }
    // Make sure that X1/Y1 is above and to the left of X2/Y2
    // swap coordinates as needed to make this true
    if (x2 < x1)
    {
        tmp = x1;
        x1 = x2;
        x2 = tmp;
    }
    if (y2 < y1)
    {
        tmp = y1;
        y1 = y2;
        y2 = tmp;
    }
    if (bFilled)
    {
#ifndef NO_RAM
        if (pBBEP->ucScreen) { // has a buffer to fill
            int tx, ty;
            for (ty = y1; ty <= y2; ty++) {
                for (tx = x1; tx <= x2; tx++) {
                    (*pBBEP->pfnSetPixelFast)(pBBEP, tx, ty, ucColor);
                }
            }
        } else
#endif // NO_RAM
        { // no buffer
            int cx, cy, iPitch;
            cx = x2-x1+1;
            iPitch = (cx+7)/8;
            memset(u8Cache, (ucColor == BBEP_WHITE) ? 0xff:0x00, iPitch);
            bbepSetAddrWindow(pBBEP, x1, y1, cx, (y2-y1+1));
            bbepStartWrite(pBBEP, pBBEP->iPlane);
            for (cy = y1; cy <= y2; cy++) { // one line at a time
                bbepWriteData(pBBEP, u8Cache, iPitch);
            }
        }
    }
    else // outline (only support on backbuffer for now
    {
#ifndef NO_RAM
        if (pBBEP->ucScreen) { // has a buffer to fill
            int tx, ty;
            for (ty = y1; ty <= y2; ty++) {
                (*pBBEP->pfnSetPixelFast)(pBBEP, x1, ty, ucColor);
                (*pBBEP->pfnSetPixelFast)(pBBEP, x2, ty, ucColor);
            }
            for (tx = x1; tx <= x2; tx++) {
                (*pBBEP->pfnSetPixelFast)(pBBEP, tx, y1, ucColor);
                (*pBBEP->pfnSetPixelFast)(pBBEP, tx, y2, ucColor);
            }
        }
#endif
    } // outline
} /* bbepRectangle() */

void bbepRoundRect(BBEPDISP *pBBEP, int x, int y, int w, int h, int r, uint8_t iColor, int bFilled)
{
    if (pBBEP == NULL) return;
    if (bFilled) {
        bbepRectangle(pBBEP, x+r, y, x+w-1-r, y+h, iColor, 1);
        bbepRectangle(pBBEP, x, y+r, x+w-1, y+h-r, iColor, 1);
        // draw four corners
        bbepEllipse(pBBEP, x+w-r-1, y+r, r, r, 1, iColor, 1);
        bbepEllipse(pBBEP, x+r, y+r, r, r, 2, iColor, 1);
        bbepEllipse(pBBEP, x+w-r-1, y+h-r, r, r, 1, iColor, 1);
        bbepEllipse(pBBEP, x+r, y+h-r, r, r, 2, iColor, 1);
    } else {
        bbepDrawLine(pBBEP, x+r, y, x+w-r, y, iColor); // top
        bbepDrawLine(pBBEP, x+r, y+h-1, x+w-r, y+h-1, iColor); // bottom
        bbepDrawLine(pBBEP, x, y+r, x, y+h-r, iColor); // left
        bbepDrawLine(pBBEP, x+w-1, y+r, x+w-1, y+h-r, iColor); // right
        // four corners
        bbepEllipse(pBBEP, x+r, y+r, r, r, 1, iColor, 0);
        bbepEllipse(pBBEP, x+w-r-1, y+r, r, r, 2, iColor, 0);
        bbepEllipse(pBBEP, x+w-r-1, y+h-r-1, r, r, 4, iColor, 0);
        bbepEllipse(pBBEP, x+r, y+h-r-1, r, r, 8, iColor, 0);
    }
} /* bbepRoundRect() */

#endif // __BB_EP_GFX__

