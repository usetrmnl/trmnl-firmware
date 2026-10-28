#include <PNGdec.h>
#include <board_config.h>
#include <color_reduce.h>
#include <epd_driver.h>
#include <png_render.h>
#include <globals.h>
#include <trmnl_log.h>

enum {
    PNG_1_BIT = 0,
    PNG_1_BIT_INVERTED,
    PNG_2_BIT_0,
    PNG_2_BIT_1,
    PNG_2_BIT_BOTH,
    PNG_2_BIT_INVERTED,
};
#ifdef BB_EPAPER

/**
 * @brief Callback function for each line of PNG decoded
 * @param PNGDRAW structure containing the current line and relevant info
 * @return none
 */
//
// Draw the PNG image into the local framebuffer memory using the drawPixel() method
// to do color translation and to properly format the memory layout
//
int png_draw_6clr(PNGDRAW *pDraw)
{
    uint8_t r=0, g=0, b=0, *s, *pPal, *pPalette = pDraw->pPalette;
    int x, y, iWidth, iDelta, iBpp = pDraw->iBpp;
    bool bRotated = (pDraw->iWidth == bbep.height() && bbep.width() != bbep.height());

    y = pDraw->y;
    if (bRotated) {
        iWidth = pDraw->iWidth;
        if (iWidth > bbep.height()) iWidth = bbep.height(); // crop to width of EPD
        if (y >= bbep.width()) return 0; // past the bottom of the EPD; stop decoding
    } else {
        iWidth = pDraw->iWidth;
        if (iWidth > bbep.width()) iWidth = bbep.width(); // crop to width of EPD
        if (y >= bbep.height()) return 0;
    }

    switch (pDraw->iPixelType) {
        case PNG_PIXEL_INDEXED:
            break;
        case PNG_PIXEL_TRUECOLOR:
	        if (iBpp <= 8) {
                iBpp *= 3;
	        }
            pPalette = NULL;
            break;
        case PNG_PIXEL_TRUECOLOR_ALPHA:
	        if (iBpp <= 8) {
                iBpp *= 4;
	        }
            pPalette = NULL;
            break;
        case PNG_PIXEL_GRAYSCALE:
            pPalette = NULL;
            break;
    } // switch on pixel type
    iDelta = iBpp/8;
    s = pDraw->pPixels;
    for (x=0; x<iWidth; x++) { // slower code, but less code :)
        switch (iBpp) {
            case 24:
            case 32:
                r = s[0];
                g = s[1];
                b = s[2];
                s += iDelta;
                break;
            case 16:
                r = s[1] & 0xf8; // red
                g = ((s[0] | s[1] << 8) >> 3) & 0xfc; // green
                b = s[0] << 3;
                s += 2;
                break;
                case 8:
                    if (pPalette) {
                        pPal = &pPalette[s[0] * 3];
                        r = pPal[0];
                        g = pPal[1];
                        b = pPal[2];
                    } else {
                        r = g = b = s[0];
                    }
                    s++;
                    break;
                case 4:
                    if (pPalette) {
                        if (x & 1) {
                            pPal = &pPalette[(s[0] & 0xf) * 3];
                            s++;
                        } else {
                            pPal = &pPalette[(s[0]>>4) * 3];
                        }
                        r = pPal[0];
                        g = pPal[1];
                        b = pPal[2];
                    } else {
                        if (x & 1) {
                            r = g = b = (s[0] & 0xf) | (s[0] << 4);
                            s++;
                        } else {
                            r = g = b = (s[0] >> 4) | (s[0] & 0xf0);
                        }
                    }
                    break;
		case 2:
		    if (pPalette) {
			pPal = &pPalette[((s[0] >> ((3-(x&3))*2)) & 3) * 3];
			r = pPal[0]; g = pPal[1]; b = pPal[2];
		    } else {
			r = g = b = (s[0] << ((x&3)*2)) & 0xc0;
		    }
		    if ((x & 3) == 3) s++;
		    break;
                case 1:
                    if (pPalette) {
                        pPal = &pPalette[((s[0] >> (7-(x&7))) & 1) * 3];
                        r = pPal[0]; g = pPal[1]; b = pPal[2];
                    } else {
                        r = g = b = ((s[0] << (x&7)) & 0x80);
                    }
                    if ((x & 7) == 7) s++;
                    break;
            } // switch on bpp
            if (bRotated) {
                bbep.drawPixel(bbep.width()-1-y, x, GetSpectraPixel(r, g, b));
            } else {
                bbep.drawPixel(x, y, GetSpectraPixel(r, g, b));
            }
        } // for x
    return 1; // continue decoding
} /* png_draw_6clr() */

#if defined( BOARD_TRMNL_4CLR ) || defined( BOARD_TRMNL_GEN2_4CLR )
//
// Draw the PNG image into the local framebuffer memory using the drawPixel() method
// to do color translation and to properly format the memory layout
//
int png_draw_4clr(PNGDRAW *pDraw)
{
    uint8_t r=0, g=0, b=0, *s, *pPal, *pPalette = pDraw->pPalette;
    int x, iDelta, iBpp = pDraw->iBpp;
    int iWidth;
    uint8_t uc=0, *d, *pTemp = bbep.getCache(); // get some scratch memory (not from the stack)

    d = pTemp;
    iWidth = pDraw->iWidth;
    if (iWidth > bbep.width()) iWidth = bbep.width(); // Crop to EPD width if larger
    if (pDraw->y >= bbep.height()) return 0; // Past the bottom of the EPD; stop decoding

    switch (pDraw->iPixelType) {
        case PNG_PIXEL_INDEXED:
            break;
        case PNG_PIXEL_TRUECOLOR:
	        if (iBpp <= 8) {
                iBpp *= 3;
	        }
            pPalette = NULL;
            break;
        case PNG_PIXEL_TRUECOLOR_ALPHA:
	        if (iBpp <= 8) {
                iBpp *= 4;
	        }
            pPalette = NULL;
            break;
        case PNG_PIXEL_GRAYSCALE:
            pPalette = NULL;
            break;
    } // switch on pixel type
    iDelta = iBpp/8;
    s = pDraw->pPixels;
    for (x=0; x<iWidth; x++) { // slower code, but less code :)
        switch (iBpp) {
            case 24:
            case 32:
                r = s[0];
                g = s[1];
                b = s[2];
                s += iDelta;
                break;
            case 16:
                r = s[1] & 0xf8; // red
                g = ((s[0] | s[1] << 8) >> 3) & 0xfc; // green
                b = s[0] << 3;
                s += 2;
                break;
                case 8:
                    if (pPalette) {
                        pPal = &pPalette[s[0] * 3];
                        r = pPal[0];
                        g = pPal[1];
                        b = pPal[2];
                    } else {
                        r = g = b = s[0];
                    }
                    s++;
                    break;
                case 4:
                    if (pPalette) {
                        if (x & 1) {
                            pPal = &pPalette[(s[0] & 0xf) * 3];
                            s++;
                        } else {
                            pPal = &pPalette[(s[0]>>4) * 3];
                        }
                        r = pPal[0];
                        g = pPal[1];
                        b = pPal[2];
                    } else {
                        if (x & 1) {
                            r = g = b = (s[0] & 0xf) | (s[0] << 4);
                            s++;
                        } else {
                            r = g = b = (s[0] >> 4) | (s[0] & 0xf0);
                        }
                    }
                    break;
                case 2:
                    if (pPalette) {
                    pPal = &pPalette[((s[0] >> ((3-(x&3))*2)) & 3) * 3];
                    r = pPal[0]; g = pPal[1]; b = pPal[2];
                    } else {
                    r = g = b = (s[0] << ((x&3)*2)) & 0xc0;
                    }
                    if ((x & 3) == 3) s++;
                    break;
                case 1:
                    if (pPalette) {
                        pPal = &pPalette[((s[0] >> (7-(x&7))) & 1) * 3];
                        r = pPal[0]; g = pPal[1]; b = pPal[2];
                    } else {
                        r = g = b = ((s[0] << (x&7)) & 0x80);
                    }
                    if ((x & 7) == 7) s++;
                    break;
            } // switch on bpp
            uc <<= 2;
            uc |= GetBWYRPixel(r, g, b); // get the best matching 2-bit color
            if ((x & 3) == 3) { // 4 pixels packed into each byte
                *d++ = uc;
            }
        } // for x
    bbep.writeData(pTemp, (iWidth+3)/4);
    if (iWidth < bbep.width()) {
        // fill the unused area of the display with white
        x = ((bbep.width()+3)/4) - ((iWidth+3)/4); // number of missing bytes
        d = bbep.getCache();
        memset(d, 0x55, x); // 0x55 = 4 pixels of white (01)
        bbep.writeData(d, x);
    }
    return 1; // continue decoding
} /* png_draw4clr() */
#endif // BOARD_TRMNL_4CLR (4 color only)

int png_draw(PNGDRAW *pDraw)
{
    int x;
    uint8_t ucBppChanged = 0, ucInvert = 0;
    uint8_t uc, ucMask, src, *s, *d, *pTemp = bbep.getCache(); // get some scratch memory (not from the stack)
    int iPlane = *(int *)pDraw->pUser;
    int iWidth;

    iWidth = pDraw->iWidth;
    if (pDraw->y >= bbep.height()) return 0; // stop decoding if we'll go past the bottom
    if (iWidth > bbep.width()) iWidth = bbep.width(); // crop image width to display size if it's larger

    if (pDraw->iPixelType == PNG_PIXEL_INDEXED || pDraw->iBpp > 2) {
        if (pDraw->iBpp == 1) { // 1-bit output, just see which color is brighter
            uint32_t u32Gray0, u32Gray1;
            u32Gray0 = pDraw->pPalette[0] + (pDraw->pPalette[1]<<2) + pDraw->pPalette[2];
            u32Gray1 = pDraw->pPalette[3] + (pDraw->pPalette[4]<<2) + pDraw->pPalette[5];
            if (u32Gray0 > u32Gray1) { // index 0 is the brighter color; output needs 0 = black
                ucInvert = 0xff;
            }
        } else {
            // Reduce the source image to 1-bpp or 2-bpp
            ReduceBpp((pDraw->pUser) ? 2:1, pDraw->iPixelType, pDraw->pPalette, pDraw->pPixels, pTemp, iWidth, pDraw->iBpp);
            ucBppChanged = 1;
        }
    } else if (pDraw->iBpp == 2) {
        ucInvert = 0xff; // 2-bit non-palette images need to be inverted colors for 4-gray mode
    }
    s = (ucBppChanged) ? pTemp : (uint8_t *)pDraw->pPixels;
    d = pTemp;
    if (iPlane == PNG_1_BIT || iPlane == PNG_1_BIT_INVERTED) {
        // 1-bit output, decode the single plane and write it
        if (iPlane == PNG_1_BIT_INVERTED) ucInvert = ~ucInvert; // to do PLANE_FALSE_DIFF
        if (iPlane == PNG_1_BIT_INVERTED && (bbep.capabilities() & BBEP_3COLOR)) { // write the red plane as 0's for this case
            memset(d, 0, iWidth/8);
        } else {
            for (x=0; x<iWidth; x+= 8) {
                d[0] = s[0] ^ ucInvert;
                d++; s++;
            }
        }
    } else { // we need to split the 2-bit data into plane 0 and 1
        src = *s++;
        src ^= ucInvert;
        uc = 0; // suppress warning/error
        if (iPlane == PNG_2_BIT_BOTH || iPlane == PNG_2_BIT_INVERTED) { // draw 2bpp data as 1-bit to use for partial update
            if (iPlane == PNG_2_BIT_BOTH) {
                ucInvert = ~ucInvert; // the invert rule is backwards for grayscale data
            }
            src = ~src;
            for (x=0; x<iWidth; x++) {
                uc <<= 1;
                if (src & 0xc0) { // non-white -> black
                    uc |= 1; // high bit of source pair
                }
                src <<= 2;
                if ((x & 3) == 3) { // new input byte
                    src = *s++;
                    src ^= ucInvert;
                }
                if ((x & 7) == 7) { // new output byte
                    *d++ = uc;
                }
            } // for x
        } else { // normal 0/1 split plane
            ucMask = (iPlane == PNG_2_BIT_0) ? 0x40 : 0x80; // lower or upper source bit
            for (x=0; x<iWidth; x++) {
                uc <<= 1;
                if (src & ucMask) {
                    uc |= 1; // high bit of source pair
                }
                src <<= 2;
                if ((x & 3) == 3) { // new input byte
                    src = *s++;
                    src ^= ucInvert;
                }
                if ((x & 7) == 7) { // new output byte
                    *d++ = uc;
                }
            } // for x
        }
    }
    bbep.writeData(pTemp, (iWidth+7)/8);
    return 1;
} /* png_draw() */
#else // TRMNL_X version
// The generic 2-bit gray update procedure is too dark on the X's 10.3" panel, so
// we need to use the more complex 4-bit update method which has a custom pattern
// specifically for that panel's characteristics. This loop maps 2-bit pixels to
// 4-bit pixels by converting the values 0/1/2/3 to 0/5/10/15
static void Expand2bppLineTo4bpp(const uint8_t *src, uint8_t *dest, int width)
{
    uint8_t b;
    uint16_t u16;

    for (int x = 0; x < width; x+=4) {
        u16 = 0;
        b = *src++;
        for (int j=0; j<4; j++) {
            u16 <<= 4;
            u16 |= (((b >> 6) & 3) * 5);
            b <<= 2;
        } // for j
      *dest++ = (uint8_t)(u16 >> 8);
      *dest++ = (uint8_t)(u16 & 0xff);
    } // for x
} /* Expand2bppLineTo4bpp() */

int png_draw(PNGDRAW *pDraw)
{
    int x, y = pDraw->y;
    uint8_t uc = 0;
    uint8_t ucMask, ucPixel, src, *s, *d;
    int iPitch, iBpp;

    if (y >= bbep.height() && pDraw->iWidth != bbep.height()) return 0; // image is larger than the display (and not rotated), stop decoding it
    if (pDraw->iPixelType == PNG_PIXEL_INDEXED || pDraw->iBpp > 4) { // need to convert through the palette and/or reduce the bpp
        s = bbep.tempBuffer(); // temp space we can use
        iBpp = (pDraw->iBpp > 4) ? 4 : pDraw->iBpp;
        if (iBpp == 2) iBpp = 4; // 2-bit indexed images -> 4bpp for calibrated gray refresh
        ReduceBpp(iBpp, pDraw->iPixelType, pDraw->pPalette, pDraw->pPixels, s, pDraw->iWidth, pDraw->iBpp);
    } else { // for grayscale images of 1/2/4-bpp we can directly use the pixels as-is
        iBpp = pDraw->iBpp;
        s = (uint8_t *)pDraw->pPixels;
    }
    if (pDraw->pUser) { // drawing previous image into previous buffer
        d = bbep.previousBuffer();
        switch (iBpp) { // if this matches the new image we can do a non-flickering update
            case 1:
                bbep.setPreviousMode(BB_MODE_1BPP);
                break;
            default:
                bbep.setPreviousMode(BB_MODE_4BPP);
                break;
        }
    } else {
        d = bbep.currentBuffer();
    }
    iPitch = bbep.width()/2;
    if (iBpp == 1) {
        if (bbep.width() >= pDraw->iWidth) { // normal orientation
            iPitch = (bbep.width() + 7)/8;
            d += y * iPitch; // point to the correct line
            memcpy(d, s, (pDraw->iWidth+7)/8);
        } else { // rotated
            uint8_t ucPixel, ucMask, j;
            d += (bbep.height() - 1) * iPitch;
            d += (y / 8);
            ucMask = 0x80 >> (y & 7); // destination mask
            for (x=0; x<pDraw->iWidth; x++) {
                if ((x & 7) == 0) uc = *s++;
                ucPixel = d[0] & ~ucMask; // unset old pixel
                if (uc & 0x80) ucPixel |= ucMask;
                d[0] = ucPixel;
                uc <<= 1;
                d -= iPitch;
            }
        }
    } else {
        if (iBpp == 2) { // expand 2-bit grayscale to 4-bit for calibrated refresh
            Expand2bppLineTo4bpp(s, bbep.tempBuffer(), pDraw->iWidth);
            s = bbep.tempBuffer();
            iBpp = 4;
        }
        // 4-bit native format (includes expanded 2-bit)
        if (bbep.width() == pDraw->iWidth) { // normal orientation
            d += y * iPitch; // point to the correct line
            memcpy(d, s, (pDraw->iWidth+1)/2);
        } else { // rotated
            d += (bbep.height() - 1) * iPitch;
            d += (y / 2);
            if (y & 1) { // odd line (column)
                for (x=0; x<pDraw->iWidth; x+=2) {
                    uc = (d[0] & 0xf0) | (s[0] >> 4);
                    *d = uc;
                    d -= iPitch;
                    uc = (d[0] & 0xf0) | (s[0] & 0xf);
                    *d = uc;
                    d -= iPitch;
                    s++;
                } // for x
            } else {
                for (x=0; x<pDraw->iWidth; x+=2) {
                    uc = (d[0] & 0xf) | (s[0] & 0xf0);
                    *d = uc;
                    d -= iPitch;
                    uc = (d[0] & 0xf) | (s[0] << 4);
                    *d = uc;
                    d -= iPitch;
                    s++;
                } // for x
            }
        }
    }
    return 1;
} /* png_draw() */
#endif
//
// A table to accelerate the testing of 2-bit images for the number
// of unique colors. Each entry sets bits 0-3 depending on the presence
// of colors 0-3 in each 2-bit pixel
//
const uint8_t ucTwoBitFlags[256] = {
0x01,0x03,0x05,0x09,0x03,0x03,0x07,0x0b,0x05,0x07,0x05,0x0d,0x09,0x0b,0x0d,0x09,
0x03,0x03,0x07,0x0b,0x03,0x03,0x07,0x0b,0x07,0x07,0x07,0x0f,0x0b,0x0b,0x0f,0x0b,
0x05,0x07,0x05,0x0d,0x07,0x07,0x07,0x0f,0x05,0x07,0x05,0x0d,0x0d,0x0f,0x0d,0x0d,
0x09,0x0b,0x0d,0x09,0x0b,0x0b,0x0f,0x0b,0x0d,0x0f,0x0d,0x0d,0x09,0x0b,0x0d,0x09,
0x03,0x03,0x07,0x0b,0x03,0x03,0x07,0x0b,0x07,0x07,0x07,0x0f,0x0b,0x0b,0x0f,0x0b,
0x03,0x03,0x07,0x0b,0x03,0x02,0x06,0x0a,0x07,0x06,0x06,0x0e,0x0b,0x0a,0x0e,0x0a,
0x07,0x07,0x07,0x0f,0x07,0x06,0x06,0x0e,0x07,0x06,0x06,0x0e,0x0f,0x0e,0x0e,0x0e,
0x0b,0x0b,0x0f,0x0b,0x0b,0x0a,0x0e,0x0a,0x0f,0x0e,0x0e,0x0e,0x0b,0x0a,0x0e,0x0a,
0x05,0x07,0x05,0x0d,0x07,0x07,0x07,0x0f,0x05,0x07,0x05,0x0d,0x0d,0x0f,0x0d,0x0d,
0x07,0x07,0x07,0x0f,0x07,0x06,0x06,0x0e,0x07,0x06,0x06,0x0e,0x0f,0x0e,0x0e,0x0e,
0x05,0x07,0x05,0x0d,0x07,0x06,0x06,0x0e,0x05,0x06,0x04,0x0c,0x0d,0x0e,0x0c,0x0c,
0x0d,0x0f,0x0d,0x0d,0x0f,0x0e,0x0e,0x0e,0x0d,0x0e,0x0c,0x0c,0x0d,0x0e,0x0c,0x0c,
0x09,0x0b,0x0d,0x09,0x0b,0x0b,0x0f,0x0b,0x0d,0x0f,0x0d,0x0d,0x09,0x0b,0x0d,0x09,
0x0b,0x0b,0x0f,0x0b,0x0b,0x0a,0x0e,0x0a,0x0f,0x0e,0x0e,0x0e,0x0b,0x0a,0x0e,0x0a,
0x0d,0x0f,0x0d,0x0d,0x0f,0x0e,0x0e,0x0e,0x0d,0x0e,0x0c,0x0c,0x0d,0x0e,0x0c,0x0c,
0x09,0x0b,0x0d,0x09,0x0b,0x0a,0x0e,0x0a,0x0d,0x0e,0x0c,0x0c,0x09,0x0a,0x0c,0x08
};

int png_draw_count(PNGDRAW *pDraw)
{
    int x, *pFlags = (int *)pDraw->pUser;
    uint8_t *s, set_bits;

    if (pDraw->y > 430) return 0; // Workaround to ignore the icon in the lower left corner

    set_bits = pFlags[0]; // use a local var
    s = (uint8_t *)pDraw->pPixels;
    for (x=0; x<pDraw->iWidth; x+=4) {
        set_bits |= ucTwoBitFlags[*s++]; // do 4 pixels at a time
    } // for x
    pFlags[0] = set_bits; // put it back in the flags array
    return 1;
} /* png_draw_count() */
/**
 * @brief Function to decode a PNG and count the number of unique colors
 *        This is needed because 2-bit (4gray) images can sometimes contain
 *        only 2 unique colors. This will allow us to use partial (non-flickering)
 *        updates on these images.
 * @param pointer to the PNG class instance
 * @param pointer to the buffer holding the PNG file
 * @param size of the PNG file
 * @return the number of unique colors in the image (2 to 4)
 */
int png_count_colors(PNG *png, const uint8_t *pData, int iDataSize)
{
int i, iColors;
    png->openRAM((uint8_t *)pData, iDataSize, png_draw_count);
    i = 0;
    png->decode(&i, 0);
    png->close();
    iColors = 0;
    if (i & 1) iColors++;
    if (i & 2) iColors++;
    if (i & 4) iColors++;
    if (i & 8) iColors++;
    Log_info("%s [%d]: png_count_colors: %d\r\n", __FILE__, __LINE__, iColors);
    return iColors;
} /* png_count_colors() */
/**
 * @brief Function to decode and display a PNG image from memory
 *        The decoded lines are written directly into the EPD framebuffer
 *        due to insufficient RAM to hold the fully decoded image
 * @param pointer to the buffer holding the PNG file
 * @param size of the PNG file
 * @return refresh mode based on image type and presence of old image
 */
int png_to_epd(const uint8_t *pPNG, int iDataSize, bool bPrevious)
{
int iPlane = PNG_1_BIT, rc = -1;
PNG *png = new PNG();

#ifndef BB_EPAPER
    if (bPrevious && bbep.getPreviousMode() != BB_MODE_NONE) return 0; // no need to decode previous image, we drew a msg/glyph
#endif

    if (!png) return PNG_MEM_ERROR; // not enough memory for the decoder instance
    rc = png->openRAM((uint8_t *)pPNG, iDataSize, png_draw);
    png->close();
    if (rc == PNG_SUCCESS) {
        Log_info("Decoding %d x %d PNG", png->getWidth(), png->getHeight());
        if (png->getWidth() == bbep.height() && png->getHeight() == bbep.width()) {
            Log_info("Rotating canvas to portrait orientation");
        } else if (png->getWidth() > bbep.width() || png->getHeight() > bbep.height()) {
            Log_info("PNG image is larger than the display (%dx%d), it will be cropped", png->getWidth(), png->getHeight());
        }
        if (rc == PNG_SUCCESS) { // okay to decode
            Log_info("%s [%d]: Decoding %d-bpp png (current)\r\n", __FILE__, __LINE__, png->getBpp());
            // Prepare target memory window (entire display)
#ifdef BB_EPAPER
            if (bbep.capabilities() & BBEP_7COLOR) { // Spectra6 panels
                CreateSpectra6Pal(); // create a fast color matching palette
                if (bbep.allocBuffer() != BBEP_SUCCESS) {
                    Log_error("%s [%d]: bbep.AllocBuffer failed!\n\r", __FILE__, __LINE__);
                    return -1;
                }
                bbep.fillScreen(BBEP_WHITE); // in case the image is too small - undrawn areas will be white
                Log_info("%s [%d]: decoding for 6-color EPD\r\n", __FILE__, __LINE__);
                png->openRAM((uint8_t *)pPNG, iDataSize, png_draw_6clr);
                png->decode(NULL, 0);
                png->close();
                delete(png); // free the decoder instance
                bbep.writePlane(); // send the pixels to the display panel
                return REFRESH_FAST;
            }
#if defined( BOARD_TRMNL_4CLR ) || defined( BOARD_TRMNL_GEN2_4CLR )
            Log_info("%s [%d]: decoding for 4-color EPD\r\n", __FILE__, __LINE__);
            png->openRAM((uint8_t *)pPNG, iDataSize, png_draw_4clr);
            bbep.startWrite(PLANE_1); // start writing image data
            png->decode(NULL, 0);
            png->close();
            if (png->getHeight() < bbep.height()) {
                // fill in missing lines with white
                uint8_t *d = bbep.getCache();
                int y, iLen = (bbep.width()+3)/4;
                memset(d, 0x55, iLen); // 4 pixels of white (01) per byte
                for (y=png->getHeight(); y<bbep.height(); y++) {
                    bbep.writeData(d, iLen);
                }
            }
            delete(png); // free the decoder instance
            return REFRESH_FULL;
#endif // BOARD_TRMNL_4CLR
            bbep.setAddrWindow(0, 0, bbep.width(), bbep.height());
            if (png->getBpp() == 1 || (png->getBpp() == 2 && png_count_colors(png, pPNG, iDataSize) == 2)) { // 1-bit image (single plane)
                png->close(); // use a different PNGDraw callback for color matching
                bbep.setPanelType(dpList[pDevice->panel_set][iTempProfile].OneBit);
                rc = REFRESH_PARTIAL; // the new image is 1bpp - try a partial update
                bbep.startWrite(PLANE_0); // start writing image data to plane 0
                png->openRAM((uint8_t *)pPNG, iDataSize, png_draw);
                if (png->getBpp() == 1 || png->getBpp() > 2) {
                    iPlane = PNG_1_BIT;
                    png->decode(&iPlane, 0);
                } else { // convert the 2-bit image to 1-bit output
                    Log_info("%s [%d]: Current png only has 2 unique colors!\n", __FILE__, __LINE__);
                    iPlane = PNG_2_BIT_BOTH;
                    if (png->decode(&iPlane, 0) != PNG_SUCCESS) {
                        Log_info("%s [%d]: Error decoding image = %d\n", __FILE__, __LINE__, png->getLastError());
                    }
                }
                png->close();
                if (bbep.getPanelType() != EP75_800x480) { // need to write the inverted plane to do PLANE_FALSE_DIFF
                    bbep.startWrite(PLANE_1); // start writing image data to plane 1
                    png->openRAM((uint8_t *)pPNG, iDataSize, png_draw);
                    if (iPlane == PNG_1_BIT) {
                        iPlane = PNG_1_BIT_INVERTED; // inverted 1-bit to second memory plane
                    } else { // convert the 2-bit image to 1-bit output
                        iPlane = PNG_2_BIT_INVERTED; // inverted 2-bit -> 1-bit to second plane
                    }
                    png->decode(&iPlane, 0);
                } // temp profile needs the second plane written
            } else { // 2-bpp (or greater, but reduced to 2-bpp)
                if (pDevice->epd_mosi_pin != 0 || pDevice->epd_sck_pin != 0) {
                    bbep.setPanelType(dpList[pDevice->panel_set][iTempProfile].TwoBit);
                    bbep.initIO(pDevice->epd_dc_pin, pDevice->epd_rst_pin, pDevice->epd_busy_pin, pDevice->epd_cs_pin,
                    pDevice->epd_mosi_pin, pDevice->epd_sck_pin, 8000000);
                } else {
                    bbep.begin(dpList[pDevice->panel_set][0].TwoBit);
                }
                rc = REFRESH_FULL; // 4gray mode must be full refresh
                iUpdateCount = 0; // grayscale mode resets the partial update counter
                bbep.startWrite(PLANE_0); // start writing image data to plane 0
                iPlane = PNG_2_BIT_0;
                Log_info("%s [%d]: decoding 4-gray plane 0\r\n", __FILE__, __LINE__);
                png->openRAM((uint8_t *)pPNG, iDataSize, png_draw);
                png->decode(&iPlane, 0); // tell PNGDraw to use bits for plane 0
                png->close(); // start over for plane 1
                iPlane = PNG_2_BIT_1;
                Log_info("%s [%d]: decoding 4-gray plane 1\r\n", __FILE__, __LINE__);
                png->openRAM((uint8_t *)pPNG, iDataSize, png_draw);
                bbep.startWrite(PLANE_1); // start writing image data to plane 1
                png->decode(&iPlane, 0); // decode it again to get plane 1 data
            }
#else // FastEPD
            switch (png->getBpp()) {
                case 1:
                    bbep.setMode(BB_MODE_1BPP);
                break;
                case 2:
                    // 2-bit PNGs are expanded to 4bpp in png_draw() so refresh uses u8_graytable
                    bbep.setMode(BB_MODE_4BPP);
                break;
                default:
                    bbep.setMode(BB_MODE_4BPP);
                break;
            }
            Log_info("%s [%d]: FastEPD graphics mode set to: %d\n", __FILE__, __LINE__, bbep.getMode());
            png->decode((void *)bPrevious, 0);
            png->close();
#endif
        }
    } else {
        Log_error("%s [%d]: png->openRAM() returned %d", __FILE__, __LINE__, rc);
    }
    delete(png); // free the decoder instance
    return rc;
} /* png_to_epd() */
