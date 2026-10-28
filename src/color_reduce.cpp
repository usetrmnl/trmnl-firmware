#include <PNGdec.h>
#include <color_reduce.h>
#include <epd_driver.h>

/**
 * @brief Reduce the bit depth of line of pixels using thresholding (aka simple color mapping)
 * @param Destination bit count (1 or 2)
 * @param Pointer to a PNG palette (3 bytes per entry)
 * @param Pointer to the source pixels
 * @param Pointer to the destination pixels
 * @param Pixel count
 * @param Original bit depth
 * @return none
 */
void ReduceBpp(int iDestBpp, int iPixelType, uint8_t *pPalette, uint8_t *pSrc, uint8_t *pDest, int w, int iSrcBpp)
{
    int g = 0, x, iDelta;
    uint8_t *s, *d, *pPal, u8, count;
    const uint8_t u8G2ToG8[4] = {0x00, 0x55, 0xaa, 0xff}; // 2-bit to 8-bit gray

    if (iPixelType == PNG_PIXEL_TRUECOLOR) iSrcBpp = 24;
    else if (iPixelType == PNG_PIXEL_TRUECOLOR_ALPHA) iSrcBpp = 32;
    iDelta = iSrcBpp/8; // bytes per pixel
    count = 8; // bits in a byte
    u8 = 0; // start with all black
    d = pDest;
    s = pSrc;
    for (x=0; x<w; x++) {
        u8 <<= iDestBpp;
        switch (iSrcBpp) {
            case 24:
            case 32:
                g = (s[0] + s[1]*2 + s[2])/4; // convert color to gray value
                s += iDelta;
                break;
            case 8:
                if (iPixelType == PNG_PIXEL_INDEXED) {
                    pPal = &pPalette[s[0] * 3];
                    g = (pPal[0] + pPal[1]*2 + pPal[2])/4;
                } else { // must be grayscale
                    g = s[0];
                }
                s++;
                break;
            case 4:
                if (x & 1) {
                    if (iPixelType == PNG_PIXEL_INDEXED) {
                        pPal = &pPalette[(s[0] & 0xf) * 3];
                        g = (pPal[0] + pPal[1]*2 + pPal[2])/4;
                    } else {
                        g = (s[0] & 0xf) | ((s[0] & 0xf) << 4); // the low nibble only: g is an int
                    }
                    s++;
                } else {
                    if (iPixelType == PNG_PIXEL_INDEXED) {
                        pPal = &pPalette[(s[0]>>4) * 3];
                        g = (pPal[0] + pPal[1]*2 + pPal[2])/4;
                    } else {
                        g = (s[0] & 0xf0) | (s[0] >> 4);
                    }
                }
                break;
            case 2: // We need to handle this case for 2-bit images with (random) palettes
                g = s[0] >> (6-((x & 3) * 2));
                if (iPixelType == PNG_PIXEL_INDEXED) {
                    pPal = &pPalette[(g & 3)*3];
                    g = (pPal[0] + pPal[1]*2 + pPal[2])/4;
                } else {
                    g = u8G2ToG8[g & 3];
                }
                if ((x & 3) == 3) {
                    s++;
                }
                break;
        } // switch on bpp
        if (iDestBpp == 1) {
            u8 |= (g >> 7); // B/W
        } else if (iDestBpp == 2) { // generate 4 gray levels (2 bits)
            u8 |= (3 ^ (g >> 6)); // 4 gray levels (inverted relative to 1-bit)
        } else { // must be 4-bpp output
            u8 |= (g >> 4);
        }
        count -= iDestBpp;
        if (count == 0) { // byte is full, move on
            *d++ = u8;
            u8 = 0;
            count = 8;
        }
    } // for x
    if (count != 8) { // partial byte remaining
        u8 <<= count;
        *d++ = u8;
    }
} /* ReduceBpp() */

#ifdef BB_EPAPER
uint8_t u8SpectraPal[512]; // RGB333 mapped to closest Spectra6 color

//
// Match the given pixel to black (00), white (01), or red (1x)
//
unsigned char GetBWRPixel(int r, int g, int b)
{
    uint8_t ucOut=BBEP_BLACK;
    int gr;

    gr = (b + r + g*2)>>2; // gray
    // match the color to closest of black/white/red
    if (r > g && r > b) { // red is dominant
        if (gr < 100 && r < 80) {
            // black
        } else {
            if (r-b > 32 && r-g > 32) {
                // is red really dominant?
                ucOut = BBEP_RED; // red (can be 2 or 3, but 3 is compatible w/BWYR)
            } else { // yellowish should be white
                // no, use white instead of pink/yellow
                ucOut = BBEP_WHITE;
            }
        }
    } else { // check for white/black
        if (gr >= 128) {
            ucOut = BBEP_WHITE; // white
        } else {
            // black
        }
    }
    return ucOut;
} /* GetBWRPixel() */
//
// Match the given pixel to black (00), white (01), yellow (10), or red (11)
// returns 2 bit value of closest matching color
//
unsigned char GetBWYRPixel(int r, int g, int b)
{
    uint8_t ucOut=BBEP_BLACK;
    int gr;

    gr = (b + r + g*2)>>2; // gray
    // match the color to closest of black/white/yellow/red
    if (r > b || g > b) { // red or yellow is dominant
        if (gr < 90 && r < 80 && g < 80) {
            // black
        } else {
            if (r-b > 32 && r-g > r/2) {
                // is red really dominant?
                ucOut = BBEP_RED; // red
            } else if (r-b > 32 && g-b > 32) {
                // yes, yellow
                ucOut = BBEP_YELLOW;
            } else {
                ucOut = BBEP_WHITE; // gray/white
            }
        }
    } else { // check for white/black
        if (gr >= 100) {
            ucOut = BBEP_WHITE; // white
        } else {
            // black
        }
    }
    return ucOut;
} /* GetBWYRPixel() */

//
// bb_epaper colors to map to Spectra6 colors
// The RGB values are not correct for the panel, but for simple mapping
// these work best. These get mapped from bb_epaper color indices to
// Spectra6 color indices by the setPixel() method.
//
const int iSpectraRGB[] = { // r, g, b
    0, 0, 0, // black = 0
    192,192,192, // white = 1
    192,192,0, // yellow = 2
    192,0,0, // red = 3
    0,0,192, // blue = 4
    0,192,0, // green = 5
};
// Map the Spectra6 palette to the closest RGB333 values
void CreateSpectra6Pal(void)
{
    int i, j;
    int r, g, b, r1, g1, b1;
    int dist, min_dist, min_index;

    for (i=0; i<512; i++) { // RGB333
        r = (i & 7)*36;
        g = ((i >> 3) & 7)*36;
        b = (i >> 6)*36;
        min_dist = 0x7fffffff;
        min_index = 0;
        for (j=0; j<6; j++) { // match to the closes Spectra6 color
            r1 = iSpectraRGB[j*3];
            g1 = iSpectraRGB[j*3+1];
            b1 = iSpectraRGB[j*3+2];
            dist = (r - r1) * (r - r1); // delta red squared
            dist += (g - g1) * (g - g1); // delta green squared
            dist += (b - b1) * (b - b1); // delta blue squared
            if (dist < min_dist) {
                min_dist = dist;
                min_index = j;
            }
        } // for j
        u8SpectraPal[i] = min_index; // best match palette index for this RGB333 color
    } // for i
} /* CreateSpectra6Pal() */
//
// Convert the RGB value into one of 6 Spectra6 colors
//
uint8_t GetSpectraPixel(int r, int g, int b)
{
uint8_t c;
uint16_t rgb333;

    rgb333 = (r>>5) + ((g & 0xe0) >> 2) + ((b & 0xe0) << 1);
    c = u8SpectraPal[rgb333];
    return c;
} /* GetSpectraPixel() */
#endif // BB_EPAPER
