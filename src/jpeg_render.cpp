#include <JPEGDEC.h>
#include <color_reduce.h>
#include <epd_driver.h>
#include <jpeg_render.h>
#include <trmnl_log.h>

static uint8_t *pDither;

/**
 * @brief JPEGDEC callback function passed blocks of MCUs (minimum coded units)
 * @param pointer to the JPEGDRAW structure
 * @return 1 to continue decoding or 0 to abort
 */
int jpeg_draw(JPEGDRAW *pDraw)
{
#ifdef BB_EPAPER
int x, y;
uint8_t *s, *d;
int iWidth;
static int iLastY = -1; // needed to know when to write the rows to the 4-color EPD

    iWidth = pDraw->iWidth;
    if ((pDraw->x + iWidth) > bbep.width()) iWidth = (bbep.width() - pDraw->x); // image too wide
    if (pDraw->y >= bbep.height()) return 0; // can't go past the bottom of the EPD

    if (pDraw->pUser) { // we're doing color matching
        uint16_t u16, *s16;
        if (bbep.capabilities() & BBEP_7COLOR) {
            // match the pixels to the Spectra6 palette
            for (y=0; y<pDraw->iHeight && (y+pDraw->y) < bbep.height(); y++) {
                s16 = (uint16_t *)pDraw->pPixels;
                s16 += (y * pDraw->iWidth);
                d = (uint8_t *)bbep.getBuffer(); // for Spectra6 we always allocate a buffer
                d += ((pDraw->y+y) * ((bbep.width()+1)/2));
                d += (pDraw->x/2);
                for (x=0; x<iWidth; x+=2) {
                    int r, g, b;
                    uint8_t u8;
                    u16 = s16[x];
                    r = (u16 >> 8) & 0xf8;
                    g = (u16 >> 3) & 0xfc;
                    b = (u16 << 3) & 0xf8;
                    u8 = GetSpectraPixel(r, g, b) << 4; // high nibble
                    u16 = s16[x+1];
                    r = (u16 >> 8) & 0xf8;
                    g = (u16 >> 3) & 0xfc;
                    b = (u16 << 3) & 0xf8;
                    u8 |= GetSpectraPixel(r, g, b); // low nibble
                    *d++ = u8;
                }
            }
        } else if (bbep.capabilities() & BBEP_4COLOR) {
            if (pDraw->y == 0 && iLastY == -1) { // first block of first row
                iLastY = 0;
                bbep.startWrite(PLANE_1);
                memset(pDraw->pUser, 0x55, 16 * ((bbep.width()+3)/4)); // fill potential undrawn area with white
            }
            if (iLastY != pDraw->y) { // write the last block of lines we processed
                for (y=iLastY; y<pDraw->y; y++) {
                    d = (uint8_t *)pDraw->pUser;
                    d += ((y-iLastY) * ((bbep.width()+3)/4));
                    bbep.writeData(d, (bbep.width()+3)/4);
                }
                iLastY = pDraw->y;
            }
            // match the pixels to the Spectra6 palette
            for (y=0; y<pDraw->iHeight; y++) {
                s16 = (uint16_t *)pDraw->pPixels;
                s16 += (y * pDraw->iWidth);
                d = (uint8_t *)pDraw->pUser;
                d += (y * ((bbep.width()+3)/4));
                d += (pDraw->x/4);
                for (x=0; x<iWidth; x+=4) { // work 4 pixels at a time
                    int r, g, b;
                    uint8_t u8 = 0;
                    for (int pix=0; pix<4; pix++) {
                        u16 = s16[x+pix];
                        r = (u16 >> 8) & 0xf8;
                        g = (u16 >> 3) & 0xfc;
                        b = (u16 << 3) & 0xf8;
                        u8 <<= 2;
                        u8 |= GetBWRPixel(r, g, b);
                    }
                    *d++ = u8;
                }
            }
        } // 4-color
    } else { // dithered output to 1-bit B/W panel
        if (pDraw->y == 0) { // first rows, prepare the EPD for receiving the data
            bbep.startWrite(PLANE_0); // start writing image data to plane 0
        }
        // JPEGDEC dithered output always uses the full width of the image
        for (y=0; y<pDraw->iHeight && (y+pDraw->y) < bbep.height(); y++) { // this is 8 or 16 depending on the color subsampling
            s = (uint8_t *)pDraw->pPixels;
            s += (y * (pDraw->iWidth >> 3));
            // The pixel format of the display is the same as JPEGDEC, so just copy it
            bbep.writeData(s, (iWidth+7)/8);
            if (iWidth < bbep.width()) { // fill the missing bytes with white
                uint8_t *d = bbep.getCache();
                int iLen = (bbep.width()/8) - ((iWidth+7)/8);
                memset(d, 0xff, iLen);
                bbep.writeData(d, iLen);
            }
        } // for y
    }
#else // FastEPD
  int x, y, iPitch = bbep.width()/2; // assume 4-bpp drawing mode
  uint8_t *s, *d, *pBuffer = bbep.currentBuffer();
  for (y=0; y<pDraw->iHeight; y++) {
    d = &pBuffer[((pDraw->y + y)*iPitch) + (pDraw->x/2)];
    s = (uint8_t *)pDraw->pPixels;
    s += (y * (pDraw->iWidth/2));
    memcpy(d, s, pDraw->iWidth/2); // source & dest format are the same
  } // for y
#endif
    return 1; // Continue decoding until image reaches the bottom row of the EPD
} /* jpeg_draw() */
/**
 * @brief Function to decode and display a JPEG image from memory
 *        The decoded lines are written directly into the EPD framebuffer
 *        due to insufficient RAM to hold the fully decoded image
 * @param pointer to the buffer holding the JPEG file
 * @param size of the JPEG file
 * @return refresh mode based on image type and presence of old image
 */
int jpeg_to_epd(const uint8_t *pJPEG, int iDataSize)
{
JPEGDEC *jpg = new JPEGDEC();
int rc = -1; // invalid mode

    if (!jpg) {
        Log_error("%s [%d]: Not enough memory for the JPEG decoder instance", __FILE__, __LINE__);
        return JPEG_ERROR_MEMORY; // not enough memory for the decoder instance
    }
    rc = jpg->openRAM((uint8_t *)pJPEG, iDataSize, jpeg_draw);
    if (rc) {
        bool bDithered = false;
        int iPitch;
        uint8_t fill=0;

#ifdef BB_EPAPER
            //bbep.setPanelType(TWO_BIT_PANEL);
            if (!(bbep.capabilities() & (BBEP_3COLOR | BBEP_4COLOR | BBEP_7COLOR))) {
                Log_info("%s [%d]: Decoding jpeg as 1-bpp dithered\r\n", __FILE__, __LINE__);
                jpg->setPixelType(ONE_BIT_DITHERED); // request 1-bit dithered output
                bDithered = true;
            }
#else
            bbep.setMode(BB_MODE_4BPP);
            bbep.fillScreen(0xf); // fill with white
            Log_info("%s [%d]: Decoding jpeg as 4-bpp dithered\r\n", __FILE__, __LINE__);
            jpg->setPixelType(FOUR_BIT_DITHERED); // request 4-bit dithered output
            bDithered = true;
#endif
            if (bDithered) { // generate 1 or 4-bit b/w dithered output
                Log_info("%s [%d]: Decoding as 1-bit dithered\r\n", __FILE__, __LINE__);
                pDither = (uint8_t *)malloc(jpg->getWidth() * 16);
                if (!pDither) {
                    Log_error("%s [%d]: Not enough memory for the JPEG dither buffer", __FILE__, __LINE__);
                    free(jpg);
                    return JPEG_ERROR_MEMORY; // not enough memory for the decoder instance
                }
                iPitch = (bbep.width() + 7)/8;
                fill = 0xff;
                jpg->setUserPointer((void *)NULL);
                jpg->decodeDither(pDither, 0);
            }
#ifdef BB_EPAPER
             else { // Do color matching in jpeg_draw()
                if (bbep.capabilities() & BBEP_4COLOR) {
                    iPitch = (bbep.width() + 3)/4;
                    fill = 0x55; // 2-bit white = 01
                } else if (bbep.capabilities() & BBEP_7COLOR) {
                    iPitch = (bbep.width() + 1)/2;
                    fill = 0x11; // 4-bit white = 0001
                    CreateSpectra6Pal(); // create a fast color matching palette
                    if (bbep.allocBuffer() != BBEP_SUCCESS) {
                        Log_error("%s [%d]: bbep.AllocBuffer failed!\n\r", __FILE__, __LINE__);
                        free(jpg);
                        return JPEG_ERROR_MEMORY;
                    }
                    bbep.fillScreen(BBEP_WHITE); // fill background with white in case of smaller image
                } else { // 1-bpp
                    iPitch = (bbep.width() + 7)/8;
                    fill = 0xff; // 1-bit white = 1
                }
                pDither = (uint8_t *)malloc(16 * iPitch); // needed to collect up to 16 rows of pixels from the JPEGDRAW callback
                if (!pDither) {
                    Log_error("%s [%d]: Not enough memory for the JPEG row buffer", __FILE__, __LINE__);
                    free(jpg);
                    return JPEG_ERROR_MEMORY; // not enough memory for the decoder instance
                }
                jpg->setUserPointer(pDither); // Not for dithering, just using the same variable
                jpg->setPixelType(RGB565_LITTLE_ENDIAN); // This is the fastest pixel format to decode
                jpg->decode(0,0,0);
                if (bbep.capabilities() & BBEP_7COLOR) {
                    bbep.writePlane(); // send the pixels to the EPD
                }
            }
            if (jpg->getHeight() < bbep.height() && !(bbep.capabilities() & BBEP_7COLOR)) {
                // fill the missing rows with white
                uint8_t *d = bbep.getCache();
                memset(d, fill, iPitch);
                for (int y=jpg->getHeight(); y < bbep.height(); y++) {
                    bbep.writeData(d, iPitch);
                }
            }
#endif // BB_EPAPER
            jpg->close();
            free(pDither);
#ifdef BB_EPAPER
            rc = REFRESH_FULL;
#endif
//        }
    }
    jpg->close();
    delete(jpg);
    return rc;
} /* jpeg_to_epd() */
