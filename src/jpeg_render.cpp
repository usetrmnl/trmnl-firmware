#include <JPEGDEC.h>
#include <display_internal.h>
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
int iPlane = *(int *)pDraw->pUser;
uint8_t src=0, uc=0, ucMask, *s, *d, *pTemp = bbep.getCache();

    bbep.setAddrWindow(pDraw->x, pDraw->y, pDraw->iWidth, pDraw->iHeight);
    if (iPlane == 0) { // 1-bit mode
        bbep.startWrite(PLANE_0); // start writing image data to plane 0
        for (y=0; y<pDraw->iHeight; y++) { // this is 8 or 16 depending on the color subsampling
            s = (uint8_t *)pDraw->pPixels;
            s += (y * (pDraw->iWidth >> 3));
            // The pixel format of the display is the same as JPEGDEC, so just copy it
            bbep.writeData(s, (pDraw->iWidth+7)/8);
        } // for y
    } else {
        bbep.startWrite((iPlane == 1) ? PLANE_0 : PLANE_1); // start writing image data to plane 0
        for (y=0; y<pDraw->iHeight; y++) { // this is 8 or 16 depending on the color subsampling
            d = pTemp;
            s = (uint8_t *)pDither;
            s += (y * (pDraw->iWidth >> 2));
            ucMask = (iPlane == 1) ? 0x40 : 0x80; // lower or upper source bit
            for (x=0; x<pDraw->iWidth; x++) {
                if ((x & 3) == 0) { // new input byte
                    src = *s++;
                }
                uc <<= 1;
                if (src & ucMask) {
                    uc |= 1; // high bit of source pair
                }
                src <<= 2;
                if ((x & 7) == 7) { // new output byte
                    *d++ = uc;
                }
            } // for x
            bbep.writeData(pTemp, (pDraw->iWidth+7)/8);
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
    return 1; // continue decoding
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
int iPlane = 0;

    if (!jpg) {
        Log_error("%s [%d]: Not enough memory for the JPEG decoder instance", __FILE__, __LINE__);
        return JPEG_ERROR_MEMORY; // not enough memory for the decoder instance
    }
    rc = jpg->openRAM((uint8_t *)pJPEG, iDataSize, jpeg_draw);
    if (rc) {
        if (jpg->getWidth() != bbep.width() || jpg->getHeight() != bbep.height()) {
            Log_error("JPEG image size doesn't match display size");
            rc = -1;
        } else { // okay to decode
#ifdef BB_EPAPER
            //bbep.setPanelType(TWO_BIT_PANEL);
            Log_info("%s [%d]: Decoding jpeg as 1-bpp dithered\r\n", __FILE__, __LINE__);
            jpg->setPixelType(ONE_BIT_DITHERED); // request 1-bit dithered output
#else
            bbep.setMode(BB_MODE_4BPP);
            Log_info("%s [%d]: Decoding jpeg as 4-bpp dithered\r\n", __FILE__, __LINE__);
            jpg->setPixelType(FOUR_BIT_DITHERED); // request 4-bit dithered output
#endif
            pDither = (uint8_t *)malloc(jpg->getWidth() * 16);
            iPlane = 0;//1; // Decode first plane
            Log_info("%s [%d]: Decoding plane 0\r\n", __FILE__, __LINE__);
            jpg->setUserPointer((void *)&iPlane);
            jpg->decodeDither(pDither, 0);
            jpg->close();
            // Decode the second plane
//            iPlane = 2;
//            Log_info("%s [%d]: Decoding plane 1\r\n", __FILE__, __LINE__);
//            jpg->openRAM((uint8_t *)pJPEG, iDataSize, jpeg_draw);
//            jpg->setPixelType(TWO_BIT_DITHERED); // request 1-bit dithered output
//            jpg->setUserPointer((void *)&iPlane);
//            jpg->decodeDither(pDither, 0);
            free(pDither);
#ifdef BB_EPAPER
            rc = REFRESH_FULL;
#endif
        }
    }
    jpg->close();
    delete(jpg);
    return rc;
} /* jpeg_to_epd() */
