//
// bb_epaper
// Written by Larry Bank (bitbank@pobox.com)
// Project started 9/11/2024
//
// SPDX-FileCopyrightText: 2024 BitBank Software, Inc.
// SPDX-License-Identifier: GPL-3.0-or-later
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

#ifdef __LINUX__
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#else // Arduino

#endif // __LINUX__

#include "bb_epaper.h"
#ifdef __MEM_ONLY__
#include "mem_io.inl"
#endif // __MEM_ONLY__

#ifdef __LINUX__
#include "rpi_io.inl"
#else
#ifdef ARDUINO
#include "arduino_io.inl" // I/O (non-portable) code is in here
#elif !defined(__MACH__)
#include "../esp_idf/esp_generic.inl" // ESP-IDF specific
#endif // ARDUINO
#endif // __LINUX__
#include "bb_ep.inl" // All of the display interface code is in here
#include "bb_ep_gfx.inl" // drawing code

#ifdef __cplusplus
//
// C++ Class implementation
//
//uint32_t BBEPAPER::getRefreshTime(void)
//{
//    return _bbep.iTimeout;
//}

BBEPAPER::BBEPAPER(int iPanel)
{
    memset(&_bbep, 0, sizeof(_bbep));
    _bbep.iFG = BBEP_BLACK;
    bbepSetPanelType(&_bbep, iPanel);
}

void BBEPAPER::setDitherPattern(uint8_t iPattern)
{
    bbepSetDitherPattern(&_bbep, iPattern);
} /* setDitherPattern() */

void BBEPAPER::setPasses(int iPasses)
{
   _bbep.iPasses = iPasses;
} /* setPasses() */

void BBEPAPER::setAddrWindow(int x, int y, int w, int h)
{
    bbepSetAddrWindow(&_bbep, x, y, w, h);
}

int BBEPAPER::begin(int iProduct, bool bSharedSPI)
{
int rc = BBEP_ERROR_BAD_PARAMETER;

    switch (iProduct) {
        case EPD_XTEINK_X4: // DC:4 RST:5 BUSY:6 CS:21 MOSI:10 SCK:8
            if (setPanelType(EP426_800x480) == BBEP_SUCCESS) {
                if (bSharedSPI) { // SPI is already initialized
                    initIO(4,5,6,21,-1,-1, 10000000);
                } else {
                    initIO(4,5,6,21,10,8, 10000000);
                }
                return BBEP_SUCCESS;
            }
            break;

        case EPD_WAVESHARE_154: // DC:10 RST:9 BUSY:8 CS:11 MOSI:13 SCK:12
            pinMode(6, OUTPUT); // EPD power enable
            digitalWrite(6, LOW);
            if (setPanelType(EP154_200x200) == BBEP_SUCCESS) {
                initIO(10, 9, 8, 11, 13, 12, 10000000);
                return BBEP_SUCCESS;
            }
            break;

        case EPD_LILYGO_T_DECK_PRO: // DC:35 RST:-1 BUSY:37 CS:34 MOSI:33 SCK:36
// make sure other devices' CS lines are inactive
            pinMode(3 /*BOARD_LORA_CS*/, OUTPUT); 
            digitalWrite(3 /*BOARD_LORA_CS*/, HIGH);
            pinMode(4 /*BOARD_LORA_RST*/, OUTPUT); 
            digitalWrite(4 /*BOARD_LORA_RST*/, HIGH);
            pinMode(48 /*BOARD_SD_CS*/, OUTPUT); 
            digitalWrite(48 /*BOARD_SD_CS*/, HIGH);
            pinMode(34/*BOARD_EPD_CS*/, OUTPUT); 
            digitalWrite(34/*BOARD_EPD_CS*/, HIGH);
            if (setPanelType(EP31_240x320) == BBEP_SUCCESS) {
                initIO(35, -1, 37, 34, 33, 36, 10000000);
                return BBEP_SUCCESS;
            }
            break;

        case EPD_LILYGO_S3_MINI: // DC:12 CS:13 RST: 11 BUSY: 10 SCK: 14 MOSI: 15
            if (setPanelType(EP102_80x128) == BBEP_SUCCESS) {
                initIO(12, 11, 10, 13, 15, 14, 8000000);
                return BBEP_SUCCESS;
            }
            break;
        case EPD_BADGER2040: // DC:20 CS:17 RST:21 BUSY: 26 PWR: 10
            if (setPanelType(EP29_128x296) == BBEP_SUCCESS) {
                initIO(20, 21, 26, 17, -1, -1, 12000000);
                setRotation(270);
                pinMode(10, OUTPUT);
                digitalWrite(10, HIGH); // keep power turned on
                return BBEP_SUCCESS;
            }
            break;
        case EPD_TRMNL_OG: // DC:5 CS:6 RST:10 BUSY:4 MOSI:8 SCK:7
            if (setPanelType(EP75_800x480) == BBEP_SUCCESS) {
                initIO(5, 10, 4, 6, 8, 7, 10000000);
                return BBEP_SUCCESS;
            }
            break;

        case EPD_RETERMINAL_SPECTRA: // Seeed Studio 7.3" color E1002
            // DC:11 RST:12 BUSY:13 CS:10 MOSI:9 SCK:7
            if (setPanelType(EP73_SPECTRA_800x480) == BBEP_SUCCESS) {
                initIO(11, 12, 13, 10, 9, 7, 10000000);
                return BBEP_SUCCESS;
            }
            break;

        case EPD_CROWPANEL154: // DC:13 CS:14 RST:10 BUSY:9 MOSI:11 SCK:12
            pinMode(7, OUTPUT);
            digitalWrite(7, HIGH); // screen power on
            if (setPanelType(EP154Z_152x152) == BBEP_SUCCESS) {
                initIO(13, 10, 9, 14, 11, 12, 12000000);
                return BBEP_SUCCESS;
            }
            break;

        case EPD_BBBADGE:
            setPanelType(EP29Z_128x296);
            // Requesting SPI speed = 0 tells the library to use SPI bit banging
            initIO(6, 3, 24, 8, 41, 7, 0);
            setRotation(270);
            break;

        case EPD_CROWPANEL579:
            pinMode(7, OUTPUT);
            digitalWrite(7, HIGH); // screen power on
            if (setPanelType(EP579_792x272) == BBEP_SUCCESS) {
                initIO(46, 47, 48, 45, 11, 12, 12000000);
                return BBEP_SUCCESS;
            }   
            break;
        case EPD_CROWPANEL37:
            pinMode(7, OUTPUT);
            digitalWrite(7, HIGH); // screen power on
            if (setPanelType(EP37B_240x416) == BBEP_SUCCESS) {
                initIO(46, 47, 48, 45, 11, 12, 12000000);
                return BBEP_SUCCESS;
            }
            break;
        case EPD_CROWPANEL42: // DC:46 CS:45 RST:47 BUSY:48 MOSI:11 SCK:12
            pinMode(7, OUTPUT); 
            digitalWrite(7, HIGH); // screen power on
            if (setPanelType(EP42B_400x300) == BBEP_SUCCESS) {
                initIO(46, 47, 48, 45, 11, 12, 12000000);
                return BBEP_SUCCESS;
            }
            break;
        case EPD_CROWPANEL29: // DC:46 CS:45 RST:47 BUSY:48 MOSI:11 SCK:12
        case EPD_CROWPANEL29_4GRAY:
            pinMode(7, OUTPUT);
            digitalWrite(7, HIGH); // screen power on
            if (setPanelType((iProduct == EPD_CROWPANEL29) ? EP29Z_128x296 : EP29Z_128x296_4GRAY) == BBEP_SUCCESS) {
                initIO(46, 47, 48, 45, 11, 12, 12000000);
                setRotation(270);
                return BBEP_SUCCESS;
            } 
            break;
        case EPD_CROWPANEL213:
        case EPD_CROWPANEL213_4GRAY:
            pinMode(7, OUTPUT);
            digitalWrite(7, HIGH); // screen power on
            if (setPanelType((iProduct == EPD_CROWPANEL213) ? EP213Z_122x250 : EP213Z_122x250_4GRAY) == BBEP_SUCCESS) {
                initIO(13, 10, 9, 14, 11, 12, 12000000);
                setRotation(270);
                return BBEP_SUCCESS;
            } 
            break;
    } // switch on product type
    return rc;
} /* begin() */

int BBEPAPER::setPanelType(int iPanel)
{
    _panel_type = iPanel;
    return bbepSetPanelType(&_bbep, iPanel);
}
// Special setup for dual-cable displays
void BBEPAPER::setCS2(uint8_t cs)
{
    bbepSetCS2(&_bbep, cs);
}

#ifndef __LINUX__
void BBEPAPER::initIO(int iDC, int iReset, int iBusy, int iCS, int iMOSI, int iSCLK, uint32_t u32Speed)
{
    bbepInitIO(&_bbep, iDC, iReset, iBusy, iCS, iMOSI, iSCLK, u32Speed);
} /* initIO() */
#else // Linux
void BBEPAPER::initIO(int iDC, int iReset, int iBusy, int iCS, int iSPIChannel, int iNotUsed, uint32_t u32Speed)
{
	(void)iNotUsed;
	_bbep.iCSPin = iCS;
	_bbep.iDCPin = iDC;
	_bbep.iBUSYPin = iBusy;
	_bbep.iRSTPin = iReset;
	_bbep.iMOSIPin = iSPIChannel;
	bbepInitIO(&_bbep, u32Speed);
} /* initIO() */
#endif

void BBEPAPER::writeRegion(int16_t x, int16_t y, int16_t w, int16_t h, int plane)
{
   bbepWriteRegion(&_bbep, x, y, w, h, plane);
} /* writeRegion() */

int BBEPAPER::writePlane(int iPlane, bool bInvert)
{
    long l = millis();
    int rc;
    rc = bbepWritePlane(&_bbep, iPlane, (int)bInvert);
    _bbep.iDataTime = (int)(millis() - l);
    return rc;
} /* writePlane() */

int BBEPAPER::refresh(int iMode, bool bWait)
{
    int rc;
    long l = millis();
    rc = bbepRefresh(&_bbep, iMode);
    if (rc == BBEP_SUCCESS && bWait) {
        bbepWaitBusy(&_bbep);
    }
    _bbep.iOpTime = (int)(millis() - l);
    return rc;
} /* refresh() */

//int BBEPAPER::getFlags(void)
//{
//    return _bbep.iFlags;
//}

//void BBEPAPER::setFlags(int iFlags)
//{
//    _bbep.iFlags = iFlags;
//}
//
// Create a RAM-only virtual display
//
int BBEPAPER::createVirtual(int iWidth, int iHeight, int iFlags)
{
    return bbepCreateVirtual(&_bbep, iWidth, iHeight, iFlags);
}
void BBEPAPER::setBuffer(uint8_t *pBuffer)
{
    _bbep.ucScreen = pBuffer;
}

void BBEPAPER::stretchAndSmooth(uint8_t *pSrc, uint8_t *pDest, int w, int h, int iSmoothType)
{
    bbepStretchAndSmooth(pSrc, pDest, w, h, iSmoothType);
}
void BBEPAPER::backupPlane(void)
{
    int iSize = ((_bbep.native_width+7)>>3) * _bbep.native_height;
    if (!(_bbep.iFlags & (BBEP_3COLOR | BBEP_4COLOR)) && _bbep.ucScreen) {
        memcpy(&_bbep.ucScreen[iSize], _bbep.ucScreen, iSize);
    }
}
int BBEPAPER::allocBuffer(bool bSecondPlane)
{
    if (_bbep.iFlags & (BBEP_4COLOR | BBEP_3COLOR | BBEP_4GRAY)) bSecondPlane = 1;
    return bbepAllocBuffer(&_bbep, (int)bSecondPlane);
} /* allocBuffer() */

uint8_t * BBEPAPER::getCache(void)
{
    return u8Cache;
}
void * BBEPAPER::getBuffer(void)
{
    return (void *)_bbep.ucScreen;
} /* getBuffer() */

void BBEPAPER::freeBuffer(void)
{
    if (_bbep.ucScreen) {
        free(_bbep.ucScreen);
        _bbep.ucScreen = NULL;
    }
} /* freeBuffer() */

uint32_t BBEPAPER::capabilities(void)
{
  return _bbep.iFlags;
}

void BBEPAPER::setRotation(int iRotation)
{
    bbepSetRotation(&_bbep, iRotation);
} /* setRotation() */

int BBEPAPER::getRotation(void)
{
  return _bbep.iOrientation;
}

void BBEPAPER::fillScreen(int iColor, int iPlane)
{
  long l = millis();
  bbepFill(&_bbep, iColor, iPlane);
  _bbep.iDataTime = (int)(millis() - l); // e.g. for bufferless mode
} /* fillScreen() */

void BBEPAPER::drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
    bbepRectangle(&_bbep, x, y, x+w-1, y+h-1, color, 0);
} /* drawRect() */

void BBEPAPER::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
    bbepRectangle(&_bbep, x, y, x+w-1, y+h-1, color, 1);
} /* fillRect() */

void BBEPAPER::setTextWrap(bool bWrap)
{
  _bbep.wrap = (int)bWrap;
}

bool BBEPAPER::hasFastRefresh()
{
    return (_bbep.pInitFast != NULL);
}

bool BBEPAPER::hasPartialRefresh()
{
    return (_bbep.pInitPart != NULL);
}

void BBEPAPER::setTextColor(int iFG, int iBG)
{
    _bbep.iFG = iFG;
    _bbep.iBG = (iBG == -1) ? iFG : iBG;
} /* setTextColor() */

void BBEPAPER::setCursor(int x, int y)
{
    if (x >= 0)
       _bbep.iCursorX = x;
    if (y >= 0)
    _bbep.iCursorY = y;
} /* setCursor() */

int BBEPAPER::loadG5Image(const uint8_t *pG5, int x, int y, int iFG, int iBG, float fScale)
{
    return bbepLoadG5(&_bbep, pG5, x, y, iFG, iBG, fScale);
} /* loadG5Image() */

int BBEPAPER::loadBMP(const uint8_t *pBMP, int x, int y, int iFG, int iBG)
{
    return bbepLoadBMP(&_bbep, pBMP, x, y, iFG, iBG);
} /* loadBMP() */

int BBEPAPER::loadBMP3(const uint8_t *pBMP, int x, int y)
{
    return bbepLoadBMP3(&_bbep, pBMP, x, y);
} /* loadBMP3() */

void BBEPAPER::setFont(int iFont)
{
    _bbep.iFont = iFont;
    _bbep.pFont = NULL;
} /* setFont() */

void BBEPAPER::setFont(const void *pFont)
{
    _bbep.iFont = -1;
    _bbep.pFont = (void *)pFont;
} /* setFont() */

//void BBEPAPER::setFreeFont(const GFXfont *pFont)
//{
//    _bbep.pFreeFont = (GFXfont *)pFont;
//} /* setFreeFont() */

void BBEPAPER::drawLine(int x1, int y1, int x2, int y2, int iColor)
{
    bbepDrawLine(&_bbep, x1, y1, x2, y2, iColor);
} /* drawLine() */

#ifndef ARDUINO
void BBEPAPER::print(const string &str)
{
   print(str.c_str());
} /* print() */

void BBEPAPER::println(const string &str)
{
char ucTemp[4];

   print(str);
   ucTemp[0] = '\n';
   ucTemp[1] = '\r';
   ucTemp[2] = 0;
   print((const char *)ucTemp);
} /* print() */

void BBEPAPER::print(const char *pString)
{
uint8_t *s = (uint8_t *)pString;

   while (*s != 0) {
      write(*s++);
   }
} /* print() */

void BBEPAPER::println(const char *pString)
{
char ucTemp[4];

    print(pString);
    ucTemp[0] = '\n';
    ucTemp[1] = '\r';
    ucTemp[2] = 0;
    print((const char *)ucTemp);
} /* println() */
void BBEPAPER::print(int value, int format)
{
char c, ucTemp[32];
char *d = &ucTemp[31];

   if (value) {
   d[0] = 0;
   switch(format) {
      case DEC:
         while (value) {
	     d--;
             *d = '0' + (value % 10);
             value /= 10;
	 }
         break;
      case HEX:
	 while (value) {
            d--;
            c = value & 0xf;
	    if (c < 10)
		    *d = '0' + c;
	    else
		    *d = 'A' + (c-10);
	    value >>= 4;
	 }
         break;
      case OCT:
	 while (value) {
            d--;
            *d = '0' + (value & 7);
	    value >>= 3;
	 }
         break;
      case BIN:
         while (value) {
            d--;
            *d = '0' + (value & 1);
	    value >>= 1;
	 }
         break;
      default:
         break;
      }
   } else { // if zero value
     d--;
     *d = '0';
   }
      print((const char *)d);
} /* print() */

void BBEPAPER::println(int value, int format)
{
char ucTemp[4];

	print(value, format);
	ucTemp[0] = '\n';
	ucTemp[1] = '\r';
	ucTemp[2] = 0;
	print((const char *)ucTemp);
} /* println() */

#endif // !ARDUINO_
//
// write (Arduino Print friend class)
//
#if !defined( __AVR__ )
size_t BBEPAPER::write(uint8_t c) {
char szTemp[2]; // used to draw 1 character at a time to the C methods
int w=8, h=8;
static int iUnicodeCount = 0;
static uint8_t u8Unicode0, u8Unicode1;

   if (iUnicodeCount == 0) {
       if (c >= 0x80) { // start of a multi-byte character
           iUnicodeCount++;
           u8Unicode0 = c;
           return 1;
       } 
   } else { // middle/end of a multi-byte character
       uint16_t u16Code;
       if (u8Unicode0 < 0xe0) { // 2 byte char, 0-0x7ff
           u16Code = (u8Unicode0 & 0x3f) << 6;
           u16Code += (c & 0x3f);
           c = bbepUnicodeTo1252(u16Code);
           iUnicodeCount = 0;
       } else { // 3 byte character 0x800 and above
           if (iUnicodeCount == 1) {
               iUnicodeCount++; // save for next byte to arrive
               u8Unicode1 = c;
               return 1;
           }
           u16Code = (u8Unicode0 & 0x3f) << 12;
           u16Code += (u8Unicode1 & 0x3f) << 6;
           u16Code += (c & 0x3f);
           c = bbepUnicodeTo1252(u16Code);
           iUnicodeCount = 0;
       }
   }
   szTemp[0] = c; szTemp[1] = 0;
   if (_bbep.pFont == NULL) { // use built-in fonts
      if (_bbep.iFont == FONT_8x8 || _bbep.iFont == FONT_6x8) {
        h = 8;
        w = (_bbep.iFont == FONT_8x8) ? 8 : 6;
      } else if (_bbep.iFont == FONT_12x16 || _bbep.iFont == FONT_16x16) {
        h = 16;
        w = (_bbep.iFont == FONT_12x16) ? 12:16;
      }

    if (c == '\n') {              // Newline?
      _bbep.iCursorX = 0;          // Reset x to zero,
      _bbep.iCursorY += h; // advance y one line
    } else if (c != '\r') {       // Ignore carriage returns
        if (_bbep.wrap && ((_bbep.iCursorX + w) > _bbep.width)) { // Off right?
            _bbep.iCursorX = 0;               // Reset x to zero,
            _bbep.iCursorY += h; // advance y one line
        }
        bbepWriteString(&_bbep, -1, -1, szTemp, _bbep.iFont, _bbep.iFG, _bbep.iBG);
    }
  } else { // Custom font
      BB_FONT *pBBF;
      BB_FONT_SMALL *pBBFS;
      BB_GLYPH *pGlyph;
      BB_GLYPH_SMALL *pSmallGlyph;
      int first, last;
      if (pgm_read_word(_bbep.pFont) == BB_FONT_MARKER) {
          pBBF = (BB_FONT *)_bbep.pFont; pBBFS = NULL;
          first = pgm_read_byte(&pBBF->first);
          last = pgm_read_byte(&pBBF->last);
      } else { // small font
          pBBFS = (BB_FONT_SMALL *)_bbep.pFont; pBBF = NULL;
          first = pgm_read_byte(&pBBFS->first);
          last = pgm_read_byte(&pBBFS->last);
      }
    if (c == '\n') {
      _bbep.iCursorX = 0;
       if (pBBF) {
           _bbep.iCursorY += pBBF->height;
       } else {
           _bbep.iCursorY += pBBFS->height;
       }
    } else if (c != '\r') {
      if (c >= first && c <= last) {
          if (pBBF) {
              pGlyph = &pBBF->glyphs[c - first];
              w = pgm_read_word(&pGlyph->width);
              h = pgm_read_word(&pGlyph->height);
          } else { // small font
              pSmallGlyph = &pBBFS->glyphs[c - first];
              w = pgm_read_word(&pSmallGlyph->width);
              h = pgm_read_word(&pSmallGlyph->height);
          }
      if (w > 0 && h > 0) { // Is there an associated bitmap?
          if (pBBF) {
              w += (int16_t)pgm_read_word(&pGlyph->xOffset);
          } else {
              w += (int8_t)pgm_read_byte(&pSmallGlyph->xOffset);
          }
          if (_bbep.wrap && (_bbep.iCursorX + w) > _bbep.width) {
            _bbep.iCursorX = 0;
            _bbep.iCursorY += h;
          }
          bbepWriteStringCustom(&_bbep, _bbep.pFont, -1, -1, szTemp, _bbep.iFG, _bbep.iPlane);
        }
      }
    }
  }
  return 1;
} /* write() */
#endif // ARDUINO && !__AVR__

void BBEPAPER::drawPixel(int16_t x, int16_t y, uint8_t color)
{
    (*_bbep.pfnSetPixel)(&_bbep, x, y, color);
}
int16_t BBEPAPER::getCursorX(void)
{
  return _bbep.iCursorX;
}
int16_t BBEPAPER::getCursorY(void)
{
  return _bbep.iCursorY;
}
void BBEPAPER::getStringBox(const char *string, BB_RECT *pRect)
{
    bbepGetStringBox(&_bbep, (char *)string, pRect);
}
#ifdef ARDUINO
void BBEPAPER::getStringBox(const String &str, BB_RECT *pRect)
{
    bbepGetStringBox(&_bbep, str.c_str(), pRect);
}
#endif
int BBEPAPER::dataTime(void)
{
    return _bbep.iDataTime;
}
int BBEPAPER::opTime(void)
{
    return _bbep.iOpTime;
}
int16_t BBEPAPER::width(void)
{
   return _bbep.width;
}
int16_t BBEPAPER::height(void)
{
   return _bbep.height;
}
void BBEPAPER::drawCircle(int32_t x, int32_t y, int32_t r, uint32_t color)
{
  bbepEllipse(&_bbep, x, y, r, r, 0xf, color, 0);
}
void BBEPAPER::fillCircle(int32_t x, int32_t y, int32_t r, uint32_t color)
{
    bbepEllipse(&_bbep, x, y, r, r, 0xf, color, 1);
}
void BBEPAPER::drawEllipse(int16_t x, int16_t y, int32_t rx, int32_t ry, uint16_t color)
{
    bbepEllipse(&_bbep, x, y, rx, ry, 0xf, color, 0);
}
void BBEPAPER::fillEllipse(int16_t x, int16_t y, int32_t rx, int32_t ry, uint16_t color)
{
    bbepEllipse(&_bbep, x, y, rx, ry, 0xf, color, 1);
}

int BBEPAPER::testPanelType(void)
{
    return bbepTestPanelType(&_bbep);
}
void BBEPAPER::wake(void)
{
    bbepWakeUp(&_bbep);
    if (_bbep.iFlags & (BBEP_7COLOR | BBEP_4COLOR)) { // need to send before you can send it data
        bbepSendCMDSequence(&_bbep, _bbep.pInitFull);
        if (_bbep.iFlags & BBEP_SPLIT_BUFFER) { // dual cable EPD
            _bbep.iCSPin = _bbep.iCS2Pin;
            bbepSendCMDSequence(&_bbep, _bbep.pInitFull); // second controller 
            _bbep.iCSPin = _bbep.iCS1Pin;
        }
    }

}

void BBEPAPER::sleep(int bDeep)
{
    bbepSleep(&_bbep, bDeep);
}
void BBEPAPER::wait(bool bQuick)
{
    bbepWaitBusy(&_bbep);
}
bool BBEPAPER::isBusy(void)
{
    return bbepIsBusy(&_bbep);
}
void BBEPAPER::drawString(const char *pText, int x, int y)
{
    if (_bbep.pFont) {
        bbepWriteStringCustom(&_bbep, (BB_FONT *)_bbep.pFont, x, y, (char *)pText, _bbep.iFG, 0); // iPlane);
    } else if (_bbep.iFont >= FONT_6x8 && _bbep.iFont < FONT_COUNT) {
        bbepWriteString(&_bbep, x, y, (char *)pText, _bbep.iFont, _bbep.iFG, _bbep.iBG);
    }
} /* drawString() */

void BBEPAPER::setPlane(int iPlane)
{
    if (_bbep.iFlags & (BBEP_3COLOR | BBEP_4COLOR | BBEP_4GRAY | BBEP_7COLOR)) return; // only valid for 1-bit mode

    _bbep.iPlane = iPlane;
}
int BBEPAPER::getPlane(void)
{
    return _bbep.iPlane;
}

int BBEPAPER::getChip(void)
{
    return _bbep.chip_type;
}
void BBEPAPER::drawSprite(const uint8_t *pSprite, int cx, int cy, int iPitch, int x, int y, uint8_t iColor)
{
    bbepDrawSprite(&_bbep, pSprite, cx, cy, iPitch, x, y, iColor);
}
void BBEPAPER::startWrite(int iPlane)
{
    bbepStartWrite(&_bbep, iPlane);
} /* startWrite() */

void BBEPAPER::writeData(uint8_t *pData, int iLen)
{
   bbepWriteData(&_bbep, pData, iLen);
} /* writeData() */

void BBEPAPER::writeCmd(uint8_t u8Cmd)
{
    bbepWriteCmd(&_bbep, u8Cmd);
} /* writeCmd() */

void BBEPAPER::drawRoundRect(int x, int y, int w, int h,
                   int r, uint8_t color)
{
    bbepRoundRect(&_bbep, x, y, w, h, r, color, 0);
}
void BBEPAPER::fillRoundRect(int x, int y, int w, int h,
                   int r, uint8_t color)
{
    bbepRoundRect(&_bbep, x, y, w, h, r, color, 1);
}

int BBEPAPER::getLastError(void)
{
    return _bbep.last_error;
}
#endif // __cplusplus
