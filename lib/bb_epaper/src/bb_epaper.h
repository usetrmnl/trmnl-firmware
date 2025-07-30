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
#ifndef __BB_EPAPER__
#define __BB_EPAPER__

#ifdef ARDUINO
#include <Arduino.h>
#ifdef __AVR__
// AVR doesn't have enough RAM for a back buffer
#define NO_RAM
#endif // __AVR__
#else // __LINUX__
// for Print support
#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2
#define PROGMEM
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#endif // __LINUX__

// error messages
enum {
    BBEP_SUCCESS,
    BBEP_ERROR_BAD_PARAMETER,
    BBEP_ERROR_BAD_DATA,
    BBEP_ERROR_NOT_SUPPORTED,
    BBEP_ERROR_NO_MEMORY,
    BBEP_ERROR_OUT_OF_BOUNDS,
    BBEP_ERROR_COUNT
};

typedef struct {
    int x; 
    int y;
    int w; 
    int h;
} BB_RECT; 

#define LIGHT_SLEEP 0
#define DEEP_SLEEP 1

// Display refresh modes
#define REFRESH_FULL 0
#define REFRESH_FAST 1
#define REFRESH_PARTIAL 2
#define REFRESH_PARTIAL2 3

// Stretch+smoothing options
#define BBEP_SMOOTH_NONE  0
#define BBEP_SMOOTH_HEAVY 1
#define BBEP_SMOOTH_LIGHT 2

// controller chip types
enum {
    BBEP_CHIP_NOT_DEFINED = 0,
    BBEP_CHIP_SSD16xx,
    BBEP_CHIP_UC81xx,
    BBEP_CHIP_IT8951,
    BBEP_CHIP_NONE, // for virtual display
    BBEP_CHIP_COUNT
};

// enum for writing local framebuffer to EPD memory plane(s)
enum {
    PLANE_0=0,
    PLANE_1,
    PLANE_BOTH,
    PLANE_DUPLICATE, // duplicate 0 to both 0 and 1
    PLANE_0_TO_1 // send plane 0 to plane 1 memory
};
#ifndef __ONEBITDISPLAY__
// 5 possible font sizes: 8x8, 16x32, 6x8, 12x16 (stretched from 6x8 with smoothing), 16x16 (stretched from 8x8)
enum {
   FONT_6x8 = 0,
   FONT_8x8,
   FONT_12x16,
   FONT_16x16,
   FONT_COUNT
};
#endif

// Centering coordinates to pass to the character drawing functions
#define CENTER_X 9998
#define CENTER_Y 9999

typedef struct epd_panel {
    uint16_t width;
    uint16_t height;
    int16_t x_offset;
    const uint8_t *pInitFull;
    const uint8_t *pInitFast;
    const uint8_t *pInitPart;
    uint16_t flags;
    uint8_t chip_type;
    const uint8_t *pColorLookup; // color translation table
} EPD_PANEL;

// Display types
enum {
    EP_PANEL_UNDEFINED=0,
    EP42_400x300, // WFT0420CZ15
    EP42B_400x300, // DEPG0420BN / GDEY042T81
    EP213_122x250, // waveshare
    EP213B_122x250, // GDEY0213B74 (Inky phat 2.13" B/W newer version)
    EP293_128x296,
    EP294_128x296, // Waveshare newer 2.9" 1-bit 128x296
    EP295_128x296, // harvested from Solum 2.9" BW ESLs
    EP266_152x296, // GDEY0266T90
    EP102_80x128, // GDEW0102T4
    EP27B_176x264, // GDEY027T91
    EP29R_128x296,
    EP122_192x176, // GDEM0122T61
    EP154R_152x152,
    EP42R_400x300,
    EP42R2_400x300, // GDEQ042Z21
    EP37_240x416, // GDEY037T03
    EP213_104x212, // InkyPHAT 2.13 black and white
    EP75_800x480, // GDEY075T7
    EP75_800x480_4GRAY, // GDEW075T7 in 4 grayscale mode
    EP75_800x480_4GRAY_OLD, // GDEY075T7 in 4 grayscale mode
    EP29_128x296, // Pimoroni Badger2040
    EP213R_122x250, // Inky phat 2.13 B/W/R
    EP154_200x200, // waveshare
    EP154B_200x200, // DEPG01540BN
    EP266YR_184x360, // GDEY0266F51
    EP29YR_128x296, // GDEY029F51
    EP29YR_168x384, // GDEY029F51H
    EP583_648x480, // DEPG0583BN
    EP296_128x296, // Waveshare 2.9" 128x296 B/W V2
    EP26R_152x296, // Solum 2.6" B/W/R harvested panel
    EP73_800x480, // GEDY073D46 (slower, EOL 7-color)
    EP73_SPECTRA_800x480, // Spectra 6 7-color 800x480
    EP74R_640x384,
    EP583R_600x448, // 4-bits per pixel needs different support
    EP75R_800x480, // Waveshare 800x480 3-color
    EP426_800x480, // Waveshare 4.26" B/W 800x480
    EP426_800x480_4GRAY, // Waveshare 4.26" 800x480 2-bit grayscale mode
    EP29R2_128x296, // Adafruit 2.9" 128x296 Tricolor FeatherWing
    EP41_640x400, // EInk ED040TC1 SPI UC81xx
    EP81_SPECTRA_1024x576, // Spectra 8.1" 1024x576 6-colors
    EP7_960x640, // ED070EC1
    EP_PANEL_COUNT
};
#ifdef FUTURE
    EPD42_4GRAY_400x300, // WFT0420CZ15
  EPD42Y_400x300, // DEPG0420YN
  EPD29_128x296,
  EPD29B_128x296,
  EPD29Y_128x296, // DEPG0290YN
  EPD213R2_122x250, // DEPG0213RW
  EPD213R_104x212_d,
  EPD154_152x152, // GDEW0154M10
  EPD154R_152x152,
  EPD154Y_152x152, // DEPG0154YN
  EPD266B_152x296, // DEPG0266BN
  EPD266Y_152x296, // DEPG0266YN
  EPD31R_168x296, // DEPG0310RW
  EPD37Y_240x416, // DEPG0370YN
  EPD579_792x272, // GDEY0579T93
  EPD74R_640x384,
  EPD30_BWYR_168x400, // Waveshare 3" B/W/Y/R
  EPD164_BWYR_168x168, // Waveshare 1.64" B/W/Y/R
  EPD236_BWYR_168x296, // Waveshare 2.36" B/W/Y/R
  EPD47_540x960, // not working yet
  EPD35R_184x384, // Hanshow Nebular 3.5" BWR
  EPD_PANEL_COUNT
};
#endif // FUTURE
// flag bits
#define BBEP_RED_SWAPPED 0x0001
#define BBEP_BITBANG  0x0002
#define BBEP_3COLOR   0x0004
#define BBEP_4COLOR   0x0008
#define BBEP_4GRAY    0x0010
#define BBEP_7COLOR   0x0020
#define BBEP_16GRAY   0x0020
#define BBEP_CS_EVERY_BYTE 0x0040
#define BBEP_PARTIAL2 0x0080
#define BBEP_4BPP_DATA 0x0100
#define BBEP_SPLIT_BUFFER 0x0200
#define BBEP_HAS_SECOND_PLANE 0x0400

#define BBEP_BLACK 0
#define BBEP_WHITE 1
#define BBEP_YELLOW 2
#define BBEP_RED 3
#define BBEP_BLUE 4
#define BBEP_GREEN 5
#define BBEP_ORANGE 6
#define BBEP_TRANSPARENT 255

// 4 gray levels
#define BBEP_GRAY0 0
#define BBEP_GRAY1 1
#define BBEP_GRAY2 2
#define BBEP_GRAY3 3

#ifndef __ONEBITDISPLAY__
// (IT8951 commands)
// Built in I80 Command Code
#define IT8951_TCON_SYS_RUN 0x0001
#define IT8951_TCON_STANDBY 0x0002
#define IT8951_TCON_SLEEP   0x0003
#define IT8951_TCON_REG_RD  0x0010
#define IT8951_TCON_REG_WR  0x0011

#define IT8951_TCON_MEM_BST_RD_T 0x0012
#define IT8951_TCON_MEM_BST_RD_S 0x0013
#define IT8951_TCON_MEM_BST_WR   0x0014
#define IT8951_TCON_MEM_BST_END  0x0015

#define IT8951_TCON_LD_IMG      0x0020
#define IT8951_TCON_LD_IMG_AREA 0x0021
#define IT8951_TCON_LD_IMG_END  0x0022

// I80 User defined command code
#define IT8951_I80_CMD_DPY_AREA     0x0034
#define IT8951_I80_CMD_GET_DEV_INFO 0x0302
#define IT8951_I80_CMD_DPY_BUF_AREA 0x0037
#define IT8951_I80_CMD_VCOM         0x0039
// Rotate mode
#define IT8951_ROTATE_0   0
#define IT8951_ROTATE_90  1
#define IT8951_ROTATE_180 2
#define IT8951_ROTATE_270 3

// Pixel mode (Bit per Pixel)
#define IT8951_2BPP 0
#define IT8951_3BPP 1
#define IT8951_4BPP 2
#define IT8951_8BPP 3

// Endian Type
#define IT8951_LDIMG_L_ENDIAN 0
#define IT8951_LDIMG_B_ENDIAN 1
/*-----------------------------------------------------------------------
IT8951 Registers defines
------------------------------------------------------------------------*/
// Register Base Address
#define IT8951_DISPLAY_REG_BASE 0x1000  // Register RW access

// Base Address of Basic LUT Registers
#define IT8951_LUT0EWHR \
    (IT8951_DISPLAY_REG_BASE + 0x00)  // LUT0 Engine Width Height Reg
#define IT8951_LUT0XYR (IT8951_DISPLAY_REG_BASE + 0x40)  // LUT0 XY Reg
#define IT8951_LUT0BADDR \
    (IT8951_DISPLAY_REG_BASE + 0x80)  // LUT0 Base Address Reg
#define IT8951_LUT0MFN \
    (IT8951_DISPLAY_REG_BASE + 0xC0)  // LUT0 Mode and Frame number Reg
#define IT8951_LUT01AF \
    (IT8951_DISPLAY_REG_BASE + 0x114)  // LUT0 and LUT1 Active Flag Reg

// Update Parameter Setting Register
#define IT8951_UP0SR \
    (IT8951_DISPLAY_REG_BASE + 0x134)  // Update Parameter0 Setting Reg
#define IT8951_UP1SR \
    (IT8951_DISPLAY_REG_BASE + 0x138)  // Update Parameter1 Setting Reg
#define IT8951_LUT0ABFRV       \
    (IT8951_DISPLAY_REG_BASE + \
     0x13C)  // LUT0 Alpha blend and Fill rectangle Value
#define IT8951_UPBBADDR \
    (IT8951_DISPLAY_REG_BASE + 0x17C)  // Update Buffer Base Address
#define IT8951_LUT0IMXY \
    (IT8951_DISPLAY_REG_BASE + 0x180)  // LUT0 Image buffer X/Y offset Reg
#define IT8951_LUTAFSR         \
    (IT8951_DISPLAY_REG_BASE + \
     0x224)  // LUT Status Reg (status of All LUT Engines)
#define IT8951_BGVR \
    (IT8951_DISPLAY_REG_BASE + 0x250)  // Bitmap (1bpp) image color table

// System Registers
#define IT8951_SYS_REG_BASE 0x0000

// Address of System Registers
#define IT8951_I80CPCR (IT8951_SYS_REG_BASE + 0x04)
// Memory Converter Registers
#define IT8951_MCSR_BASE_ADDR 0x0200
#define IT8951_MCSR           (IT8951_MCSR_BASE_ADDR + 0x0000)
#define IT8951_LISAR          (IT8951_MCSR_BASE_ADDR + 0x0008)

// (UC81xx commands)
enum {
    UC8151_PSR      = 0x00,
    UC8151_PWR      = 0x01,
    UC8151_POFF     = 0x02,
    UC8151_PFS      = 0x03,
    UC8151_PON      = 0x04,
    UC8151_PMES     = 0x05,
    UC8151_BTST     = 0x06,
    UC8151_DSLP     = 0x07,
    UC8151_DTM1     = 0x10,
    UC8151_DSP      = 0x11,
    UC8151_DRF      = 0x12,
    UC8151_DTM2     = 0x13,
    UC8151_LUT_VCOM = 0x20,
    UC8151_LUT_WW   = 0x21,
    UC8151_LUT_BW   = 0x22,
    UC8151_LUT_WB   = 0x23,
    UC8151_LUT_BB   = 0x24,
    UC8151_LUT_VCOM2 = 0x25,
    UC8151_PLL      = 0x30,
    UC8151_TSC      = 0x40,
    UC8151_TSE      = 0x41,
    UC8151_TSR      = 0x43,
    UC8151_TSW      = 0x42,
    UC8151_CDI      = 0x50,
    UC8151_LPD      = 0x51,
    UC8151_TCON     = 0x60,
    UC8151_TRES     = 0x61,
    UC8151_REV      = 0x70,
    UC8151_FLG      = 0x71,
    UC8151_AMV      = 0x80,
    UC8151_VV       = 0x81,
    UC8151_VDCS     = 0x82,
    UC8151_PTL      = 0x90,
    UC8151_PTIN     = 0x91,
    UC8151_PTOU     = 0x92,
    UC8151_PGM      = 0xa0,
    UC8151_APG      = 0xa1,
    UC8151_ROTP     = 0xa2,
    UC8151_CCSET    = 0xe0,
    UC8151_PWS      = 0xe3,
    UC8151_TSSET    = 0xe5
  };

enum {
    SSD1608_DRIVER_CONTROL = 0x01,
    SSD1608_GATE_VOLTAGE = 0x03,
    SSD1608_SOURCE_VOLTAGE = 0x04,
    SSD1608_DISPLAY_CONTROL = 0x07,
    SSD1608_NON_OVERLAP = 0x0B,
    SSD1608_BOOSTER_SOFT_START = 0x0C,
    SSD1608_GATE_SCAN_START = 0x0F,
    SSD1608_DEEP_SLEEP = 0x10,
    SSD1608_DATA_MODE = 0x11,
    SSD1608_SW_RESET = 0x12,
    SSD1608_TEMP_WRITE = 0x1A,
    SSD1608_TEMP_READ = 0x1B,
    SSD1608_TEMP_CONTROL = 0x1C,
    SSD1608_TEMP_LOAD = 0x1D,
    SSD1608_MASTER_ACTIVATE = 0x20,
    SSD1608_DISP_CTRL1 = 0x21,
    SSD1608_DISP_CTRL2 = 0x22,
    SSD1608_WRITE_RAM = 0x24,
    SSD1608_WRITE_ALTRAM = 0x26,
    SSD1608_READ_RAM = 0x25,
    SSD1608_VCOM_SENSE = 0x28,
    SSD1608_VCOM_DURATION = 0x29,
    SSD1608_WRITE_VCOM = 0x2C,
    SSD1608_READ_OTP = 0x2D,
    SSD1608_WRITE_LUT = 0x32,
    SSD1608_WRITE_DUMMY = 0x3A,
    SSD1608_WRITE_GATELINE = 0x3B,
    SSD1608_WRITE_BORDER = 0x3C,
    SSD1608_SET_RAMXPOS = 0x44,
    SSD1608_SET_RAMYPOS = 0x45,
    SSD1608_SET_RAMXCOUNT = 0x4E,
    SSD1608_SET_RAMYCOUNT = 0x4F,
    SSD1608_NOP = 0xFF,
};
#endif

#define BUSY_WAIT 0xff

// Normal pixel drawing function pointer
typedef int (BB_SET_PIXEL)(void *pBBEP, int x, int y, unsigned char color);
// Fast pixel drawing function pointer (no boundary checking)
typedef void (BB_SET_PIXEL_FAST)(void *pBBEP, int x, int y, unsigned char color);

typedef struct bbepstruct
{
uint8_t wrap, type, chip_type, last_error;
uint8_t *ucScreen;
int iCursorX, iCursorY;
int width, height, native_width, native_height;
int iScreenOffset, iOrientation;
int iFG, iBG; //current color
int iFont, iFlags;
void *pFont;
int iDataTime, iOpTime; // time in milliseconds for data transmission and operation
uint32_t iSpeed;
uint32_t iTimeout; // for e-paper panels
uint8_t iDCPin, iMOSIPin, iCLKPin, iCSPin, iRSTPin, iBUSYPin;
uint8_t iCS1Pin, iCS2Pin;
uint8_t x_offset, y_offset; // memory offsets
uint8_t is_awake, iPlane;
const uint8_t *pColorLookup; // color translation table
const uint8_t *pInitFull; // full update init sequence
const uint8_t *pInitFast; // fast update init sequence
const uint8_t *pInitPart; // partial update init sequence
BB_SET_PIXEL *pfnSetPixel;
BB_SET_PIXEL_FAST *pfnSetPixelFast;
} BBEPDISP;

#ifdef __cplusplus
#ifndef ARDUINO
#include <string>
using namespace std;
class BBEPAPER
#else // Arduino
#ifndef __AVR__
class BBEPAPER : public Print
#else
class BBEPAPER
#endif // !__AVR__
#endif // __LINUX__
{
  public:
    BBEPAPER(int iPanel);
    int createVirtual(int iWidth, int iHeight, int iFlags);
    void setAddrWindow(int x, int y, int w, int h);
    int setPanelType(int iPanel);
    void setCS2(uint8_t cs);
    bool hasFastRefresh();
    bool hasPartialRefresh();
#ifndef __LINUX__
#ifdef ARDUINO
#ifdef SCK
    void initIO(int iDC, int iReset, int iBusy, int iCS = SS, int iMOSI = MOSI, int iSCLK = SCK, uint32_t u32Speed = 8000000);
#else // no SCK/MOSI default pins
    void initIO(int iDC, int iReset, int iBusy, int iCS, int iMOSI, int iSCLK, uint32_t u32Speed = 8000000);
#endif
#else // esp-idf?
    void initIO(int iDC, int iReset, int iBusy, int iCS, int iMOSI, int iSCLK, uint32_t u32Speed);
#endif // ARDUINO
#else // __LINUX__
    void initIO(int iDC, int iReset, int iBusy, int iCS, int iSPIChannel, uint32_t u32Speed = 8000000);
#endif
    int writePlane(int iPlane = PLANE_BOTH, bool bInvert = false);
    void startWrite(int iPlane);
    void writeData(uint8_t *pData, int iLen);
    void writeCmd(uint8_t u8Cmd);
    int refresh(int iMode, bool bWait = true);
    void setBuffer(uint8_t *pBuffer);
    int allocBuffer(bool bSecondPlane = true);
    void * getBuffer(void);
    uint8_t * getCache(void);
    void freeBuffer(void);
    uint32_t capabilities();
    void setRotation(int iAngle);
    int getRotation(void);
    void backupPlane(void);
    int getLastError(void);
    void drawRoundRect(int x, int y, int w, int h,
                       int r, uint8_t color);
    void fillRoundRect(int x, int y, int w, int h,
                       int r, uint8_t color);
    void fillScreen(int iColor, int iPlane = PLANE_DUPLICATE);
    void drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
    void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
    void setTextWrap(bool bWrap);
    void setTextColor(int iFG, int iBG = BBEP_TRANSPARENT);
    void setCursor(int x, int y);
    int loadBMP(const uint8_t *pBMP, int x, int y, int iFG, int iBG);
    int loadBMP3(const uint8_t *pBMP, int x, int y);
    int loadG5Image(const uint8_t *pG5, int x, int y, int iFG, int iBG, float fScale = 1.0f);
    void setFont(int iFont);
    void setFont(const void *pFont);
    void drawLine(int x1, int y1, int x2, int y2, int iColor);
    void drawPixel(int16_t x, int16_t y, uint8_t color);
    int16_t getCursorX(void);
    int16_t getCursorY(void);
    int testPanelType(void);
    void getStringBox(const char *string, BB_RECT *pRect);
#ifdef ARDUINO
    void getStringBox(const String &str, BB_RECT *pRect);
#endif
    int dataTime();
    int opTime();
    int16_t height(void);
    int16_t width(void);
    void drawCircle(int32_t x, int32_t y, int32_t r, uint32_t color);
    void fillCircle(int32_t x, int32_t y, int32_t r, uint32_t color);
    void drawEllipse(int16_t x, int16_t y, int32_t rx, int32_t ry, uint16_t color);
    void fillEllipse(int16_t x, int16_t y, int32_t rx, int32_t ry, uint16_t color);
    void stretchAndSmooth(uint8_t *pSrc, uint8_t *pDest, int w, int h, int iSmoothType);
    void sleep(int bDeep);
    void wake(void);
    void wait(bool bQuick = false);
    bool isBusy(void);
    void drawString(const char *pText, int x, int y);
    void setPlane(int iPlane);
    int getPlane(void);
    int getChip(void);
    void drawSprite(const uint8_t *pSprite, int cx, int cy, int iPitch, int x, int y, uint8_t iColor);    
#if !defined (ARDUINO)
    void print(const char *pString);
    void println(const char *pString);
    void print(int, int);
    void println(int, int);
    void print(const string &);
    void println(const string &);
    size_t write(uint8_t ucChar);
    void delayMicroseconds(int iTime);
#else
#ifndef __AVR__
    using Print::write;
    virtual size_t write(uint8_t);
#endif // __AVR__
#endif // !ARDUINO

  private:
    BBEPDISP _bbep;
    uint32_t _tar_memaddr   = 0x001236E0;
    uint16_t _dev_memaddr_l = 0x36E0;
    uint16_t _dev_memaddr_h = 0x0012;

}; // class BBEPAPER
#endif // __cplusplus

#if !defined(BITBANK_LCD_MODES)
#define BITBANK_LCD_MODES
typedef enum
{
 MODE_DATA = 0,
 MODE_COMMAND
} DC_MODE;
#endif

// Forward references to core C functions
void bbepWriteCmd(BBEPDISP *pBBEP, uint8_t cmd);
void bbepWriteData(BBEPDISP *pBBEP, uint8_t *pData, int iLen);
void bbepCMD2(BBEPDISP *pBBEP, uint8_t cmd1, uint8_t cmd2);
#endif // __BB_EPAPER__

