#pragma once
#include <stdint.h>

// EPD 4bpp color nibble values (ACeP 6-color)
#define EPD_BLACK   0x00
#define EPD_WHITE   0x11
#define EPD_YELLOW  0x22
#define EPD_RED     0x33
#define EPD_BLUE    0x55
#define EPD_GREEN   0x66

// Each driver IC handles 800 × 720 pixels at 4 bpp = 288 000 bytes
#define EPD_IMAGE_DATA_BUFFER   288000
#define EPD_NUM_DRIVERS         8

// E2803 SPI command bytes
#define EPD_CMD_PSR         0x00
#define EPD_CMD_PWR         0x01
#define EPD_CMD_POF         0x02
#define EPD_CMD_PON         0x04
#define EPD_CMD_DTM         0x10
#define EPD_CMD_DRF         0x12
#define EPD_CMD_CDI         0x50
#define EPD_CMD_TCON        0x60
#define EPD_CMD_TRES        0x61
#define EPD_CMD_TSC         0x70
#define EPD_CMD_PWS         0xE3
#define EPD_CMD_SPIM        0xE7
#define EPD_CMD_VCOM_WOUT_EN 0x02
#define EPD_CMD_TM_TCON     0xE0
#define EPD_CMD_CMD66       0x66
#define EPD_CMD_EN_BUF      0xE5
#define EPD_CMD_CCSET       0xE6

#ifdef __cplusplus
extern "C" {
#endif

void          epdInit(void);
void          epdHardwareReset(void);
void          epdSetCsAll(unsigned int level);
void          epdSetCs(unsigned char cs, unsigned int level);
void          epdCheckBusyHigh(void);
void          epdCheckBusyLow(void);
void          epdWrite(unsigned char cmd, const unsigned char *data, unsigned int len);
void          epdRead(unsigned char cmd, unsigned char *data, unsigned int len);
void          epdWriteCommand(unsigned char cmd);
void          epdWriteData(const unsigned char *data, unsigned int len);
void          epdWriteImage(unsigned char csx, const unsigned char *imageData, unsigned long len);
void          epdDisplay(void);
unsigned char epdSetPower(void);
unsigned char epdCheckDriverICStatus(void);

#ifdef __cplusplus
}
#endif
