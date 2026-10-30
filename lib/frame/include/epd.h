//=================================================================================================
//                   E2803 EPD Driver for 31.5" Control Board (ESP32-C5)
//
// File Name : epd.h
// Based on : EL351TW1.h from E Ink sample code
// Version : 1.0
//=================================================================================================

#ifndef __EPD_H__
#define __EPD_H__

// Color definitions (4bpp - each pixel uses 4 bits)
#define EPD_BLACK       0x00
#define EPD_WHITE       0x11
#define EPD_YELLOW      0x22
#define EPD_RED         0x33
#define EPD_BLUE        0x55
#define EPD_GREEN       0x66

// E2803 Command definitions
#define EPD_CMD_PSR             0x00    // Panel Setting Register
#define EPD_CMD_PWR             0x01    // Power Setting
#define EPD_CMD_POF             0x02    // Power OFF
#define EPD_CMD_PON             0x04    // Power ON
#define EPD_CMD_DTM             0x10    // Data Transfer Mode (image data)
#define EPD_CMD_DRF             0x12    // Display Refresh
#define EPD_CMD_TSC             0x40    // Temperature Sensor Calibration
#define EPD_CMD_CDI             0x50    // VCOM and Data Interval
#define EPD_CMD_TCON            0x60    // TCON Setting
#define EPD_CMD_TRES            0x61    // Resolution Setting
#define EPD_CMD_CMD66           0xF0    // Cascade Setting
#define EPD_CMD_VCOM_WOUT_EN    0xB4    // VCOM Write Out Enable
#define EPD_CMD_EN_BUF          0xB6    // Enable Buffer
#define EPD_CMD_TM_TCON         0xD2    // Timing TCON
#define EPD_CMD_CCSET           0xE0    // Cascade Setting
#define EPD_CMD_PWS             0xE3    // Power Saving
#define EPD_CMD_SPIM            0xE6    // SPI Mode

// Data packet flags
#define FIRST_DATA_PACKET       1
#define NOT_FIRST_DATA_PACKET   0

// Image buffer size for one driver IC section (800×720 @ 4bpp = 288,000 bytes)
#define EPD_IMAGE_DATA_BUFFER   288000 // 288000

// Number of driver ICs (chip selects)
#define EPD_NUM_DRIVERS         8

#ifdef __cplusplus
extern "C" {
#endif

// Image data buffer (shared across sections) - defined in epd.cpp
extern unsigned char *epdImageDataBuffer;

// Hardware control functions
void epdHardwareReset(void);
void epdSetCsAll(unsigned int setLevel);
void epdSetCs(unsigned char csNumber, unsigned int setLevel);
void epdCheckBusyHigh(void);
void epdCheckBusyLow(void);

// Initialization functions
void epdInit(void);
unsigned char epdCheckDriverICStatus(void);

// SPI communication functions
void epdWrite(unsigned char epdCommand, const unsigned char *epdData, unsigned int epdDataLength);
void epdRead(unsigned char epdCommand, unsigned char *epdData, unsigned int epdDataLength);
void epdWriteCommand(unsigned char epdCommand);
void epdWriteData(const unsigned char *epdData, unsigned int epdDataLength);

// Power and display functions
unsigned char epdSetPower(void);
void epdDisplay(void);
void epdDisplayColor(unsigned char colorSelect);
void epdDisplayColorBar(void);
void epdWriteImage(void *pFramebuffer);
//void epdWriteImage(unsigned char csx, const unsigned char *imageData, unsigned long imageDataLength);

#ifdef __cplusplus
}
#endif

#endif //#ifndef __EPD_H__
