/*****************************************************************************
* | File      	:   DEV_Config.h
* | Author      :   Waveshare team
* | Function    :   Hardware underlying interface
* | Info        :
*----------------
* |	This version:   V1.0
* | Date        :   2020-02-19
* | Info        :
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documnetation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to  whom the Software is
# furished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
******************************************************************************/
#ifndef _DEV_CONFIG_H_
#define _DEV_CONFIG_H_

#include <Arduino.h>
#include <stdint.h>
#include <stdio.h>

// TRMNL Device structure - defines the GPIO connections for the display, button and battery
// Only needed on "OG" class devices with SPI ePaper displays
typedef struct tag_trmnl_device
{
   const char *device_name;
   uint8_t epd_sck_pin;
   uint8_t epd_mosi_pin;
   uint8_t epd_cs_pin;
   uint8_t epd_rst_pin;
   uint8_t epd_dc_pin;
   uint8_t epd_busy_pin;
   uint8_t sensor_sda;
   uint8_t sensor_scl;
   uint8_t interrupt_pin;
   uint8_t battery_pin;
   uint8_t panel_set;
} TRMNL_DEVICE;

// This enum defines sets of display configurations for different SPI panel types
// These sets are what define the temperature profile - 3 per size (default, A, B)
enum {
  EPD_75 = 0,
  EPD_426,
  EPD_397,
  EPD_75_3CLR,
  EPD_75_4CLR,
  EPD_75_6CLR,
  EPD_CROWPANEL,
};

/**
 * data
**/
#define UBYTE   uint8_t
#define UWORD   uint16_t
#define UDOUBLE uint32_t

#if defined (BOARD_SEEED_RETERMINAL_E1003)
   #define EPD_SCK_PIN  7
   #define EPD_MOSI_PIN 9
   #define EPD_MISO_PIN 8
   #define EPD_CS_PIN   10
   #define EPD_RST_PIN  12
   #define EPD_EN_PIN   11
   #define EPD_BUSY_PIN 13
   #define EPD_VCC_EN   21
#endif

#define GPIO_PIN_SET   1

/**
 * GPIO read and write
**/
#define DEV_Digital_Write(_pin, _value) digitalWrite(_pin, _value == 0? LOW:HIGH)
#define DEV_Digital_Read(_pin) digitalRead(_pin)

/**
 * delay x ms
**/
#define DEV_Delay_ms(__xms) delay(__xms)

/*------------------------------------------------------------------------------------------------------*/
UBYTE DEV_Module_Init(void);
void DEV_SPI_WriteByte(UBYTE data);

#endif
