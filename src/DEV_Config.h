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

/**
 * data
**/
#define UBYTE   uint8_t
#define UWORD   uint16_t
#define UDOUBLE uint32_t

/**
 * GPIO config
**/
#if defined(BOARD_TRMNL)
// Xiao ESP32C3 plus 8-pin breakout
//   #define EPD_SCK_PIN  8
//   #define EPD_MOSI_PIN 10
//   #define EPD_CS_PIN   21
//   #define EPD_RST_PIN  5
//   #define EPD_DC_PIN   4
//   #define EPD_BUSY_PIN 20
// Pin def for Xiao ESP32C3 + old EPD breakout
//   #define EPD_SCK_PIN  8
//   #define EPD_MOSI_PIN 10
//   #define EPD_CS_PIN   3
//   #define EPD_RST_PIN  2
//   #define EPD_DC_PIN   5
//   #define EPD_BUSY_PIN 7
   // Pin definition for TRMNL Board
  #define EPD_SCK_PIN  7
  #define EPD_MOSI_PIN 8
  #define EPD_CS_PIN   6
  #define EPD_RST_PIN  10
  #define EPD_DC_PIN   5
  #define EPD_BUSY_PIN 4

#elif defined(BOARD_WAVESHARE_ESP32_DRIVER)
   // Pin definition for Waveshare ESP32 Driver Board
   #define EPD_SCK_PIN  13
   #define EPD_MOSI_PIN 14
   #define EPD_CS_PIN   15
   #define EPD_RST_PIN  26
   #define EPD_DC_PIN   27
   #define EPD_BUSY_PIN 25

#elif defined(BOARD_SEEED_XIAO_ESP32C3)
   // Pin definition for Seeed XIAO ESP32C3 Board
   #define EPD_SCK_PIN  8
   #define EPD_MOSI_PIN 10
   #define EPD_CS_PIN   3
   #define EPD_RST_PIN  2
   #define EPD_DC_PIN   5
   #define EPD_BUSY_PIN 4

#elif defined(BOARD_SEEED_XIAO_ESP32S3)
   // Pin definition for Seeed XIAO ESP32S3 Board
   #define EPD_SCK_PIN  7
   #define EPD_MOSI_PIN 9
   #define EPD_CS_PIN   2
   #define EPD_RST_PIN  1
   #define EPD_DC_PIN   4
   #define EPD_BUSY_PIN 3
#else
   #error "Board type not defined. Please define BOARD_WAVESHARE_ESP32_DRIVER or BOARD_TRMNL or BOARD_SEEED_XIAO_ESP32C3 or BOARD_SEEED_XIAO_ESP32S3 in platformio.ini build_flags."
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
