//
// bb_eink I/O wrapper functions for Raspberry Pi
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
// Adapt these functions to whatever target platform you're using
// and the rest of the code can remain unchanged
//
#ifndef __BB_RPI_IO__
#define __BB_RPI_IO__

#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h> 
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/spi/spidev.h>
#include <gpiod.h>
#include <math.h>
#include <time.h>
#ifndef CONSUMER
#define CONSUMER "Consumer"
#endif
#define pgm_read_byte(a) (*(uint8_t *)a)
#define pgm_read_word(a) (*(uint16_t *)a)
#define pgm_read_dword(a) (*(uint32_t *)a)
#define memcpy_P memcpy
struct gpiod_chip *chip = NULL;
#ifdef GPIOD_API
struct gpiod_line *lines[64];
#else
struct gpiod_line_request *lines[64];
#endif
static int spi_fd; // SPI handle

// forward references
void bbepWakeUp(BBEPDISP *pBBEP);
void bbepSendCMDSequence(BBEPDISP *pBBEP, const uint8_t *pSeq);

// Initialize the I2C bus on Linux
void I2CInit(BBI2C *pI2C, uint32_t iClock)
{
char filename[32];
int iChannel = pI2C->iSDA;

// Only try to initialize the handle if it hasn't already been done
    if (pI2C->file_i2c == -1) {
        sprintf(filename, "/dev/i2c-%d", iChannel);
        if ((pI2C->file_i2c = open(filename, O_RDWR)) < 0)
        {
                fprintf(stderr, "Failed to open the i2c bus\n");
        }
    }
} /* I2CInit() */
//
// Test if an I2C address responds
// returns 1 for success, 0 for failure
//
uint8_t I2CTest(BBI2C *pI2C, uint8_t addr)
{
uint8_t response = 0;
    if (ioctl(pI2C->file_i2c, I2C_SLAVE, addr) >= 0) {
            // probe this address
        uint8_t ucTemp;
        if (read(pI2C->file_i2c, &ucTemp, 1) >= 0)
            response = 1;
    }
    return response;
} /* I2CTest() */
//
// Read n bytes from the given I2C address
//
int I2CRead(BBI2C *pI2C, uint8_t iAddr, uint8_t *pData, int iLen)
{
int rc;
        ioctl(pI2C->file_i2c, I2C_SLAVE, iAddr);
        rc = read(pI2C->file_i2c, pData, iLen);
        return rc;
} /* I2CRead() */
//
// Write n bytes to the given address
//
int I2CWrite(BBI2C *pI2C, uint8_t iAddr, uint8_t *pData, int iLen)
{
int rc;
        ioctl(pI2C->file_i2c, I2C_SLAVE, iAddr);
        rc = write(pI2C->file_i2c, pData, iLen);
        return rc;
} /* I2CWrite() */
//
// Read n bytes from the given address, after setting the register number
//
int I2CReadRegister(BBI2C *pI2C, uint8_t iAddr, uint8_t u8Register, uint8_t *pData, int iLen)
{
int rc;
        // Reading from an I2C device involves first writing the 8-bit register
        // followed by reading the data
        ioctl(pI2C->file_i2c, I2C_SLAVE, iAddr);
        rc = write(pI2C->file_i2c, &u8Register, 1); // write the register value
        if (rc == 1)
        {
                rc = read(pI2C->file_i2c, pData, iLen);
        }
        return rc;

} /* I2CReadRegister() */

void SPI_transfer(BBEPDISP *pBBEP, uint8_t *pBuf, int iLen)
{
struct spi_ioc_transfer spi;
   memset(&spi, 0, sizeof(spi));
   spi.tx_buf = (unsigned long)pBuf;
   spi.len = iLen;
   spi.speed_hz = pBBEP->iSpeed;
   spi.bits_per_word = 8;
   ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi);
} /* SPI_transfer() */

int digitalRead(int iPin)
{
	if (lines[iPin] == 0) return 0;
#ifdef GPIOD_API // 1.x (old) API
  return gpiod_line_get_value(lines[iPin]);
#else // 2.x (new)
  return gpiod_line_request_get_value(lines[iPin], iPin) == GPIOD_LINE_VALUE_ACTIVE;
#endif
} /* digitalRead() */

void digitalWrite(int iPin, int iState)
{
	if (lines[iPin] == 0) return;
#ifdef GPIOD_API // old 1.6 API
   gpiod_line_set_value(lines[iPin], iState);
#else // new 2.x API
   gpiod_line_request_set_value(lines[iPin], iPin, (iState) ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE);
#endif
} /* digitalWrite() */

void pinMode(int iPin, int iMode)
{
#ifdef GPIOD_API // old 1.6 API
   if (chip == NULL) {
       chip = gpiod_chip_open_by_name("gpiochip0");
   }
   lines[iPin] = gpiod_chip_get_line(chip, iPin);
   if (iMode == OUTPUT) {
       gpiod_line_request_output(lines[iPin], CONSUMER, 0);
   } else if (iMode == INPUT_PULLUP) {
       gpiod_line_request_input_flags(lines[iPin], CONSUMER, GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP);
   } else { // plain input
       gpiod_line_request_input(lines[iPin], CONSUMER);
   }
#else // new 2.x API
   struct gpiod_line_settings *settings;
   struct gpiod_line_config *line_cfg;
   struct gpiod_request_config *req_cfg;
   chip = gpiod_chip_open("/dev/gpiochip0");
   if (!chip) {
	printf("chip open failed\n");
	   return;
   }
   settings = gpiod_line_settings_new();
   if (!settings) {
	printf("line_settings_new failed\n");
	   return;
   }
   gpiod_line_settings_set_direction(settings, (iMode == OUTPUT) ? GPIOD_LINE_DIRECTION_OUTPUT : GPIOD_LINE_DIRECTION_INPUT);
   line_cfg = gpiod_line_config_new();
   if (!line_cfg) return;
   gpiod_line_config_add_line_settings(line_cfg, (const unsigned int *)&iPin, 1, settings);
   req_cfg = gpiod_request_config_new();
   gpiod_request_config_set_consumer(req_cfg, CONSUMER);
   lines[iPin] = gpiod_chip_request_lines(chip, req_cfg, line_cfg);
   gpiod_request_config_free(req_cfg);
   gpiod_line_config_free(line_cfg);
   gpiod_line_settings_free(settings);
   gpiod_chip_close(chip);
#endif
} /* pinMode() */

void delay(int iMS)
{
  usleep(iMS * 1000);
} /* delay() */

long millis(void)
{
int iTime;
struct timespec res;

    clock_gettime(CLOCK_MONOTONIC, &res);
    iTime = 1000*res.tv_sec + res.tv_nsec/1000000;

    return (long)iTime;
} /* millis() */

#ifdef FUTURE
static void delayMicroseconds(int iMS)
{
  usleep(iMS);
} /* delayMicroseconds() */
#endif // FUTURE

//
// Initialize the GPIO pins and SPI for use by bb_eink
//
void bbepInitIO(BBEPDISP *pBBEP, uint32_t u32Speed)
{
char szName[32];

    pinMode(pBBEP->iDCPin, OUTPUT);
    pinMode(pBBEP->iRSTPin, OUTPUT);
    digitalWrite(pBBEP->iRSTPin, LOW);
    delay(100);
    digitalWrite(pBBEP->iRSTPin, HIGH);
    delay(100);
    if (pBBEP->iBUSYPin != 0xff) {
        pinMode(pBBEP->iBUSYPin, INPUT);
    }
    pBBEP->iSpeed = u32Speed;
    pinMode(pBBEP->iCSPin, OUTPUT);
    digitalWrite(pBBEP->iCSPin, HIGH); // we have to manually control the CS pin
    sprintf(szName, "/dev/spidev%d.0", pBBEP->iMOSIPin); // SPI channel #
    spi_fd = open(szName, O_RDWR);
    //spi_fd = open("/dev/spidev0.1", O_RDWR); // DEBUG - open SPI channel 0
// Before we can start sending pixels, many panels need to know the display resolution
    bbepSendCMDSequence(pBBEP, pBBEP->pInitFull);
} /* bbepInitIO() */
//
// Convenience function to write a command byte along with a data
// byte (it's single parameter)
//
void bbepCMD2(BBEPDISP *pBBEP, uint8_t cmd1, uint8_t cmd2)
{
    if (!pBBEP->is_awake) {
        // if it's asleep, it can't receive commands
        bbepWakeUp(pBBEP);
        pBBEP->is_awake = 1;
    }
    digitalWrite(pBBEP->iDCPin, LOW);
    digitalWrite(pBBEP->iCSPin, LOW);
    SPI_transfer(pBBEP, &cmd1, 1);
    digitalWrite(pBBEP->iDCPin, HIGH);
    SPI_transfer(pBBEP, &cmd2, 1); // second byte is data
    digitalWrite(pBBEP->iCSPin, HIGH);
    digitalWrite(pBBEP->iDCPin, HIGH); // leave data mode as the default
} /* bbepCMD2() */
//
// Set the second CS pin for dual-controller displays
//
void bbepSetCS2(BBEPDISP *pBBEP, uint8_t cs)
{
    pBBEP->iCS1Pin = pBBEP->iCSPin;
    pBBEP->iCS2Pin = cs;
    pinMode(cs, OUTPUT);
    digitalWrite(cs, HIGH); // disable second CS for now
} /* bbepSetCS2() */

//
// Write a single byte as a COMMAND (D/C set low)
//
void bbepWriteCmd(BBEPDISP *pBBEP, uint8_t cmd)
{
    if (!pBBEP->is_awake) {
        // if it's asleep, it can't receive commands
        bbepWakeUp(pBBEP);
        pBBEP->is_awake = 1;
    }
    digitalWrite(pBBEP->iDCPin, LOW);
    digitalWrite(pBBEP->iCSPin, LOW);
    SPI_transfer(pBBEP, &cmd, 1);
    digitalWrite(pBBEP->iCSPin, HIGH);
    digitalWrite(pBBEP->iDCPin, HIGH); // leave data mode as the default
} /* bbepWriteCmd() */
//
// Write 1 or more bytes as DATA (D/C set high)
//
void bbepWriteData(BBEPDISP *pBBEP, uint8_t *pData, int iLen)
{
    digitalWrite(pBBEP->iCSPin, LOW);
    SPI_transfer(pBBEP, pData, iLen);
    digitalWrite(pBBEP->iCSPin, HIGH);
} /* bbepWriteData() */

#endif // __BB_RPI_IO__
