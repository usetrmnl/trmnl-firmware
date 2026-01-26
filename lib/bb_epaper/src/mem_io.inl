//
// bb_eink I/O stub functions for memory-only use
// Written by Larry Bank (bitbank@pobox.com)
// Project started 11/17/2025
//
// SPDX-FileCopyrightText: 2025 BitBank Software, Inc.
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
#ifndef __BB_EP_IO__
#define __BB_EP_IO__

#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h> 
#include <string.h>
#define pgm_read_byte(a) (*(uint8_t *)a)
#define pgm_read_word(a) (*(uint16_t *)a)
#define pgm_read_dword(a) (*(uint32_t *)a)
#define memcpy_P memcpy

// forward references
void bbepWakeUp(BBEPDISP *pBBEP);
void bbepSendCMDSequence(BBEPDISP *pBBEP, const uint8_t *pSeq);

void SPI_transfer(BBEPDISP *pBBEP, uint8_t *pBuf, int iLen)
{
    (void)pBBEP;
    (void)pBuf;
    (void)iLen;
} /* SPI_transfer() */

int digitalRead(int iPin)
{
    (void)iPin;
    return 0;
} /* digitalRead() */

void digitalWrite(int iPin, int iState)
{
    (void)iPin;
    (void)iState;
} /* digitalWrite() */

void pinMode(int iPin, int iMode)
{
   (void)iPin;
   (void)iMode;
} /* pinMode() */

void delay(int iMS)
{
  (void)iMS;
} /* delay() */

long millis(void)
{
   return 0;
} /* millis() */

//
// Initialize the GPIO pins and SPI for use by bb_eink
//
void bbepInitIO(BBEPDISP *pBBEP, int iDC, int iReset, int iBusy, int iCS, int iMOSI, int iSCK, uint32_t u32Speed)
{
    pBBEP->iSpeed = u32Speed;
    (void)iDC; (void)iReset; (void)iBusy; (void)iCS; (void)iMOSI; (void)iSCK;
} /* bbepInitIO() */
//
// Convenience function to write a command byte along with a data
// byte (it's single parameter)
//
void bbepCMD2(BBEPDISP *pBBEP, uint8_t cmd1, uint8_t cmd2)
{
    pBBEP->is_awake = 1;
    (void)cmd1;
    (void)cmd2;
} /* bbepCMD2() */
//
// Set the second CS pin for dual-controller displays
//
void bbepSetCS2(BBEPDISP *pBBEP, uint8_t cs)
{
    pBBEP->iCS1Pin = pBBEP->iCSPin;
    pBBEP->iCS2Pin = cs;
} /* bbepSetCS2() */

//
// Write a single byte as a COMMAND (D/C set low)
//
void bbepWriteCmd(BBEPDISP *pBBEP, uint8_t cmd)
{
    (void)cmd;
    pBBEP->is_awake = 1;
} /* bbepWriteCmd() */
//
// Write 1 or more bytes as DATA (D/C set high)
//
void bbepWriteData(BBEPDISP *pBBEP, uint8_t *pData, int iLen)
{
    (void)pBBEP;
    (void)pData;
    (void)iLen;
} /* bbepWriteData() */

#endif // __BB_EP_IO__
