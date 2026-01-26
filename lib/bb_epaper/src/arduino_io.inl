//
// bb_epaper I/O wrapper functions for Arduino
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
#ifndef __ARDUINO_IO__
#define __ARDUINO_IO__

#include <Arduino.h>
#include <SPI.h>
// foreward references
void bbepWakeUp(BBEPDISP *pBBEP);
void bbepSendCMDSequence(BBEPDISP *pBBEP, const uint8_t *pSeq);
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

void SPI_Write(BBEPDISP *pBBEP, uint8_t *pData, int iLen)
{
int i, j;
uint8_t uc;

    for (i=0; i<iLen; i++) {
        uc = *pData++;
        for (j=0; j<8; j++) {
            digitalWrite(pBBEP->iCLKPin, 0);
            digitalWrite(pBBEP->iMOSIPin, uc & 0x80);
            digitalWrite(pBBEP->iCLKPin, 1);
            uc <<= 1;
        }
    }
} /* SPI_Write() */
//
// Initialize the GPIO pins and SPI for use by bb_eink
//
void bbepInitIO(BBEPDISP *pBBEP, uint8_t u8DC, uint8_t u8RST, uint8_t u8BUSY, uint8_t u8CS, uint8_t u8MOSI, uint8_t u8SCK, uint32_t u32Speed)
{
    pBBEP->iDCPin = u8DC;
    pBBEP->iCSPin = u8CS;
    pBBEP->iMOSIPin = u8MOSI;
    pBBEP->iCLKPin = u8SCK;
    pBBEP->iRSTPin = u8RST;
    pBBEP->iBUSYPin = u8BUSY;

    pinMode(pBBEP->iDCPin, OUTPUT);
    if (pBBEP->iRSTPin != 0xff) {
        pinMode(pBBEP->iRSTPin, OUTPUT);
        digitalWrite(pBBEP->iRSTPin, LOW);
        delay(100);
        digitalWrite(pBBEP->iRSTPin, HIGH);
        delay(100);
    }
    if (pBBEP->iBUSYPin != 0xff) {
        pinMode(pBBEP->iBUSYPin, INPUT);
    }
    pBBEP->iSpeed = u32Speed;
    pinMode(pBBEP->iCSPin, OUTPUT);
    digitalWrite(pBBEP->iCSPin, HIGH); // manually control the CS pin
    if (u32Speed == 0) { // bit bang mode
        pinMode(pBBEP->iMOSIPin, OUTPUT);
        pinMode(pBBEP->iCLKPin, OUTPUT);
    } else {
        if (u8MOSI != 0xff) { // Not shared SPI?
#ifdef ARDUINO_ARCH_ESP32
            SPI.begin(pBBEP->iCLKPin, -1, pBBEP->iMOSIPin, -1); //pBBEP->iCSPin);
#else
            SPI.begin(); // other architectures have fixed SPI pins
#endif
            SPI.beginTransaction(SPISettings(u32Speed, MSBFIRST, SPI_MODE0));
#ifdef ARDUINO_ARCH_ESP32
// For NRF52, you have to leave an 'open' transaction
            SPI.endTransaction(); // N.B. - if you call beginTransaction() again without a matching endTransaction(), it will hang on ESP32
#endif
        }
    }
    pBBEP->is_awake = 1;
// Before we can start sending pixels, many panels need to know the display resolution
    bbepSendCMDSequence(pBBEP, pBBEP->pInitFull);
    if (pBBEP->iFlags & BBEP_7COLOR) { // need to send before you can send it data
        if (pBBEP->iFlags & BBEP_SPLIT_BUFFER) {    
           // Send the same sequence to the second controller
           pBBEP->iCSPin = pBBEP->iCS2Pin;
           bbepSendCMDSequence(pBBEP, pBBEP->pInitFull);
           pBBEP->iCSPin = pBBEP->iCS1Pin;
        }
    }
} /* bbepInitIO() */

void bbepWriteIT8951Cmd(BBEPDISP *pBBEP, uint16_t cmd)
{
    uint8_t ucTemp[4];
    ucTemp[0] = 0x60; // command introducer = 0x6000
    ucTemp[1] = 0;
    ucTemp[2] = (uint8_t)(cmd >> 8);
    ucTemp[3] = (uint8_t)cmd;
    digitalWrite(pBBEP->iCSPin, LOW);
#ifdef ARDUINO_ARCH_ESP32
    SPI.transferBytes(ucTemp, NULL, 4);
#else
    SPI.transfer(ucTemp, 4);
#endif
    digitalWrite(pBBEP->iCSPin, HIGH);
} /* bbepWriteIT8951Cmd() */

void bbepWriteIT8951Data(BBEPDISP *pBBEP, uint8_t *pData, int iLen)
{
    digitalWrite(pBBEP->iCSPin, LOW);
    SPI.transfer(0);
    SPI.transfer(0); // 0x0000 is the data introducer
#ifdef ARDUINO_ARCH_ESP32
    SPI.transferBytes(pData, NULL, iLen);
#else
    SPI.transfer(pData, iLen);
#endif
    digitalWrite(pBBEP->iCSPin, HIGH);
} /* bbepWriteIT8951Data() */

void bbepWriteIT8951CmdArgs(BBEPDISP *pBBEP, uint16_t cmd, uint16_t *pArgs, int iCount)
{
    bbepWriteIT8951Cmd(pBBEP, cmd);
    for (int i=0; i<iCount; i++) {
        pArgs[i] = __builtin_bswap16(pArgs[i]);
    }
    bbepWriteIT8951Data(pBBEP, (uint8_t *)pArgs, iCount * 2);
} /* bbepWriteIT8951CmdArgs() */
//
// Write a single byte as a COMMAND (D/C set low)
//
void bbepWriteCmd(BBEPDISP *pBBEP, uint8_t cmd)
{
//Serial.printf("CMD: 0x%02x\n", cmd);

    if (!pBBEP->is_awake) {
        // if it's asleep, it can't receive commands
        bbepWakeUp(pBBEP);
        pBBEP->is_awake = 1;
    }
    digitalWrite(pBBEP->iDCPin, LOW);
    delay(1);
    digitalWrite(pBBEP->iCSPin, LOW);
    if (pBBEP->iSpeed == 0) { // bit bang
        SPI_Write(pBBEP, &cmd, 1);
    } else {
        SPI.transfer(cmd);
    }
    digitalWrite(pBBEP->iCSPin, HIGH);
    digitalWrite(pBBEP->iDCPin, HIGH); // leave data mode as the default
} /* bbepWriteCmd() */
//
// Write 1 or more bytes as DATA (D/C set high)
//
void bbepWriteData(BBEPDISP *pBBEP, uint8_t *pData, int iLen)
{
//if (iLen < 5) {
//  Serial.print("DATA: ");
//  for (int i=0; i<iLen; i++) {
//     Serial.printf("0x%02x, ", pData[i]);
//  }
//  Serial.println(" ");
//} else {
//  Serial.printf("DATA: len=%d\n", iLen);
//}
//    digitalWrite(pBBEP->iDCPin, HIGH);
#ifdef ARDUINO_ARCH_ESP32
    if (pBBEP->iFlags & BBEP_CS_EVERY_BYTE) {
        for (int i=0; i<iLen; i++) {
            digitalWrite(pBBEP->iCSPin, LOW);
            if (pBBEP->iSpeed == 0) { // bit bang
                SPI_Write(pBBEP, &pData[i], 1);
            } else {
                SPI.transfer(pData[i]);
            }
            digitalWrite(pBBEP->iCSPin, HIGH);
        }
    } else {
        digitalWrite(pBBEP->iCSPin, LOW);
        if (pBBEP->iSpeed == 0) { // bit bang
            SPI_Write(pBBEP, pData, iLen);
        } else {
            SPI.transferBytes(pData, NULL, iLen);
        }
        digitalWrite(pBBEP->iCSPin, HIGH);
    }
#else
    if (pBBEP->iFlags & BBEP_CS_EVERY_BYTE) {
        for (int i=0; i<iLen; i++) { // Arduino clobbers the data (duplex)
            digitalWrite(pBBEP->iCSPin, LOW);
            if (pBBEP->iSpeed == 0) { // bit bang
                SPI_Write(pBBEP, &pData[i], 1);
            } else {
                SPI.transfer(pData[i]);
            }
            digitalWrite(pBBEP->iCSPin, HIGH);
        }
    } else {
        digitalWrite(pBBEP->iCSPin, LOW);
        for (int i=0; i<iLen; i++) { // Arduino clobbers the data (duplex)
            if (pBBEP->iSpeed == 0) { // bit bang
                SPI_Write(pBBEP, &pData[i], 1);
            } else {
                SPI.transfer(pData[i]);
            }
        }
        digitalWrite(pBBEP->iCSPin, HIGH);
    }
#endif
} /* bbepWriteData() */

//
// Convenience function to write a command byte along with a data
// byte (it's single parameter)
//
void bbepCMD2(BBEPDISP *pBBEP, uint8_t cmd1, uint8_t cmd2)
{
    bbepWriteCmd(pBBEP, cmd1);
    bbepWriteData(pBBEP, &cmd2, 1);
} /* bbepCMD2() */

#endif // __ARDUINO_IO__
