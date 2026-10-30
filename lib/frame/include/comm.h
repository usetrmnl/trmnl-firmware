//=================================================================================================
//                   Communication interface for 31.5" Control Board
//
// File Name : comm.h
// Author : Electronic Design Dept. II
// Data : 2023.11.13
// Version : 1.0
// Copyright : E Ink Holdings Inc.
//=================================================================================================

#ifndef __COMM_H__
#define __COMM_H__


#endif //#ifndef __COMM_H__


#ifdef __COMM_C__
#define __COMM_EXTERN__
#else
#define __COMM_EXTERN__ extern
#endif

// Delay function
__COMM_EXTERN__ void delayms(unsigned int delayTime);

// SPI communication functions
__COMM_EXTERN__ void spiInit(void);
__COMM_EXTERN__ unsigned char spiTransmitCommand(unsigned char commandBuf);
__COMM_EXTERN__ unsigned char spiTransmitData(unsigned char *dataBuffer, unsigned long dataLength);
__COMM_EXTERN__ unsigned char spiTransmitLargeData(unsigned char commandBuf, unsigned char *dataBuffer, unsigned long dataLength);
__COMM_EXTERN__ unsigned char spiTransmit(unsigned char commandBuf, unsigned char *dataBuffer, unsigned int dataLength);
__COMM_EXTERN__ unsigned char spiReceive(unsigned char commandBuf, unsigned char *dataBuffer, unsigned int dataLength);

// I2C communication functions
__COMM_EXTERN__ unsigned char i2cTransmitData(unsigned char i2cAddress, unsigned char *dataBuffer, unsigned int dataLength);
__COMM_EXTERN__ unsigned char i2cReceiveData(unsigned char i2cAddress, unsigned char *dataBuffer, unsigned int dataLength);

// GPIO functions
__COMM_EXTERN__ void setGpioLevel(unsigned char pinNumber, unsigned char voltageLevel);
__COMM_EXTERN__ unsigned char getGpioLevel(unsigned char pinNumber);

__COMM_EXTERN__ void set_ESP32_GpioLevel(uint8_t pinState);
__COMM_EXTERN__ void setGpioLevel_EXPANDER_2(unsigned char pinNumber, unsigned char voltageLevel);