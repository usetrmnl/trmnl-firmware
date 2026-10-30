//=================================================================================================
//                   Communication interface for 31.5" Control Board
//
// File Name : comm.cpp
// Author : Electronic Design Dept. II
// Data : 2023.11.13
// Version : 1.0
// Copyright : E Ink Holdings Inc.
//=================================================================================================

#define __COMM_C__

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "pindefine.h"
#include "comm.h"

#include "TCA9555.h"

// Global TCA9555 expander objects
// These are instantiated in main.cpp and available to all modules
extern TCA9555 expander1;  // First expander at address 0x20
extern TCA9555 expander2;  // Second expander at address 0x21

// Global interrupt flags
extern volatile bool expander1_interrupt;
extern volatile bool expander2_interrupt;

// SPI configuration for ESP32-C5
// Using hardware SPI with pins defined in pindefine.h
#define EPD_SPI_CLOCK   SPI_CLK     // GPIO 5
#define EPD_SPI_MOSI    SPI_Data0   // GPIO 9
#define EPD_SPI_MISO    SPI_Data1   // GPIO 10

// SPI settings
static SPISettings spiSettings(4000000, MSBFIRST, SPI_MODE0);  // 4MHz, MSB first, Mode 0
static bool spiInitialized = false;

/**
 * @brief Initialize SPI for EPD communication
 */
void spiInit(void)
{
    if (!spiInitialized) {
        // Initialize SPI with custom pins for ESP32-C5
        SPI.begin(EPD_SPI_CLOCK, EPD_SPI_MISO, EPD_SPI_MOSI, -1);  // -1 = no hardware CS
        spiInitialized = true;
        printf("SPI initialized: CLK=%d, MOSI=%d, MISO=%d\n",
               EPD_SPI_CLOCK, EPD_SPI_MOSI, EPD_SPI_MISO);
    }
}

//=================================================================================================
// I2C Communication Functions
//=================================================================================================

/**
 * @brief Transmit data via I2C
 * @param i2cAddress: 7-bit I2C slave address
 * @param dataBuffer: Pointer to data buffer to transmit
 * @param dataLength: Number of bytes to transmit
 * @return 1 on success, 0 on failure
 */
unsigned char i2cTransmitData(unsigned char i2cAddress, unsigned char *dataBuffer, unsigned int dataLength)
{
    if (dataBuffer == NULL || dataLength == 0) {
        return 0; // Invalid parameters
    }

    Wire.beginTransmission(i2cAddress);

    // Write all bytes from the buffer
    for (unsigned int i = 0; i < dataLength; i++) {
        Wire.write(dataBuffer[i]);
    }

    // endTransmission returns:
    // 0: success
    // 1: data too long to fit in transmit buffer
    // 2: received NACK on transmit of address
    // 3: received NACK on transmit of data
    // 4: other error
    unsigned char error = Wire.endTransmission();

    return (error == 0) ? 1 : 0; // Return 1 on success, 0 on failure
}

/**
 * @brief Receive data via I2C
 * @param i2cAddress: 7-bit I2C slave address
 * @param dataBuffer: Pointer to buffer to store received data
 * @param dataLength: Number of bytes to receive
 * @return 1 on success, 0 on failure
 */
unsigned char i2cReceiveData(unsigned char i2cAddress, unsigned char *dataBuffer, unsigned int dataLength)
{
    if (dataBuffer == NULL || dataLength == 0) {
        return 0; // Invalid parameters
    }

    // Request bytes from I2C slave
    unsigned char bytesReceived = Wire.requestFrom(i2cAddress, (uint8_t)dataLength);

    if (bytesReceived != dataLength) {
        return 0; // Didn't receive expected number of bytes
    }

    // Read all received bytes into buffer
    for (unsigned int i = 0; i < dataLength; i++) {
        if (Wire.available()) {
            dataBuffer[i] = Wire.read();
        } else {
            return 0; // No more data available
        }
    }

    return 1; // Success
}

//=================================================================================================
// Delay Function
//=================================================================================================

/**
 * @brief Delay in milliseconds
 * @param delayTime: Delay time in milliseconds
 */
void delayms(unsigned int delayTime)
{
    delay(delayTime);
}

//=================================================================================================
// SPI Communication Functions for E2803 EPD Driver
//=================================================================================================

/**
 * @brief Transmit a command byte via SPI
 * @param commandBuf: Command byte to send
 * @return 1 on success
 */
unsigned char spiTransmitCommand(unsigned char commandBuf)
{
    if (!spiInitialized) spiInit();

    SPI.beginTransaction(spiSettings);
    SPI.transfer(commandBuf);
    SPI.endTransaction();

    return 1;
}

/**
 * @brief Transmit data bytes via SPI
 * @param dataBuffer: Pointer to data buffer
 * @param dataLength: Number of bytes to send
 * @return 1 on success
 */
unsigned char spiTransmitData(unsigned char *dataBuffer, unsigned long dataLength)
{
    if (!spiInitialized) spiInit();
    if (dataBuffer == NULL || dataLength == 0) return 0;

    SPI.beginTransaction(spiSettings);
    for (unsigned long i = 0; i < dataLength; i++) {
        SPI.transfer(dataBuffer[i]);
    }
    SPI.endTransaction();

    return 1;
}

/**
 * @brief Transmit command followed by large data buffer
 * @param commandBuf: Command byte
 * @param dataBuffer: Pointer to data buffer
 * @param dataLength: Number of data bytes
 * @return 1 on success
 */
unsigned char spiTransmitLargeData(unsigned char commandBuf, unsigned char *dataBuffer, unsigned long dataLength)
{
    if (!spiInitialized) spiInit();
    if (dataBuffer == NULL || dataLength == 0) return 0;

    SPI.beginTransaction(spiSettings);

    // Send command
    SPI.transfer(commandBuf);

    // Send data in chunks for better performance
    const unsigned long chunkSize = 4096;
    unsigned long remaining = dataLength;
    unsigned long offset = 0;

    while (remaining > 0) {
        unsigned long bytesToSend = (remaining > chunkSize) ? chunkSize : remaining;
        // Use writeBytes() instead of transfer() - transfer() is bidirectional and
        // overwrites the source buffer with MISO data, corrupting image data.
        SPI.writeBytes(&dataBuffer[offset], bytesToSend);
        offset += bytesToSend;
        remaining -= bytesToSend;
    }

    SPI.endTransaction();

    return 1;
}

/**
 * @brief Transmit command and data via SPI
 * @param commandBuf: Command byte
 * @param dataBuffer: Pointer to data buffer
 * @param dataLength: Number of data bytes
 * @return 1 on success
 */
unsigned char spiTransmit(unsigned char commandBuf, unsigned char *dataBuffer, unsigned int dataLength)
{
    if (!spiInitialized) spiInit();

    SPI.beginTransaction(spiSettings);

    // Send command
    SPI.transfer(commandBuf);

    // Send data if provided
    if (dataBuffer != NULL && dataLength > 0) {
        for (unsigned int i = 0; i < dataLength; i++) {
            SPI.transfer(dataBuffer[i]);
        }
    }

    SPI.endTransaction();

    return 1;
}

/**
 * @brief Receive data via SPI after sending command
 * @param commandBuf: Command byte to send
 * @param dataBuffer: Pointer to buffer for received data
 * @param dataLength: Number of bytes to receive
 * @return 1 on success
 */
unsigned char spiReceive(unsigned char commandBuf, unsigned char *dataBuffer, unsigned int dataLength)
{
    if (!spiInitialized) spiInit();
    if (dataBuffer == NULL || dataLength == 0) return 0;

    SPI.beginTransaction(spiSettings);

    // Send command
    SPI.transfer(commandBuf);

    // Receive data
    for (unsigned int i = 0; i < dataLength; i++) {
        dataBuffer[i] = SPI.transfer(0x00);  // Send dummy byte to receive
    }

    SPI.endTransaction();

    return 1;
}

//=================================================================================================
// GPIO Functions (Using Arduino digitalRead/Write)
//=================================================================================================

/**
 * @brief Set GPIO pin level
 * @param pinNumber: GPIO pin number
 * @param voltageLevel: HIGH or LOW
 */
void setGpioLevel(unsigned char pinNumber, unsigned char voltageLevel)
{
    expander1.write1(pinNumber, voltageLevel);
    expander1.pinMode1(pinNumber, OUTPUT);
}

/**
 * @brief Get GPIO pin level
 * @param pinNumber: GPIO pin number
 * @return Current pin level (HIGH or LOW)
 */
unsigned char getGpioLevel(unsigned char pinNumber)
{
    return digitalRead(pinNumber);
}

// temporary Enable IST 9201 pin
void set_ESP32_GpioLevel(uint8_t pinState)
{
    // expander1.write1(pinNumber, voltageLevel);
    // expander1.pinMode1(pinNumber, OUTPUT);
    // digitalWrite(TOOGLE_SEL1, pinState); // Enable IST 9201
    digitalWrite(PMIC_EN, pinState); // Enable IST 9201

}

// temporary
void setGpioLevel_EXPANDER_2(unsigned char pinNumber, unsigned char voltageLevel)
{
    expander2.write1(pinNumber, voltageLevel);
    expander2.pinMode1(pinNumber, OUTPUT);
}