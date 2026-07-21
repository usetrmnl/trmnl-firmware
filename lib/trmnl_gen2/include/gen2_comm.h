#pragma once
#include "TCA9555.h"
#include <stdint.h>

// Global TCA9555 expander objects — defined in gen2_comm.cpp
extern TCA9555 expander1;  // I2C 0x20: SPI CS pins + power rail controls
extern TCA9555 expander2;  // I2C 0x21: display supply sequencing + EPD reset

#ifdef __cplusplus
extern "C" {
#endif

// Initialize SPI bus (4 MHz, no hardware CS — CS via expander1)
void gen2_spiInit(void);

// Initialize I2C bus, power expander 3.3V rail, init both expanders,
// set PMIC_EN/PMIC_PG/EPD_BUSY pin directions.
void gen2_i2cInit(void);

// I2C helpers used by pmic.c and battery.cpp
unsigned char i2cTransmitData(unsigned char addr, unsigned char *buf, unsigned int len);
unsigned char i2cReceiveData(unsigned char addr, unsigned char *buf, unsigned int len);

// Delay helper
void delayms(unsigned int ms);

// SPI helpers used by epd.cpp
unsigned char spiTransmitCommand(unsigned char cmd);
unsigned char spiTransmitData(unsigned char *buf, unsigned long len);
unsigned char spiTransmitLargeData(unsigned char cmd, unsigned char *buf, unsigned long len);
unsigned char spiTransmit(unsigned char cmd, unsigned char *buf, unsigned int len);
unsigned char spiReceive(unsigned char cmd, unsigned char *buf, unsigned int len);

// GPIO helpers used by epd.cpp and pmic.c
// setGpioLevel → expander1.write1 + pinMode1
void setGpioLevel(unsigned char pin, unsigned char level);
// setGpioLevel_EXPANDER_2 → expander2.write1 + pinMode1
void setGpioLevel_EXPANDER_2(unsigned char pin, unsigned char level);
// getGpioLevel → digitalRead (native ESP32-C5 GPIO)
unsigned char getGpioLevel(unsigned char pin);
// set_ESP32_GpioLevel → digitalWrite(PMIC_EN, state)
void set_ESP32_GpioLevel(uint8_t state);

#ifdef __cplusplus
}
#endif
