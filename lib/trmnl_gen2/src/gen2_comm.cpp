#include "gen2_comm.h"
#include "gen2_pins.h"
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

// Global TCA9555 expander objects
TCA9555 expander1(0x20);
TCA9555 expander2(0x21);

static SPISettings spiSettings(4000000, MSBFIRST, SPI_MODE0);
static bool spiInitialized = false;

void gen2_spiInit(void)
{
    if (!spiInitialized) {
        SPI.begin(GEN2_SPI_CLK, GEN2_SPI_MISO, GEN2_SPI_MOSI, -1);
        spiInitialized = true;
        Serial.printf("[GEN2] SPI init: CLK=%d MOSI=%d MISO=%d\n",
                      GEN2_SPI_CLK, GEN2_SPI_MOSI, GEN2_SPI_MISO);
    }
}

void gen2_i2cInit(void)
{
    // Power 3.3 V rail for expanders before starting I2C
    pinMode(GEN2_EXP_EN, OUTPUT);
    digitalWrite(GEN2_EXP_EN, HIGH);
    delay(10);

    Wire.begin(GEN2_I2C_SDA, GEN2_I2C_SCL);

    bool ok1 = expander1.begin();
    bool ok2 = expander2.begin();
    Serial.printf("[GEN2] expander1@0x20 %s  expander2@0x21 %s\n",
                  ok1 ? "OK" : "FAIL", ok2 ? "OK" : "FAIL");

    // PMIC control
    pinMode(GEN2_PMIC_EN, OUTPUT);
    digitalWrite(GEN2_PMIC_EN, LOW);

    // Status inputs
    pinMode(GEN2_PMIC_PG, INPUT);
    pinMode(GEN2_EPD_BUSY, INPUT);
}

// I2C -----------------------------------------------------------------------

unsigned char i2cTransmitData(unsigned char addr, unsigned char *buf, unsigned int len)
{
    if (!buf || len == 0) return 0;
    Wire.beginTransmission(addr);
    for (unsigned int i = 0; i < len; i++) Wire.write(buf[i]);
    return (Wire.endTransmission() == 0) ? 1 : 0;
}

unsigned char i2cReceiveData(unsigned char addr, unsigned char *buf, unsigned int len)
{
    if (!buf || len == 0) return 0;
    if (Wire.requestFrom(addr, (uint8_t)len) != len) return 0;
    for (unsigned int i = 0; i < len; i++) {
        if (!Wire.available()) return 0;
        buf[i] = Wire.read();
    }
    return 1;
}

// Delay helper --------------------------------------------------------------

void delayms(unsigned int ms) { delay(ms); }

// SPI -----------------------------------------------------------------------

unsigned char spiTransmitCommand(unsigned char cmd)
{
    if (!spiInitialized) gen2_spiInit();
    SPI.beginTransaction(spiSettings);
    SPI.transfer(cmd);
    SPI.endTransaction();
    return 1;
}

unsigned char spiTransmitData(unsigned char *buf, unsigned long len)
{
    if (!spiInitialized) gen2_spiInit();
    if (!buf || len == 0) return 0;
    SPI.beginTransaction(spiSettings);
    for (unsigned long i = 0; i < len; i++) SPI.transfer(buf[i]);
    SPI.endTransaction();
    return 1;
}

unsigned char spiTransmitLargeData(unsigned char cmd, unsigned char *buf, unsigned long len)
{
    if (!spiInitialized) gen2_spiInit();
    if (!buf || len == 0) return 0;
    SPI.beginTransaction(spiSettings);
    SPI.transfer(cmd);
    // Use writeBytes() — transfer() is bidirectional and would corrupt the image buffer
    const unsigned long kChunk = 4096;
    unsigned long remaining = len, offset = 0;
    while (remaining > 0) {
        unsigned long n = (remaining > kChunk) ? kChunk : remaining;
        SPI.writeBytes(buf + offset, n);
        offset += n;
        remaining -= n;
    }
    SPI.endTransaction();
    return 1;
}

unsigned char spiTransmit(unsigned char cmd, unsigned char *buf, unsigned int len)
{
    if (!spiInitialized) gen2_spiInit();
    SPI.beginTransaction(spiSettings);
    SPI.transfer(cmd);
    if (buf && len > 0)
        for (unsigned int i = 0; i < len; i++) SPI.transfer(buf[i]);
    SPI.endTransaction();
    return 1;
}

unsigned char spiReceive(unsigned char cmd, unsigned char *buf, unsigned int len)
{
    if (!spiInitialized) gen2_spiInit();
    if (!buf || len == 0) return 0;
    SPI.beginTransaction(spiSettings);
    SPI.transfer(cmd);
    for (unsigned int i = 0; i < len; i++) buf[i] = SPI.transfer(0x00);
    SPI.endTransaction();
    return 1;
}

// GPIO helpers --------------------------------------------------------------

void setGpioLevel(unsigned char pin, unsigned char level)
{
    expander1.write1(pin, level);
    expander1.pinMode1(pin, OUTPUT);
}

void setGpioLevel_EXPANDER_2(unsigned char pin, unsigned char level)
{
    expander2.write1(pin, level);
    expander2.pinMode1(pin, OUTPUT);
}

unsigned char getGpioLevel(unsigned char pin)
{
    return (unsigned char)digitalRead(pin);
}

void set_ESP32_GpioLevel(uint8_t state)
{
    digitalWrite(GEN2_PMIC_EN, state);
}
