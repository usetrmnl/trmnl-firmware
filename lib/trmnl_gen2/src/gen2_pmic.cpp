// IST9201 PMIC driver for TRMNL Gen2 31.5" display board.
// Adapted from E Ink Holdings sample code (pmic.c).

#include "gen2_pmic.h"
#include "gen2_pins.h"
#include "gen2_comm.h"
#include "gen2_status.h"
#include <Arduino.h>

unsigned char initPmicData[28] = {
    0x02, 0xE3,  // VPOS1
    0x02, 0xE3,  // VNEG1
    0x01, 0x33,  // VPOS2
    0x01, 0x6C,  // VNEG2
    0x01, 0x34,  // VPOS3
    0x00, 0xA2,  // VNEG3
    0x00, 0x9A,  // DC VCOM
    0x11, 0x3F,  // VCOMH
    0x00, 0x9F,  // VCOML
    0xAA,        // Delay Time 1
    0xAA,        // Delay Time 2
    0x80,        // VDDH_EXT delay
    0xD8, 0xBC,  // VGH1
    0x31, 0x94,  // VGH2
    0x00,        // 0x1A VPDD
    0x80,        // 0x1B
    0x00,        // 0x1C
};

unsigned char setPmic(void)
{
    unsigned char status;
    unsigned char buf[3];

    buf[0] = 0x01; buf[1] = initPmicData[0];  buf[2] = initPmicData[1];
    status = sendPmicData(buf, 3);
    buf[0] = 0x05; buf[1] = initPmicData[4];  buf[2] = initPmicData[5];
    status = sendPmicData(buf, 3);
    buf[0] = 0x09; buf[1] = initPmicData[8];  buf[2] = initPmicData[9];
    status = sendPmicData(buf, 3);
    buf[0] = 0x03; buf[1] = initPmicData[2];  buf[2] = initPmicData[3];
    status = sendPmicData(buf, 3);
    buf[0] = 0x07; buf[1] = initPmicData[6];  buf[2] = initPmicData[7];
    status = sendPmicData(buf, 3);
    buf[0] = 0x0B; buf[1] = initPmicData[10]; buf[2] = initPmicData[11];
    status = sendPmicData(buf, 3);
    buf[0] = 0x0D; buf[1] = initPmicData[12]; buf[2] = initPmicData[13];
    status = sendPmicData(buf, 3);
    buf[0] = 0x13; buf[1] = initPmicData[18];
    status = sendPmicData(buf, 2);
    buf[0] = 0x14; buf[1] = initPmicData[19];
    status = sendPmicData(buf, 2);
    buf[0] = 0x15; buf[1] = initPmicData[20];
    status = sendPmicData(buf, 2);
    buf[0] = 0x16; buf[1] = initPmicData[21]; buf[2] = initPmicData[22];
    status = sendPmicData(buf, 3);
    buf[0] = 0x18; buf[1] = initPmicData[23]; buf[2] = initPmicData[24];
    status = sendPmicData(buf, 3);
    buf[0] = 0x1A; buf[1] = initPmicData[25];
    status = sendPmicData(buf, 2);
    buf[0] = 0x1B; buf[1] = initPmicData[26];
    status = sendPmicData(buf, 2);
    buf[0] = 0x1C; buf[1] = initPmicData[27];
    status = sendPmicData(buf, 2);

    return status;
}

unsigned char sendPmicData(unsigned char *buf, unsigned int len)
{
    return i2cTransmitData(GEN2_PMIC_ADDR, buf, len) ? DONE : ERROR;
}

unsigned char readPmicRegister(unsigned char reg, unsigned char *buf, unsigned int len)
{
    if (!i2cTransmitData(GEN2_PMIC_ADDR, &reg, 1)) return ERROR;
    return i2cReceiveData(GEN2_PMIC_ADDR, buf, len) ? DONE : ERROR;
}

unsigned char enablePmic(void)
{
    if (setPmic() != DONE) return ERROR;

    delay(100);
    powerSwitchEnable();
    set_ESP32_GpioLevel(0x01);  // PMIC_EN HIGH

    // Poll PMIC_PG for up to 5 seconds
    Serial.println("[PMIC] Waiting for PMIC_PG...");
    int elapsed = 0;
    while (elapsed < 1000) { delay(100); elapsed += 100; }  // wait 1 s unconditionally
    if (!getGpioLevel(GEN2_PMIC_PG)) {
        elapsed = 0;
        while (!getGpioLevel(GEN2_PMIC_PG) && elapsed < 4000) { delay(100); elapsed += 100; }
        if (!getGpioLevel(GEN2_PMIC_PG))
            Serial.println("[PMIC] WARNING: PMIC_PG timeout — proceeding anyway");
    }
    Serial.printf("[PMIC] PMIC_PG = %d\n", getGpioLevel(GEN2_PMIC_PG));
    return getGpioLevel(GEN2_PMIC_PG) ? DONE : ERROR;
}

void disablePmic(void)
{
    set_ESP32_GpioLevel(0x00);  // PMIC_EN LOW
    while (getGpioLevel(GEN2_PMIC_PG))
        ;  // wait for PMIC_PG to go LOW
}

void powerSwitchEnable(void)
{
    setGpioLevel_EXPANDER_2(EXP2_EN_DC19, GPIO_HIGH);
    delay(100);
    setGpioLevel(EXP1_VDDN_EN, GPIO_HIGH);  // -19 V
    delay(50);
    setGpioLevel(EXP1_VDDP_EN, GPIO_HIGH);  // +19 V
    delay(50);
    setGpioLevel_EXPANDER_2(EXP2_EN_NEGATIVE_3_5V, GPIO_HIGH);  // -3.5 V
    delay(50);
}

void powerSwitchDisable(void)
{
    delay(5000);  // wait for discharge
    setGpioLevel_EXPANDER_2(EXP2_EN_NEGATIVE_3_5V, GPIO_LOW);
    delay(5000);
    setGpioLevel_EXPANDER_2(EXP2_EN_DC19, GPIO_LOW);
}

unsigned int voltageToRegisterData(unsigned int voltage, unsigned char select)
{
    switch (select) {
        case SET_VPOS_VENG2: return 1023 - (170 - voltage) * 1000 / 176;
        case SET_VPOS_VENG3: return 1023 - (270 - voltage) * 1000 / 274;
        case SET_VCOMDC:     return 255  - (50  - voltage) * 250  / 49;
        default:             return 0;
    }
}
