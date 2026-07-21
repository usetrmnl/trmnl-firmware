// E2803 EPD driver for TRMNL Gen2 31.5" ACeP 6-color display.
// Adapted from E Ink Holdings sample code (epd.cpp).
// The display uses 8 E2803 driver ICs, each driving 800×720 pixels.

#include "gen2_epd.h"
#include "gen2_pins.h"
#include "gen2_comm.h"
#include "gen2_pmic.h"
#include "gen2_status.h"
#include <Arduino.h>
#include <string.h>

// CS pin mapping: expander1 pins 0-7 in the correct IC order
static const unsigned char spiCsPin[8] = {
    EXP1_SPI1_CS0, EXP1_SPI1_CS1, EXP1_SPI1_CS2, EXP1_SPI1_CS3,
    EXP1_SPI2_CS0, EXP1_SPI2_CS1, EXP1_SPI2_CS2, EXP1_SPI2_CS3
};

// EPD initialization register values (from E Ink sample code)
static const unsigned char PSR_V[2]          = { 0xDB, 0x89 };
static const unsigned char PWR_V[6]          = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
static const unsigned char POF_V[1]          = { 0x00 };
static const unsigned char DRF_V[1]          = { 0x01 };
static const unsigned char CDI_V[1]          = { 0xF7 };
static const unsigned char TCON_V[2]         = { 0x03, 0x03 };
static const unsigned char TRES_V[4]         = { 0x03, 0x20, 0x02, 0xD0 };  // 800×720
static const unsigned char CMD66_V[6]        = { 0x49, 0x55, 0x13, 0x5D, 0x05, 0x10 };
static const unsigned char VCOM_WOUT_EN_V[6] = { 0x00, 0x00, 0x80, 0xFF, 0xCA, 0x1B };
static const unsigned char EN_BUF_V[1]       = { 0x00 };
static const unsigned char TM_TCON_V[7]      = { 0x00, 0xF4, 0x81, 0x40, 0x00, 0x00, 0x00 };
static const unsigned char CCSET_V[1]        = { 0x01 };
static const unsigned char PWS_V[1]          = { 0x22 };
static const unsigned char SPIM_V[1]         = { 0x00 };

// Tracks whether PMIC_PG stayed HIGH across busy-wait loops in epdDisplay()
static unsigned char pgWasGood = 1;

// EPD reset via expander2
static void epdResetPin(unsigned int level)
{
    expander2.write1(EXP2_EPD_RST, level);
    delay(10);
    expander2.pinMode1(EXP2_EPD_RST, OUTPUT);
}

void epdSetCsAll(unsigned int level)
{
    for (int i = 0; i < 8; i++) {
        expander1.write1(spiCsPin[i], level);
        expander1.pinMode1(spiCsPin[i], OUTPUT);
    }
}

void epdSetCs(unsigned char cs, unsigned int level)
{
    if (cs < 8) {
        expander1.write1(spiCsPin[cs], level);
        expander1.pinMode1(spiCsPin[cs], OUTPUT);
    }
}

// EPD BUSY pin is hardware-inverted (MOSFET): logical HIGH when panel is free.
void epdCheckBusyHigh(void)
{
    const int kTimeout = 60000;  // ms — ACeP refresh can take up to 30 s
    int elapsed = 0;
    while (!(!digitalRead(GEN2_EPD_BUSY))) {  // double-negation for MOSFET inversion
        if (!getGpioLevel(GEN2_PMIC_PG)) pgWasGood = 0;
        delay(1);
        if (++elapsed >= kTimeout) {
            Serial.println("[EPD] ERROR: BUSY timeout (waiting HIGH)");
            return;
        }
    }
    Serial.printf("[EPD] BUSY cleared in %d ms\n", elapsed);
}

void epdCheckBusyLow(void)
{
    const int kTimeout = 60000;
    int elapsed = 0;
    while (!digitalRead(GEN2_EPD_BUSY)) {  // inversion: logical LOW = panel busy
        delay(1);
        if (++elapsed >= kTimeout) {
            Serial.println("[EPD] ERROR: BUSY timeout (waiting LOW)");
            return;
        }
    }
}

void epdHardwareReset(void)
{
    Serial.println("[EPD] Hardware reset");
    epdResetPin(GPIO_LOW);
    delay(20);
    epdResetPin(GPIO_HIGH);
    delay(20);
}

void epdWrite(unsigned char cmd, const unsigned char *data, unsigned int len)
{
    spiTransmit(cmd, (unsigned char *)data, len);
}

void epdRead(unsigned char cmd, unsigned char *data, unsigned int len)
{
    spiReceive(cmd, data, len);
}

void epdWriteCommand(unsigned char cmd)
{
    spiTransmitCommand(cmd);
}

void epdWriteData(const unsigned char *data, unsigned int len)
{
    spiTransmitData((unsigned char *)data, len);
}

void epdInit(void)
{
    Serial.println("[EPD] Init start");

    epdHardwareReset();
    epdCheckBusyHigh();

    // Broadcast init commands to all 8 ICs simultaneously
    epdSetCsAll(GPIO_LOW);
    epdWrite(EPD_CMD_VCOM_WOUT_EN, VCOM_WOUT_EN_V, sizeof(VCOM_WOUT_EN_V));
    epdSetCsAll(GPIO_HIGH);

    epdSetCsAll(GPIO_LOW);
    epdWrite(EPD_CMD_TM_TCON, TM_TCON_V, sizeof(TM_TCON_V));
    epdSetCsAll(GPIO_HIGH);

    epdSetCsAll(GPIO_LOW);
    epdWrite(EPD_CMD_CMD66, CMD66_V, sizeof(CMD66_V));
    epdSetCsAll(GPIO_HIGH);

    epdSetCsAll(GPIO_LOW);
    epdWrite(EPD_CMD_PSR, PSR_V, sizeof(PSR_V));
    epdSetCsAll(GPIO_HIGH);

    epdSetCsAll(GPIO_LOW);
    epdWrite(EPD_CMD_PWR, PWR_V, sizeof(PWR_V));
    epdSetCsAll(GPIO_HIGH);

    epdSetCsAll(GPIO_LOW);
    epdWrite(EPD_CMD_CDI, CDI_V, sizeof(CDI_V));
    epdSetCsAll(GPIO_HIGH);

    epdSetCsAll(GPIO_LOW);
    epdWrite(EPD_CMD_TCON, TCON_V, sizeof(TCON_V));
    epdSetCsAll(GPIO_HIGH);

    epdSetCsAll(GPIO_LOW);
    epdWrite(EPD_CMD_TRES, TRES_V, sizeof(TRES_V));
    epdSetCsAll(GPIO_HIGH);

    epdSetCsAll(GPIO_LOW);
    epdWrite(EPD_CMD_EN_BUF, EN_BUF_V, sizeof(EN_BUF_V));
    epdSetCsAll(GPIO_HIGH);

    epdSetCsAll(GPIO_LOW);
    epdWrite(EPD_CMD_PWS, PWS_V, sizeof(PWS_V));
    epdSetCsAll(GPIO_HIGH);

    epdSetCsAll(GPIO_LOW);
    epdWrite(EPD_CMD_SPIM, SPIM_V, sizeof(SPIM_V));
    epdSetCsAll(GPIO_HIGH);

    epdSetCsAll(GPIO_LOW);
    epdWrite(EPD_CMD_CCSET, CCSET_V, sizeof(CCSET_V));
    epdSetCsAll(GPIO_HIGH);

    Serial.println("[EPD] Init complete");
}

void epdWriteImage(unsigned char csx, const unsigned char *imageData, unsigned long len)
{
    Serial.printf("[EPD] Writing image to IC %d (%lu bytes)\n", csx, len);
    epdSetCs(csx, GPIO_LOW);
    spiTransmitLargeData(EPD_CMD_DTM, (unsigned char *)imageData, len);
    epdSetCs(csx, GPIO_HIGH);
}

void epdDisplay(void)
{
    Serial.println("[EPD] Display refresh start");
    if (enablePmic() == DONE) {
        pgWasGood = 1;

        epdSetCsAll(GPIO_LOW);
        epdWriteCommand(EPD_CMD_PON);
        epdCheckBusyHigh();
        epdSetCsAll(GPIO_HIGH);

        epdSetCsAll(GPIO_LOW);
        delay(30);
        epdWrite(EPD_CMD_DRF, DRF_V, sizeof(DRF_V));
        epdCheckBusyHigh();
        epdSetCsAll(GPIO_HIGH);

        epdSetCsAll(GPIO_LOW);
        epdWrite(EPD_CMD_POF, POF_V, sizeof(POF_V));
        epdSetCsAll(GPIO_HIGH);

        if (!pgWasGood) Serial.println("[EPD] WARNING: PMIC_PG dropped during refresh");
    } else {
        Serial.println("[EPD] PMIC enable failed — skipping refresh");
    }
    disablePmic();
    Serial.println("[EPD] Display refresh done");
}

unsigned char epdCheckDriverICStatus(void)
{
    unsigned char status = DONE;
    for (unsigned char csx = 0; csx < 8; csx++) {
        unsigned char buf[3] = { 0 };
        epdSetCs(csx, GPIO_LOW);
        delay(10);
        epdRead(0xF2, buf, 3);
        epdSetCs(csx, GPIO_HIGH);
        if ((buf[0] & 0x01) == 0) {
            Serial.printf("[EPD] IC %d not ready (0x%02X)\n", csx, buf[0]);
            status = ERROR;
        }
    }
    return status;
}

unsigned char epdSetPower(void)
{
    unsigned char vcomStatus = DONE;
    unsigned char tscBuf[2], pwrBuf[5];

    epdSetCs(0, GPIO_LOW);
    epdRead(EPD_CMD_TSC, tscBuf, 2);
    epdCheckBusyHigh();
    epdSetCs(0, GPIO_HIGH);

    epdSetCs(0, GPIO_LOW);
    epdWriteCommand(EPD_CMD_PON);
    epdCheckBusyHigh();
    epdSetCs(0, GPIO_HIGH);

    epdSetCs(0, GPIO_LOW);
    epdRead(0x9B, pwrBuf, 4);
    epdSetCs(0, GPIO_HIGH);

    epdSetCs(0, GPIO_LOW);
    epdWriteCommand(EPD_CMD_POF);
    epdCheckBusyHigh();
    epdSetCs(0, GPIO_HIGH);

    epdSetCs(0, GPIO_LOW);
    epdRead(0x8A, &pwrBuf[4], 1);
    epdSetCs(0, GPIO_HIGH);

    if (pwrBuf[4] == 0x00) {
        vcomStatus = ERROR;
    } else {
        for (int i = 0; i < 4; i++) {
            if (pwrBuf[i] > 120) { vcomStatus = ERROR; break; }
        }
        if (vcomStatus == DONE) {
            pwrBuf[4] -= 128;
            // Apply calibrated voltages to PMIC
            unsigned int buf;
            buf = voltageToRegisterData(pwrBuf[0], SET_VPOS_VENG2);
            initPmicData[4] = buf / 256; initPmicData[5] = buf % 256;
            buf = voltageToRegisterData(pwrBuf[1], SET_VPOS_VENG3);
            initPmicData[8] = buf / 256; initPmicData[9] = buf % 256;
            buf = voltageToRegisterData(pwrBuf[2], SET_VPOS_VENG2);
            initPmicData[6] = buf / 256; initPmicData[7] = buf % 256;
            buf = voltageToRegisterData(pwrBuf[3], SET_VPOS_VENG3);
            initPmicData[10] = buf / 256; initPmicData[11] = buf % 256;
            buf = voltageToRegisterData(pwrBuf[4], SET_VCOMDC);
            initPmicData[12] = buf / 256; initPmicData[13] = buf % 256;
        }
    }
    return vcomStatus;
}
