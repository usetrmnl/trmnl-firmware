//=================================================================================================
//                   PMIC for 31.5" Control Board
//
// File Name : pmic.cpp
// Author : Electronic Design Dept. II
// Data : 2023.12.15
// Version : 1.1
// Copyright : E Ink Holdings Inc.
//=================================================================================================

#define __PMIC_CPP__
#include <Arduino.h>
#include <trmnl_log.h>
#include <stdio.h>
#include <string.h>
#include "pmic.h"
#include "pindefine.h"
#include "comm.h"
#include "status.h"

unsigned char initPmicData[28] = {
    //-----setting VPOS1
    0x02, // value1
    0xE3, // value2
    //-----setting VNEG1
    0x02, // value1
    0xE3, // value2
    //-----setting VPOS2
    0x01, // value1
    0x33, // value2
    //-----setting VNEG2
    0x01, // value1
    0x6C, // value2
    //-----setting VPOS3
    0x01, // value1
    0x34, // value2
    //-----setting VNEG3
    0x00, // value1
    0xA2, // value2
    //-----setting DC VCOM
    0x00, // value1
    0x9A, // value2
    //-----setting VCOMH
    0x11, // value1
    0x3F, // value2
    //-----setting VCOML
    0x00, // value1
    0x9F, // value2
    //-----setting Delay Time 1
    0xAA, // value
    //-----setting Delay Time 2
    0xAA, // value
    //-----setting VDDH_EXT delay time
    0x80, // value
    //-----setting VGH1
    0xD8, // value1
    0xBC, // value2
    //-----setting VGH2
    0x31, // value
    0x94, // value
    //-----setting 0x1A (VPDD)
    0x00, // value
    //-----setting 0x1B
    0x80, // value
    //-----setting 0x1C
    0x00 // value
};

unsigned char setPmic(void)
{
    unsigned char status;
    unsigned char buf[3];

    //-----setting VPOS1
    buf[0] = 0x01;            // register
    buf[1] = initPmicData[0]; // value1
    buf[2] = initPmicData[1]; // value2
#if SHOW_LOG
    printf("1. 0x%02X, 2. 0x%02X, 3. 0x%02X \r\n", buf[0], buf[1], buf[2]);
#endif
    status = sendPmicData(buf, 3);

    //-----setting VPOS2
    buf[0] = 0x05;            // register
    buf[1] = initPmicData[4]; // value1
    buf[2] = initPmicData[5]; // value2
#if SHOW_LOG
    printf("1. 0x%02X, 2. 0x%02X, 3. 0x%02X \r\n", buf[0], buf[1], buf[2]);
#endif
    status = sendPmicData(buf, 3);

    //-----setting VPOS3
    buf[0] = 0x09;            // register
    buf[1] = initPmicData[8]; // value1
    buf[2] = initPmicData[9]; // value2
#if SHOW_LOG
    printf("1. 0x%02X, 2. 0x%02X, 3. 0x%02X \r\n", buf[0], buf[1], buf[2]);
#endif
    status = sendPmicData(buf, 3);

    //-----setting VNEG1
    buf[0] = 0x03;            // register
    buf[1] = initPmicData[2]; // value1
    buf[2] = initPmicData[3]; // value2
#if SHOW_LOG
    printf("1. 0x%02X, 2. 0x%02X, 3. 0x%02X \r\n", buf[0], buf[1], buf[2]);
#endif
    status = sendPmicData(buf, 3);

    //-----setting VNEG2
    buf[0] = 0x07;            // register
    buf[1] = initPmicData[6]; // value1
    buf[2] = initPmicData[7]; // value2
#if SHOW_LOG
    printf("1. 0x%02X, 2. 0x%02X, 3. 0x%02X \r\n", buf[0], buf[1], buf[2]);
#endif
    status = sendPmicData(buf, 3);

    //-----setting VNEG3
    buf[0] = 0x0B;             // register
    buf[1] = initPmicData[10]; // value1
    buf[2] = initPmicData[11]; // value2
#if SHOW_LOG
    printf("1. 0x%02X, 2. 0x%02X, 3. 0x%02X \r\n", buf[0], buf[1], buf[2]);
#endif
    status = sendPmicData(buf, 3);

    //-----setting DC VCOM
    buf[0] = 0x0D;             // register
    buf[1] = initPmicData[12]; // value1
    buf[2] = initPmicData[13]; // value2
#if SHOW_LOG
    printf("1. 0x%02X, 2. 0x%02X, 3. 0x%02X \r\n", buf[0], buf[1], buf[2]);
#endif
    status = sendPmicData(buf, 3);

    // 0x0F, 0x10, 0x11, 0x12   ΤU AVCOMH  VCOML b31.5 Τ   A ۷   AC VCOM

    //-----setting Delay Time 1
    buf[0] = 0x13;             // register
    buf[1] = initPmicData[18]; // value
#if SHOW_LOG
    printf("1. 0x%02X, 2. 0x%02X \r\n", buf[0], buf[1]);
#endif
    status = sendPmicData(buf, 2);

    //-----setting Delay Time 2
    buf[0] = 0x14;             // register
    buf[1] = initPmicData[19]; // value
#if SHOW_LOG
    printf("1. 0x%02X, 2. 0x%02X \r\n", buf[0], buf[1]);
#endif
    status = sendPmicData(buf, 2);

    //-----setting VDDH_EXT delay time
    buf[0] = 0x15;             // register
    buf[1] = initPmicData[20]; // value
#if SHOW_LOG
    printf("1. 0x%02X, 2. 0x%02X \r\n", buf[0], buf[1]);
#endif
    status = sendPmicData(buf, 2);

    //-----setting VGH1
    buf[0] = 0x16;             // register
    buf[1] = initPmicData[21]; // value1
    buf[2] = initPmicData[22]; // value2
#if SHOW_LOG
    printf("1. 0x%02X, 2. 0x%02X, 3. 0x%02X \r\n", buf[0], buf[1], buf[2]);
#endif
    status = sendPmicData(buf, 3);

    //-----setting VGH2
    buf[0] = 0x18;             // register
    buf[1] = initPmicData[23]; // value
    buf[2] = initPmicData[24]; // value
#if SHOW_LOG
    printf("1. 0x%02X, 2. 0x%02X, 3. 0x%02X \r\n", buf[0], buf[1], buf[2]);
#endif
    status = sendPmicData(buf, 3);

    //-----setting 0x1A (VPDD)
    buf[0] = 0x1A;             // register
    buf[1] = initPmicData[25]; // value
#if SHOW_LOG
    printf("1. 0x%02X, 2. 0x%02X \r\n", buf[0], buf[1]);
#endif
    status = sendPmicData(buf, 2);

    //-----setting 0x1B
    buf[0] = 0x1B;             // register
    buf[1] = initPmicData[26]; // value
#if SHOW_LOG
    printf("1. 0x%02X, 2. 0x%02X \r\n", buf[0], buf[1]);
#endif
    status = sendPmicData(buf, 2);

    //-----setting 0x1C
    buf[0] = 0x1C;             // register
    buf[1] = initPmicData[27]; // value
#if SHOW_LOG
    printf("1. 0x%02X, 2. 0x%02X \r\n", buf[0], buf[1]);
#endif
    status = sendPmicData(buf, 2);

    return status;
}

unsigned char sendPmicData(unsigned char *dataBuffer, unsigned int dataLength)
{
    unsigned char status;
    if (i2cTransmitData(PMIC_ADDR, dataBuffer, dataLength))
    {
        status = DONE;
#if SHOW_LOG
        printf("Complete I2C data sending. \r\n");
#endif
    }
    else
    {
        status = ERROR;
#if SHOW_LOG
        printf("Error sending I2C data. \r\n");
#endif
    }
    return status;
}

void recievePmicData(unsigned char *dataBuffer, unsigned int dataLength)
{
    unsigned char status;
    if (i2cReceiveData(PMIC_ADDR, dataBuffer, dataLength))
    {
#if SHOW_LOG
        printf("Complete I2C data receiving. \r\n");
#endif
    }
    else
    {
#if SHOW_LOG
        printf("Error receiving I2C data. \r\n");
#endif
    }
    // Not used in current implementation
}

unsigned char readPmicRegister(unsigned char registerAddress, unsigned char *dataBuffer, unsigned int dataLength)
{
    // First, write the register address we want to read from
    if (!i2cTransmitData(PMIC_ADDR, &registerAddress, 1))
    {
        printf("Error: Failed to set PMIC register address 0x%02X\n", registerAddress);
        return ERROR;
    }

    // Then, read the data from that register
    if (i2cReceiveData(PMIC_ADDR, dataBuffer, dataLength))
    {
        printf("Successfully read %d bytes from PMIC register 0x%02X\n", dataLength, registerAddress);
        return DONE;
    }
    else
    {
        printf("Error: Failed to read from PMIC register 0x%02X\n", registerAddress);
        return ERROR;
    }
}

unsigned char enablePmic(void)
{
    if (setPmic() == DONE)
    {
        Log_info("PMIC registers configured successfully\r\n");
        delay(100);

        powerSwitchEnable(); // we use other pins on OUR green-black devkit
        // delay(1000);       // test
        digitalWrite(PMIC_EN, 1); // Enable IST 9201
        printf("Setting PMIC_EN to HIGH...\r\n");
        // Read PMIC_PG for 1 second no matter its value, then check it
        printf("Waiting for PMIC_PG to go HIGH...\n");
        int elapsed = 0;
        while (elapsed < 1000)
        {
            delay(100);
            elapsed += 100;
            printf("PMIC_PG pin state: %d (%d ms)\n", digitalRead(PMIC_PG), elapsed);
        }
        ////////// debug only
        // unsigned char reg_data_0x1D = 0;
        // unsigned char reg_data_0x1E = 0;
        // // Read 1 byte from register 0x1D
        // printf("Reading from PMIC register 0x1D...\n");
        // if (readPmicRegister(0x1D, &reg_data_0x1D, 1) == DONE)
        // {
        //     printf("Register 0x1D: 0x%02X (%d)\n", reg_data_0x1D, reg_data_0x1D);
        // }
        // printf("\n");
        // // Read 1 byte from register 0x1E
        // printf("Reading from PMIC register 0x1E...\n");
        // if (readPmicRegister(0x1E, &reg_data_0x1E, 1) == DONE)
        // {
        //     printf("Register 0x1E: 0x%02X (%d)\n", reg_data_0x1E, reg_data_0x1E);
        // }
        //////////
        if (!digitalRead(PMIC_PG))
        {
            // Not high yet - allow up to 4 more seconds before giving up
            printf("PMIC_PG not HIGH after 1s, waiting up to 4 more seconds...\n");
            elapsed = 0;
            while (!digitalRead(PMIC_PG) && elapsed < 4000)
            {
                delay(100);
                elapsed += 100;
            }
            if (!digitalRead(PMIC_PG))
            {
                printf("WARNING: PMIC_PG timeout! Proceeding anyway.\n");
                printf("PMIC_PG pin state: %d (should be 1)\n", digitalRead(PMIC_PG));
            }
        }

        if (digitalRead(PMIC_PG))
        {
            printf("PMIC_PG is HIGH - Power good!\n");
        }
#if SHOW_LOG
        printf("enablePmic() done. \r\n");
#endif
        return digitalRead(PMIC_PG) ? DONE : ERROR;
    }
    else
    {
#if SHOW_LOG
        printf("enablePmic() error. \r\n");
#endif
        return ERROR;
    }
}

void disablePmic(void)
{
    digitalWrite(PMIC_EN, 0); // Enable IST 9201
    while (digitalRead(PMIC_PG))
        ; // Waiting for the PMIC_PG is the low level.
}

void powerSwitchEnable(void) // temporary modified in our test PCB setup
{
    setGpioLevel_EXPANDER_2(EXP2_EN_DC19, GPIO_HIGH);
    printf("EN DC19\n");
    delay(100);
    setGpioLevel(EXP1_VDDN_EN, GPIO_HIGH); // -19v
    printf("-19\n");
    delay(50);
    setGpioLevel(EXP1_VDDP_EN, GPIO_HIGH); // +19v
    printf("+19\n");
    delay(50);
    // setGpioLevel_EXPANDER_2(EXP2_EN_DC19, GPIO_HIGH); // -3.5v from black fix temporary
    setGpioLevel_EXPANDER_2(EXP2_EN_NEGATIVE_3_5V, GPIO_HIGH); // -3.5v our DC-DC EN
    printf("-3.5\n");
    delay(50);
    // setGpioLevel(EXP1_VNCP_EN, GPIO_HIGH);
    // printf("-3.5\n");
    // delay(50);
    // expander2.write1(EXP2_EN_NEGATIVE_3_5V, HIGH);
    // expander2.pinMode1(EXP2_EN_NEGATIVE_3_5V, OUTPUT);

    // setGpioLevel(EXP1_VDDN_EN, GPIO_HIGH);
    // printf("-19\n");
    // delay(1000); // Wait for VDDN to be ready.
    // setGpioLevel(EXP1_VDDP_EN, GPIO_HIGH);
    // printf("+19\n");
    // delay(1000); // Wait for VDDP to be ready.
    // setGpioLevel(EXP1_VNCP_EN, GPIO_HIGH);
    // printf("-3.5\n");
#if SHOW_LOG
    printf("Power Switch Enable \r\n");
#endif
}

void powerSwitchDisable(void)
{
    //====== Please use delay() to wait for PMIC's voltages to be 0V ======
    delay(5000); // Please confirm the discharge time of PMIC's power supply.
                   //========================================================
    // setGpioLevel(EXP1_VNCP_EN, GPIO_LOW);
    setGpioLevel_EXPANDER_2(EXP2_EN_NEGATIVE_3_5V, GPIO_LOW);
    printf("-3.5\n");
    //====== Please use delay() to wait for VNCP to be 0V ======
    delay(5000); // Please confirm the discharge time of VNCP power supply.
                   //========================================================
    setGpioLevel_EXPANDER_2(EXP2_EN_DC19, GPIO_LOW);
    printf("DC-DC 19\n");
    // delay(5000);
    // setGpioLevel(EXP1_VDDP_EN, GPIO_LOW);
    // setGpioLevel(EXP1_VDDN_EN, GPIO_LOW);

#if SHOW_LOG
    printf("Power Switch Disable \r\n");
#endif
}

unsigned int voltageToRegisterData(unsigned int voltageData, unsigned char setSelect)
{
    unsigned int registerBuf;
    switch (setSelect)
    {
    case 2:
        registerBuf = 1023 - (170 - voltageData) * 1000 / 176;
        break;
    case 3:
        registerBuf = 1023 - (270 - voltageData) * 1000 / 274;
        break;
    case 4:
        registerBuf = 255 - (50 - voltageData) * 250 / 49;
        break;
    default:
        return registerBuf = 0;
        break;
    }

#if SHOW_LOG
    printf(" Register Data = %3d \r\n", registerBuf);
#endif

    return registerBuf;
}
