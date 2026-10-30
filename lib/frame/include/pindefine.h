//=================================================================================================
//                   Pin Define for 31.5" Control Board 
//
// File Name : pindefine.h
// Author : Electronic Design Dept. II
// Data : 2023.12.15
// Version : 1.1
// Copyright : E Ink Holdings Inc.
//=================================================================================================

#ifndef __PINDEFINE_H__
#define __PINDEFINE_H__

//==============  ESP32-C5 SPI Pin Setting   ==============//
// Hardware SPI pins for EPD communication
// Note: CS pins are managed via TCA9555 I/O expanders
#define SPI_CLK		9 // ESP32-C5 SPI Clock
#define SPI_Data0	10    // ESP32-C5 SPI MOSI
#define SPI_Data1	8    // ESP32-C5 SPI MISO
#define SPI_Data2	7   // Reserved for QSPI
#define SPI_Data3	6   // Reserved for QSPI

#define SPI_1DSX	23   // Reserved for 1DSX
#define EXP_EN	    24 // VIN exp 3.3V black brd(fix) (OLD SPI_2DSX)
// Legacy CS pin definitions (not used - CS is via TCA9555 expanders)
#define SPI_CS0		1
#define SPI_CS1		2
#define SPI_CS2		3
#define SPI_CS3		4
#define SPI_CS4		15
#define SPI_CS5		16
#define SPI_CS6		17
#define SPI_CS7		18

//==============   ESP32-C5 GPIO Setting   ==============//
// Direct GPIO pins on ESP32-C5 MCU
#define EPD_BUSY	27//was4    // EPD busy signal input
// #define EPD_RST		5    // EPD reset output
#define PMIC_PG		26   // PMIC power good input
// #define PMIC_PS		25   // PMIC power save mode control

// #define TOOGLE_SEL1 23 //PMIC ENABLE on FIXED BOARD
// #define TOOGLE_SEL2 24
#define PMIC_EN	25

#define BTN3	4
#define BTN4	1
#define BTN5	0

#define GAUGE_INT	5
//===============================================


//==============   Expander 1 GPIO Setting   ==============//
//Please modify the pin number
#define EXP1_SPI1_CS3	0
#define EXP1_SPI1_CS2	1
#define EXP1_SPI1_CS1	2
#define EXP1_SPI1_CS0	3
#define EXP1_SPI2_CS3	4
#define EXP1_SPI2_CS2	5
#define EXP1_SPI2_CS1	6
#define EXP1_SPI2_CS0	7
#define EXP1_CS_SD_CARD	8
#define EXP1_EN_V5A 9
#define EXP1_VDDN_EN 10 //MSFT out
#define EXP1_VDDP_EN 11 //MSFT out
#define EXP1_VNCP_EN 12 //MSFT out
#define EXP1_PMIC_PS 13
#define EXP1_CHARGER_status 14
#define EXP1_CHARGER_pg 15

//===============================================

//==============   Expander 2 GPIO Setting   ==============//
//Please modify the pin number
// #define EXP2_BTN3	
// #define EXP2_BTN4	
// #define EXP2_BTN5	
#define EXP2_EN_1_35V	0//not used on FIXED BOARD (was#6)
#define EXP2_BAT_ID     1
#define EXP2_EN_V5D     3
#define EXP2_EN_DC19	4
#define EXP2_EN_DC12	5
#define EXP2_EN_VIN_DC12	6
#define EXP2_EN_NEGATIVE_3_5V	8 //DC EN(was7)
#define EXP2_EN_VIN_SCREEN_3V3	9
// #define EXP2_GAUGE_INT	13
#define EXP2_EPD_RST 15
//===============================================


#define GPIO_LOW	0
#define GPIO_HIGH	1

#endif //#ifndef __PINDEFINE_H__
