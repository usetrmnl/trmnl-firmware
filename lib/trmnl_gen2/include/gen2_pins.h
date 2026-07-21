#pragma once

// ESP32-C5 SPI pins for E2803 EPD driver ICs
#define GEN2_SPI_CLK    9    // SPI Clock
#define GEN2_SPI_MOSI   10   // SPI MOSI (SPI_Data0)
#define GEN2_SPI_MISO   8    // SPI MISO (SPI_Data1)

// Direct ESP32-C5 GPIO
#define GEN2_EXP_EN     24   // 3.3V rail enable for TCA9555 expanders
#define GEN2_EPD_BUSY   27   // EPD BUSY signal (hardware MOSFET inverts it: pin HIGH = panel free)
#define GEN2_PMIC_PG    26   // IST9201 PMIC power-good input (HIGH = rails OK)
#define GEN2_PMIC_EN    25   // IST9201 PMIC enable output

#define GEN2_BTN3       4
#define GEN2_BTN4       1
#define GEN2_BTN5       0
#define GEN2_GAUGE_INT  5    // BQ27427 interrupt

// I2C
#define GEN2_I2C_SDA    11
#define GEN2_I2C_SCL    12

// TCA9555 Expander 1 (I2C 0x20) — SPI CS and power rail controls
#define EXP1_SPI1_CS3   0    // EPD IC 3 chip-select
#define EXP1_SPI1_CS2   1    // EPD IC 2 chip-select
#define EXP1_SPI1_CS1   2    // EPD IC 1 chip-select
#define EXP1_SPI1_CS0   3    // EPD IC 0 chip-select
#define EXP1_SPI2_CS3   4    // EPD IC 7 chip-select
#define EXP1_SPI2_CS2   5    // EPD IC 6 chip-select
#define EXP1_SPI2_CS1   6    // EPD IC 5 chip-select
#define EXP1_SPI2_CS0   7    // EPD IC 4 chip-select
#define EXP1_CS_SD_CARD 8
#define EXP1_EN_V5A     9
#define EXP1_VDDN_EN    10   // -19 V rail enable
#define EXP1_VDDP_EN    11   // +19 V rail enable
#define EXP1_VNCP_EN    12   // -3.5 V rail (charge-pump, not used on current PCB)
#define EXP1_PMIC_PS    13
#define EXP1_CHARGER_status 14  // BQ25616 STAT — open-drain, LOW = charging
#define EXP1_CHARGER_pg     15  // BQ25616 PG#  — open-drain, LOW = VBUS present

// TCA9555 Expander 2 (I2C 0x21) — display supply sequencing + EPD reset
#define EXP2_EN_1_35V           0
#define EXP2_BAT_ID             1
#define EXP2_EN_V5D             3
#define EXP2_EN_DC19            4   // Main DC19 converter enable
#define EXP2_EN_DC12            5
#define EXP2_EN_VIN_DC12        6
#define EXP2_EN_NEGATIVE_3_5V   8   // -3.5 V DC-DC enable
#define EXP2_EN_VIN_SCREEN_3V3  9
#define EXP2_EPD_RST            15  // EPD hardware reset (active LOW)

// Logic level aliases matching the original E Ink sample code
#define GPIO_LOW    0
#define GPIO_HIGH   1
