//=================================================================================================
//                   E2803 EPD Driver for 31.5" Control Board (ESP32-C5)
//
// File Name : epd.cpp
// Based on : EL351TW1.c from E Ink sample code
// Version : 1.0
//=================================================================================================

#define __EPD_CPP__

#include <Arduino.h>
#include <trmnl_log.h>
#include <SPI.h>
#include <stdio.h>
#include <string.h>
#include "battery.h"
//#include "globals.h"
#include "pindefine.h"
#include "status.h"
#include "TCA9555.h"
#include "pmic.h"
#include "epd.h"
#include "comm.h"

// Define pins for ESP32-C5
// IMPORTANT: ESP32-C5 has specific GPIO restrictions
// Common valid I2C pins for ESP32-C5: GPIO 6/7, GPIO 8/9, GPIO 18/19
// GPIO 2/3 may cause ESP_ERR_INVALID_STATE on some ESP32-C5 boards
#define I2C_SDA 2 // Try GPIO 6 for SDA (change if needed)
#define I2C_SCL 3 // Try GPIO 7 for SCL (change if needed)
// Create two TCA9555 objects with different I2C addresses
// Default address is 0x20 (when A0, A1, A2 are all LOW)
// Second device should have different address (set via hardware pins A0, A1, A2)
TCA9555 expander1(0x20); // First expander at default address
TCA9555 expander2(0x21); // Second expander (adjust address based on hardware)

// Interrupt flags
volatile bool expander1_interrupt = false;
volatile bool expander2_interrupt = false;


static SPISettings spiSettings(10000000, MSBFIRST, SPI_MODE0); // 10MHz 1-bit SPI is fine

// SPI CS pin mapping - using TCA9555 expander pins
// 8 chip selects: EXP1_SPI1_CS0-3 (pins 0-3) and EXP1_SPI2_CS0-3 (pins 4-7)
static const unsigned char spiCsPin[8] = {
    EXP1_SPI1_CS0, EXP1_SPI1_CS1, EXP1_SPI1_CS2, EXP1_SPI1_CS3,
    EXP1_SPI2_CS0, EXP1_SPI2_CS1, EXP1_SPI2_CS2, EXP1_SPI2_CS3};

// EPD register configuration values (from E Ink sample code)
const unsigned char PSR_V[2] = {
    0xDB, 0x89};
const unsigned char PWR_V[6] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
const unsigned char POF_V[1] = {
    0x00};
const unsigned char DRF_V[1] = {
    0x01};
const unsigned char CDI_V[1] = {
    0xF7};
const unsigned char TCON_V[2] = {
    0x03, 0x03};
const unsigned char TRES_V[4] = {
    0x03, 0x20, 0x02, 0xD0 // Resolution: 800x720
};
const unsigned char CMD66_V[6] = {
    0x49, 0x55, 0x13, 0x5D, 0x05, 0x10};
const unsigned char VCOM_WOUT_EN_V[6] = {
    0x00, 0x00, 0x80, 0xFF, 0xCA, 0x1B};
const unsigned char EN_BUF_V[1] = {
    0x00};
const unsigned char TM_TCON_V[7] = {
    0x00, 0xF4, 0x81, 0x40, 0x00, 0x00, 0x00};
const unsigned char CCSET_V[1] = {
    0x01};
const unsigned char PWS_V[1] = {
    0x22};
const unsigned char SPIM_V[1] = {
    0x00};

// Image data buffer - allocated in PSRAM at runtime
unsigned char *epdImageDataBuffer = nullptr;

// Tracks whether PMIC_PG stayed HIGH across the busy-wait loops in epdDisplay()
static unsigned char pgWasGood = 1;

//================== GPIO Control Functions ====================================

void gpio_cs_pins_init() // set all pins HIGH
{
  // Set all CS pins HIGH (inactive)
  expander1.write1(EXP1_SPI1_CS0, HIGH);
  expander1.write1(EXP1_SPI1_CS1, HIGH);
  expander1.write1(EXP1_SPI1_CS2, HIGH);
  expander1.write1(EXP1_SPI1_CS3, HIGH);
  expander1.write1(EXP1_SPI2_CS0, HIGH);
  expander1.write1(EXP1_SPI2_CS1, HIGH);
  expander1.write1(EXP1_SPI2_CS2, HIGH);
  expander1.write1(EXP1_SPI2_CS3, HIGH);
  // Initialize all SPI chip select pins as outputs (active low, so start HIGH)
  expander1.pinMode1(EXP1_SPI1_CS3, OUTPUT);
  expander1.pinMode1(EXP1_SPI1_CS2, OUTPUT);
  expander1.pinMode1(EXP1_SPI1_CS1, OUTPUT);
  expander1.pinMode1(EXP1_SPI1_CS0, OUTPUT);
  expander1.pinMode1(EXP1_SPI2_CS3, OUTPUT);
  expander1.pinMode1(EXP1_SPI2_CS2, OUTPUT);
  expander1.pinMode1(EXP1_SPI2_CS1, OUTPUT);
  expander1.pinMode1(EXP1_SPI2_CS0, OUTPUT);
}

void epd_gpio_init()
{
  // Initialize EPD control pins on ESP32-C5
  // pinMode(EPD_RST, OUTPUT);

  pinMode(EPD_BUSY, INPUT);

  // Set reset pin HIGH (not in reset)
  // digitalWrite(EPD_RST, HIGH);
  expander2.write1(EXP2_EPD_RST, HIGH);
  delay(50);
  expander2.pinMode1(EXP2_EPD_RST, OUTPUT);
  delay(50);
  Serial.println("EPD GPIO initialized: RST=OUTPUT, BUSY=INPUT");
}

void frameDeepSleep(uint32_t sleepSeconds)
{
//  Serial.printf("[SLEEP] Will sleep for %u s (%u min)\n", sleepSeconds, sleepSeconds / 60);
//  Serial.flush();
  esp_sleep_enable_timer_wakeup((uint64_t)sleepSeconds * 1000000ULL);

  Serial.println("[SLEEP] Configuring IO as deep sleep wakeup source (active LOW)...");
  Serial.flush();
  expander2.pinMode1(EXP2_EN_DC12, INPUT);
  delay(50);
  expander1.pinMode1(EXP1_VDDP_EN, INPUT);
  expander1.pinMode1(EXP1_VDDN_EN, INPUT);
  delay(50);
  setGpioLevel_EXPANDER_2(EXP2_EN_VIN_DC12, LOW);
  delay(50);
  expander1.pinMode1(EXP1_VNCP_EN, INPUT);
  delay(50);
  expander1.write1(EXP1_EN_V5A, LOW);
  delay(50);
  expander2.write1(EXP2_EN_V5D, LOW);
  delay(50);
  expander1.pinMode1(EXP1_EN_V5A, OUTPUT);
  delay(50);
  expander2.pinMode1(EXP2_EN_V5D, OUTPUT);
  delay(50);
  expander2.pinMode1(EXP2_EPD_RST, INPUT);
  delay(50);
  expander1.pinMode1(EXP1_PMIC_PS, INPUT);
  expander2.pinMode1(EXP2_EPD_RST, INPUT);
  expander1.pinMode1(EXP1_EN_V5A, INPUT);
  expander2.pinMode1(EXP2_EN_V5D, INPUT);
  expander2.pinMode1(EXP2_EN_NEGATIVE_3_5V, INPUT);
  expander2.pinMode1(EXP2_EN_DC12, INPUT);
  expander2.pinMode1(EXP2_EN_DC19, INPUT);
  expander1.pinMode1(EXP1_VNCP_EN, INPUT);
  expander1.pinMode1(EXP1_VDDP_EN, INPUT);
  expander1.pinMode1(EXP1_VDDN_EN, INPUT);
  delay(50);
  expander1.pinMode1(EXP1_SPI1_CS3, INPUT);
  expander1.pinMode1(EXP1_SPI1_CS2, INPUT);
  expander1.pinMode1(EXP1_SPI1_CS1, INPUT);
  expander1.pinMode1(EXP1_SPI1_CS0, INPUT);
  expander1.pinMode1(EXP1_SPI2_CS3, INPUT);
  expander1.pinMode1(EXP1_SPI2_CS2, INPUT);
  expander1.pinMode1(EXP1_SPI2_CS1, INPUT);
  expander1.pinMode1(EXP1_SPI2_CS0, INPUT);
  delay(50);
  expander2.pinMode1(EXP2_EN_VIN_SCREEN_3V3, INPUT);
  delay(50);
  pinMode(0, INPUT);
  pinMode(1, INPUT);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(5, INPUT);
  pinMode(6, INPUT);
  pinMode(7, INPUT);
  pinMode(8, INPUT);
  pinMode(9, INPUT);
  pinMode(10, INPUT);
  // pinMode(13, INPUT);
  // pinMode(14, INPUT);
  pinMode(23, INPUT);
  pinMode(24, INPUT);
  pinMode(25, INPUT);
  pinMode(26, INPUT);
  pinMode(27, INPUT);
  pinMode(28, INPUT);
  delay(100);
  esp_err_t err = esp_deep_sleep_enable_gpio_wakeup(1ULL << 0, ESP_GPIO_WAKEUP_GPIO_LOW);
  esp_err_t err1 = esp_deep_sleep_enable_gpio_wakeup(1ULL << 1, ESP_GPIO_WAKEUP_GPIO_LOW);
  esp_err_t err2 = esp_deep_sleep_enable_gpio_wakeup(1ULL << 4, ESP_GPIO_WAKEUP_GPIO_LOW);
  // esp_err_t err3 = esp_deep_sleep_enable_gpio_wakeup(1ULL << 5, ESP_GPIO_WAKEUP_GPIO_LOW);
  if (err != ESP_OK)
  {
    Serial.printf("[SLEEP] ERROR: Failed to configure GPIO wakeup: %d\n", err);
    return;
  }

  Serial.println("[SLEEP] Entering deep sleep.");
  Serial.flush();
  esp_deep_sleep_start();
} /* frameDeepSleep() */

// Interrupt service routines
void IRAM_ATTR expander1_ISR()
{
  expander1_interrupt = true;
}

void IRAM_ATTR expander2_ISR()
{
  expander2_interrupt = true;
}

// I2C Scanner function to find all devices on the bus
void scanI2C()
{
  byte error, address;
  int deviceCount = 0;

  Serial.println("\n========== I2C Bus Scanner ==========");
  Serial.println("Scanning I2C bus (addresses 0x01 to 0x7F)...\n");

  for (address = 1; address < 127; address++)
  {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("Device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.print(" (");
      Serial.print(address);
      Serial.print(")");

      // Identify known devices
      if (address >= 0x20 && address <= 0x27)
      {
        Serial.print(" - Likely TCA9555/TCA9535");
        if (address == 0x20)
          Serial.print(" (Expander 1)");
        if (address == 0x21)
          Serial.print(" (Expander 2)");
      }
      else
      {
        Serial.print(" - Unknown device (possibly IST9201?)");
      }
      Serial.println();
      deviceCount++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }

  Serial.println("\n====================================");
  if (deviceCount == 0)
  {
    Serial.println("No I2C devices found!");
    Serial.println("Check connections and pull-up resistors.");
  }
  else
  {
    Serial.print("Total devices found: ");
    Serial.println(deviceCount);
  }
  Serial.println("====================================\n");
} /* scanI2C() */

void frameInit(void)
{
  if (!Wire.begin(I2C_SDA, I2C_SCL, 100000))
  {
    Serial.println("ERROR: Wire.begin() with frequency FAILED!");
    Serial.println("Trying without frequency parameter...");

    if (!Wire.begin(I2C_SDA, I2C_SCL))
    {
      Serial.println("ERROR: Wire.begin() FAILED!");
      Serial.println("This is ESP_ERR_INVALID_STATE - pins may not support I2C");
      while (1)
      {
        delay(1000);
      }
    }
  }

  pinMode(EXP_EN, OUTPUT);
  delay(10);
  digitalWrite(EXP_EN, HIGH);
  delay(100);
  expander2.write1(12, LOW);      // test
  expander2.pinMode1(12, OUTPUT); // test
  pinMode(PMIC_PG, INPUT);        // PG io 26
  delay(50);
  pinMode(EPD_BUSY, INPUT); // busy io 27
  delay(50);
  pinMode(SPI_1DSX, INPUT); // 1DSX io 23
  delay(50);
  // pinMode(SPI_2DSX, INPUT); // 2DSX io 24
  // delay(50);
  pinMode(6, INPUT);
  delay(50);
  pinMode(7, INPUT);
  delay(50);
  pinMode(8, INPUT);
  delay(50);
  pinMode(9, INPUT);
  delay(50);
  pinMode(10, INPUT);
  delay(50);
  expander2.pinMode1(EXP2_EPD_RST, INPUT);
  expander1.pinMode1(EXP1_EN_V5A, INPUT);
  expander2.pinMode1(EXP2_EN_V5D, INPUT);
  expander2.pinMode1(EXP2_EN_NEGATIVE_3_5V, INPUT);
  expander2.pinMode1(EXP2_EN_DC12, INPUT);
  expander2.pinMode1(EXP2_EN_DC19, INPUT);
  expander1.pinMode1(EXP1_VNCP_EN, INPUT);
  expander1.pinMode1(EXP1_VDDP_EN, INPUT);
  expander1.pinMode1(EXP1_VDDN_EN, INPUT);
  expander1.pinMode1(EXP1_SPI1_CS3, INPUT);
  // delay(50);
  expander1.pinMode1(EXP1_SPI1_CS2, INPUT);
  // delay(50);
  expander1.pinMode1(EXP1_SPI1_CS1, INPUT);
  // delay(50);
  expander1.pinMode1(EXP1_SPI1_CS0, INPUT);
  // delay(50);
  expander1.pinMode1(EXP1_SPI2_CS3, INPUT);
  // delay(50);
  expander1.pinMode1(EXP1_SPI2_CS2, INPUT);
  // delay(50);
  expander1.pinMode1(EXP1_SPI2_CS1, INPUT);
  // delay(50);
  expander1.pinMode1(EXP1_SPI2_CS0, INPUT);
  // pinMode(PMIC_PS, OUTPUT);   // PS expander1 p15
  // digitalWrite(PMIC_PS, LOW); // PS expander1 p15
  expander1.write1(EXP1_PMIC_PS, LOW);
  expander1.pinMode1(EXP1_PMIC_PS, OUTPUT);
  // pinMode(TOOGLE_SEL1, OUTPUT); // not used
  // pinMode(TOOGLE_SEL2, OUTPUT); // not used

  // digitalWrite(TOOGLE_SEL1, LOW);// not used
  // digitalWrite(TOOGLE_SEL2, LOW);// not used
  pinMode(PMIC_EN, OUTPUT); // PS expander1 p15
  digitalWrite(PMIC_EN, LOW);
  // expanders_voltage_gpio_init();
  // pinMode(25, OUTPUT); // ist EN
  // digitalWrite(25, LOW);
  // expander1.write1(EXP1_PMIC_EN, LOW);// changed
  // expander1.pinMode1(EXP1_PMIC_EN, OUTPUT);// changed

  Serial.println("Setting voltage enable pins HIGH...");
  // expander1.write1(EXP1_EN_V5A, HIGH);
  // expander2.write1(EXP2_EN_V5D, HIGH);
  // expander1.pinMode1(EXP1_EN_V5A, OUTPUT);
  // expander2.pinMode1(EXP2_EN_V5D, OUTPUT);
  // expander2.write1(EXP2_EN_1_35V, HIGH);
  // delay(50);
  // expander2.pinMode1(EXP2_EN_1_35V, OUTPUT);
  setGpioLevel_EXPANDER_2(EXP2_EN_VIN_SCREEN_3V3, GPIO_HIGH);
  delay(50);
  setGpioLevel(EXP1_VNCP_EN, GPIO_HIGH);
  // Serial.println("-3.5v ON MOSFET");
  // Serial.println("5v");
  delay(1000);
  Serial.println("EN-SWITCH");
  setGpioLevel_EXPANDER_2(EXP2_EN_VIN_DC12, GPIO_HIGH);
  delay(500);
  // setGpioLevel_EXPANDER_2(EXP2_EN_VIN_SCREEN_3V3, GPIO_HIGH);
  delay(500);
  expander2.write1(EXP2_EN_DC12, HIGH);
  expander2.pinMode1(EXP2_EN_DC12, OUTPUT);
  Serial.println("12v");
  delay(1000);
  expander1.write1(EXP1_EN_V5A, HIGH);
  expander2.write1(EXP2_EN_V5D, HIGH);
  expander1.pinMode1(EXP1_EN_V5A, OUTPUT);
  expander2.pinMode1(EXP2_EN_V5D, OUTPUT);
  delay(500);
  scanI2C();
  // Read PMIC registers 0x1D and 0x1E
  printf("\n========== Reading PMIC Registers ==========\n");

  unsigned char reg_data_0x1D = 0;
  unsigned char reg_data_0x1E = 0;

  // Read 1 byte from register 0x1D
  printf("Reading from PMIC register 0x1D...\n");
  if (readPmicRegister(0x1D, &reg_data_0x1D, 1) == DONE)
  {
    printf("Register 0x1D: 0x%02X (%d)\n", reg_data_0x1D, reg_data_0x1D);
  }

  printf("\n");

  // Read 1 byte from register 0x1E
  printf("Reading from PMIC register 0x1E...\n");
  if (readPmicRegister(0x1E, &reg_data_0x1E, 1) == DONE)
  {
    printf("Register 0x1E: 0x%02X (%d)\n", reg_data_0x1E, reg_data_0x1E);
  }

  printf("===========================================\n\n");
  // delay(500);
  // powerSwitchEnable(); // tetst
  //  // Initialize EPD GPIO pins
  epd_gpio_init();
  // // Initialize SPI CS pins on expander
  gpio_cs_pins_init();
  // // Initialize SPI
  spiInit();
  epdHardwareReset();
  // Initialize EPD driver ICs
  printf("\n========== Initializing EPD ==========\n");

  // Read battery before starting WiFi (shows state at wake-up)
  BatteryStatus bat = batteryRead();
  batteryLog(bat);

} /* frameInit() */
/**
 * @brief Set EPD reset pin level
 */
static void epdResetPin(unsigned int pinStatus)
{
    // digitalWrite(EPD_RST, pinStatus);
    expander2.write1(EXP2_EPD_RST, pinStatus);
    delay(10);
    expander2.pinMode1(EXP2_EPD_RST, OUTPUT);
}

/**
 * @brief Set all chip select pins to specified level
 * @param setLevel: GPIO_HIGH or GPIO_LOW
 */
void epdSetCsAll(unsigned int setLevel)
{
    // for(int i = 0; i < 8; i++)
    // {
    //     expander1.write1(spiCsPin[i], setLevel);
    // }
    expander1.write1(EXP1_SPI1_CS0, setLevel);
    expander1.pinMode1(EXP1_SPI1_CS0, OUTPUT);
    expander1.write1(EXP1_SPI1_CS1, setLevel);
    expander1.pinMode1(EXP1_SPI1_CS1, OUTPUT);
    expander1.write1(EXP1_SPI1_CS2, setLevel);
    expander1.pinMode1(EXP1_SPI1_CS2, OUTPUT);
    expander1.write1(EXP1_SPI1_CS3, setLevel);
    expander1.pinMode1(EXP1_SPI1_CS3, OUTPUT);
    expander1.write1(EXP1_SPI2_CS0, setLevel);
    expander1.pinMode1(EXP1_SPI2_CS0, OUTPUT);
    expander1.write1(EXP1_SPI2_CS1, setLevel);
    expander1.pinMode1(EXP1_SPI2_CS1, OUTPUT);
    expander1.write1(EXP1_SPI2_CS2, setLevel);
    expander1.pinMode1(EXP1_SPI2_CS2, OUTPUT);
    expander1.write1(EXP1_SPI2_CS3, setLevel);
    expander1.pinMode1(EXP1_SPI2_CS3, OUTPUT);
}

/**
 * @brief Set specific chip select pin
 * @param csNumber: CS index (0-7)
 * @param setLevel: GPIO_HIGH or GPIO_LOW
 */
void epdSetCs(unsigned char csNumber, unsigned int setLevel)
{
    if (csNumber < 8)
    {
        expander1.write1(spiCsPin[csNumber], setLevel);
        expander1.pinMode1(spiCsPin[csNumber], OUTPUT);
    }
}

/**
 * @brief Wait for BUSY pin to go HIGH
 */
void epdCheckBusyHigh(void)
{
    //   expander2.write1(12, HIGH);// for debug only
    //   expander2.pinMode1(12, OUTPUT);// for debug only
    int timeout = 60000; // 60 second timeout (31.5" ACeP needs up to 30s refresh)
    int elapsed = 0;
    while (!(!digitalRead(EPD_BUSY))) // added invertion because Hardware FIX mosfet
    {
        if (!digitalRead(PMIC_PG))
        {
            pgWasGood = 0; // no printf here, epdDisplay() reports this once at the end
        }

        delay(1);
        elapsed++;
        if (elapsed >= timeout)
        {
            printf("ERROR: EPD BUSY timeout (waiting for HIGH)\n");
            return;
        }
    }
    Serial.println("Busy_pin ");
    Serial.println(elapsed);
}

/**
 * @brief Wait for BUSY pin to go LOW
 */
void epdCheckBusyLow(void)
{
    int timeout = 60000; // 60 second timeout
    int elapsed = 0;
    while (!digitalRead(EPD_BUSY)) // added invertion because Hardware FIX
    {
        delay(1);
        elapsed++;
        if (elapsed >= timeout)
        {
            printf("ERROR: EPD BUSY timeout (waiting for LOW)\n");
            return;
        }
    }
}

//================== Hardware Reset ====================================

void epdHardwareReset(void)
{
    printf("EPD Hardware Reset...\n");
    epdResetPin(GPIO_LOW);
    delay(20);
    epdResetPin(GPIO_HIGH);
    delay(20);
    printf("EPD Hardware Reset complete\n");
}

//================== SPI Communication ====================================

/**
 * @brief Write command and data to EPD via SPI
 */
void epdWrite(unsigned char epdCommand, const unsigned char *epdData, unsigned int epdDataLength)
{
    spiTransmit(epdCommand, (unsigned char *)epdData, epdDataLength);
}

/**
 * @brief Read data from EPD via SPI
 */
void epdRead(unsigned char epdCommand, unsigned char *epdData, unsigned int epdDataLength)
{
    spiReceive(epdCommand, epdData, epdDataLength);
}

/**
 * @brief Write command only (no data) to EPD
 */
void epdWriteCommand(unsigned char epdCommand)
{
    spiTransmitCommand(epdCommand);
}

/**
 * @brief Write data only (after command) to EPD
 */
void epdWriteData(const unsigned char *epdData, unsigned int epdDataLength)
{
    spiTransmitData((unsigned char *)epdData, epdDataLength);
}

//================== EPD Initialization ====================================

void epdInit(void)
{
    printf("Initializing EPD...\n");

    // Allocate image buffer in PSRAM
    if (epdImageDataBuffer == nullptr)
    {
        epdImageDataBuffer = (unsigned char *)ps_malloc(EPD_IMAGE_DATA_BUFFER);
        if (epdImageDataBuffer == nullptr)
        {
            printf("[EPD] ERROR: Failed to allocate image buffer in PSRAM!\n");
        }
        else
        {
            printf("[EPD] Image buffer allocated in PSRAM (%u bytes)\n", EPD_IMAGE_DATA_BUFFER);
        }
    }

    // Hardware reset
    epdHardwareReset();
    epdCheckBusyHigh();

    // Initialize all driver ICs with same configuration
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

    printf("EPD initialization complete\n");
}

//================== Driver IC Status Check ====================================

unsigned char epdCheckDriverICStatus(void)
{
    unsigned char csx, status = DONE;
    unsigned char dataBuf[3];

    printf("Checking driver IC status...\n");

    for (csx = 0; csx < 8; csx++)
    {
        memset(dataBuf, 0, sizeof(dataBuf));
        epdSetCs(csx, GPIO_LOW);
        delay(10);
        epdRead(0xF2, dataBuf, sizeof(dataBuf));
        epdSetCs(csx, GPIO_HIGH);

#if SHOW_LOG
        printf("Driver IC [%d] = 0x%02X 0x%02X 0x%02X\n", csx, dataBuf[0], dataBuf[1], dataBuf[2]);
#endif

        if ((dataBuf[0] & 0x01) == 0x01)
        {
#if SHOW_LOG
            printf("Driver IC [%d] is ready.\n", csx);
#endif
        }
        else
        {
#if SHOW_LOG
            printf("Driver IC [%d] did not reply.\n", csx);
#endif
            status = ERROR;
        }
    }

    return status;
}

//================== Power Management ====================================

static void epdSetPwrToPmic(unsigned char *pmicData)
{
    unsigned int buf;

    buf = voltageToRegisterData(pmicData[0], SET_VPOS_VENG2);
    initPmicData[4] = buf / 256;
    initPmicData[5] = buf % 256;

    buf = voltageToRegisterData(pmicData[1], SET_VPOS_VENG3);
    initPmicData[8] = buf / 256;
    initPmicData[9] = buf % 256;

    buf = voltageToRegisterData(pmicData[2], SET_VPOS_VENG2);
    initPmicData[6] = buf / 256;
    initPmicData[7] = buf % 256;

    buf = voltageToRegisterData(pmicData[3], SET_VPOS_VENG3);
    initPmicData[10] = buf / 256;
    initPmicData[11] = buf % 256;

    buf = voltageToRegisterData(pmicData[4], SET_VCOMDC);
    initPmicData[12] = buf / 256;
    initPmicData[13] = buf % 256;
}

//================== Display Functions ====================================

void epdDisplay(void)
{
#if SHOW_LOG
    //   expander2.write1(12, HIGH);// for debug only
    //   expander2.pinMode1(12, OUTPUT);// for debug only
    printf("Turn on PMIC.\n");
#endif
    if (enablePmic() == DONE)
    {
        pgWasGood = 1; // reset before tracking PMIC_PG across the busy-wait loops below

#if SHOW_LOG
        printf("Write PON\n");
#endif
        epdSetCsAll(GPIO_LOW);
        epdWriteCommand(EPD_CMD_PON);
        epdCheckBusyHigh();
        epdSetCsAll(GPIO_HIGH);

#if SHOW_LOG
        printf("Write DRF\n");
#endif
        epdSetCsAll(GPIO_LOW);
        delay(30);
        epdWrite(EPD_CMD_DRF, DRF_V, sizeof(DRF_V));
        epdCheckBusyHigh();
        epdSetCsAll(GPIO_HIGH);

#if SHOW_LOG
        printf("Write POF\n");
#endif
        epdSetCsAll(GPIO_LOW);
        epdWrite(EPD_CMD_POF, POF_V, sizeof(POF_V));
        epdSetCsAll(GPIO_HIGH);
#if SHOW_LOG
        if (!pgWasGood)
        {
            printf("WARNING: PMIC_PG == LOW ==\n");
        }
#endif
    }
    else
    {
#if SHOW_LOG
        printf("PMIC enable failed - skipping display refresh.\n");
#endif
    }

#if SHOW_LOG
    printf("Turn off PMIC.\n");
#endif
    disablePmic();

#if SHOW_LOG
    printf("Display Done!!\n");
#endif
}

void epdDisplayColor(unsigned char colorSelect)
{
    unsigned long i;

    printf("Displaying solid color: 0x%02X\n", colorSelect);

    memset(epdImageDataBuffer, colorSelect, EPD_IMAGE_DATA_BUFFER);

    if (epdSetPower() == DONE)
    {
        epdSetCsAll(GPIO_LOW);
        epdWriteCommand(EPD_CMD_DTM);

        // Send image data in chunks
        for (i = 0; i < 288000 / EPD_IMAGE_DATA_BUFFER; i++)
        {
            epdWriteData(epdImageDataBuffer, EPD_IMAGE_DATA_BUFFER);
        }
        epdWriteData(epdImageDataBuffer, 288000 % EPD_IMAGE_DATA_BUFFER);
        epdSetCsAll(GPIO_HIGH);

        epdDisplay();

#if SHOW_LOG
        printf("Display color complete.\n");
#endif
    }
    else
    {
#if SHOW_LOG
        printf("Display color does not work due to epdSetPower() NG.\n");
#endif
    }
}

void epdDisplayColorBar(void)
{
    unsigned long i;

    printf("Displaying color bar pattern...\n");

    if (epdSetPower() == DONE)
    {
        epdSetCsAll(GPIO_LOW);
        epdWriteCommand(EPD_CMD_DTM);

        // BLACK (48000 bytes = 1/6 of screen)
        memset(epdImageDataBuffer, EPD_BLACK, EPD_IMAGE_DATA_BUFFER);
        for (i = 0; i < 48000 / EPD_IMAGE_DATA_BUFFER; i++)
        {
            epdWriteData(epdImageDataBuffer, EPD_IMAGE_DATA_BUFFER);
        }
        epdWriteData(epdImageDataBuffer, 48000 % EPD_IMAGE_DATA_BUFFER);

        // WHITE
        memset(epdImageDataBuffer, EPD_WHITE, EPD_IMAGE_DATA_BUFFER);
        for (i = 0; i < 48000 / EPD_IMAGE_DATA_BUFFER; i++)
        {
            epdWriteData(epdImageDataBuffer, EPD_IMAGE_DATA_BUFFER);
        }
        epdWriteData(epdImageDataBuffer, 48000 % EPD_IMAGE_DATA_BUFFER);

        // YELLOW
        memset(epdImageDataBuffer, EPD_YELLOW, EPD_IMAGE_DATA_BUFFER);
        for (i = 0; i < 48000 / EPD_IMAGE_DATA_BUFFER; i++)
        {
            epdWriteData(epdImageDataBuffer, EPD_IMAGE_DATA_BUFFER);
        }
        epdWriteData(epdImageDataBuffer, 48000 % EPD_IMAGE_DATA_BUFFER);

        // RED
        memset(epdImageDataBuffer, EPD_RED, EPD_IMAGE_DATA_BUFFER);
        for (i = 0; i < 48000 / EPD_IMAGE_DATA_BUFFER; i++)
        {
            epdWriteData(epdImageDataBuffer, EPD_IMAGE_DATA_BUFFER);
        }
        epdWriteData(epdImageDataBuffer, 48000 % EPD_IMAGE_DATA_BUFFER);

        // BLUE
        memset(epdImageDataBuffer, EPD_BLUE, EPD_IMAGE_DATA_BUFFER);
        for (i = 0; i < 48000 / EPD_IMAGE_DATA_BUFFER; i++)
        {
            epdWriteData(epdImageDataBuffer, EPD_IMAGE_DATA_BUFFER);
        }
        epdWriteData(epdImageDataBuffer, 48000 % EPD_IMAGE_DATA_BUFFER);

        // GREEN
        memset(epdImageDataBuffer, EPD_GREEN, EPD_IMAGE_DATA_BUFFER);
        for (i = 0; i < 48000 / EPD_IMAGE_DATA_BUFFER; i++)
        {
            epdWriteData(epdImageDataBuffer, EPD_IMAGE_DATA_BUFFER);
        }
        epdWriteData(epdImageDataBuffer, 48000 % EPD_IMAGE_DATA_BUFFER);

        epdSetCsAll(GPIO_HIGH);

        epdDisplay();

#if SHOW_LOG
        printf("Display color bar complete.\n");
#endif
    }
    else
    {
#if SHOW_LOG
        printf("Display color bar does not work due to epdSetPower() NG.\n");
#endif
    }
}
//
// Write the image data from the local framebuffer into each of the 8 EPD controllers
// The physical layout of the controllers is portrait mode, 1440 wide by 2560 tall. We treat the
// display as if it's landscape 2560 x 1440 and must rotate the lines as we write them.
//
// Each of the 8 controllers manages a vertical column of pixels (in landscape orientation) from the top
// of the display to the bottom. They are all 400x720 bytes, but not all are of each controller's pixels
// are visible. From left to right, here are the visible widths of each controller (in pixels):
// 0: 400, 1: 400, 2: 400, 3: 80, 4: 400, 5: 400, 6: 400, 7: 80
// 400+400+400+80+400+400+400+80 = 2560 pixels wide
// The controllers with only 80 visible columns still need the same number of bytes written to them
// as the other controllers; columns 81-399 are not visible.
//
// Within each vertical bar, the bytes are laid out as follows. Each 400 byte row of bytes defines
// 2 rows of 400 pixels, with the bytes oriented vertically and the most significant nibble on the top
// e.g. here are 4 bytes which represent a 4x2 block of pixels:
// +---+---+---+---+
// |B0H|B1H|B2H|B3H| pixels (0,0),(1,0),(2,0),(3,0) from the high nibbles of bytes 0-3
// +---+---+---+---+
// |B0L|B1L|B2L|B3L| pixels (0,1),(1,1),(2,1),(3,1) from the low nibbles of bytes 0-3
// +---+---+---+---+
//
void epdWriteImage(void *pFrameBuffer)       
{
int x, tx, ty, tw, i;
int csx, iWidth, iPitch;
uint8_t uc, *s, *d;
uint8_t *u8LineBuf;
const int pix_offs[8] = {0, 200, 400, 600, 640, 840, 1040, 1240};

    u8LineBuf = (uint8_t *)malloc(400);
    iPitch = 2560/2; // bytes per row of source image
    Serial.println("[EPD] Writing image data to EPD"); 
    for (csx=0; csx<8; csx++) { // 8 horizontal regions 
        s = (uint8_t *)pFrameBuffer;
        s += pix_offs[csx]; // 200 bytes per 400 pixel region
        epdSetCs(csx, GPIO_LOW);
        SPI.beginTransaction(spiSettings); // DEBUG - convert to QSPI
        SPI.transfer(EPD_CMD_DTM);
        for (ty = 0; ty < 1440; ty+=2) { // write 800 x 720 pixels into this controller's RAM
            uint8_t *ts = s;
            d = u8LineBuf;
            tw = (csx == 3 || csx == 7) ? 80 : 400;
            for (tx=0; tx<tw; tx+=2) {
                *d++ = (ts[0] & 0xf0) | (ts[iPitch] >> 4);
                *d++ = (ts[0] << 4) | (ts[iPitch] & 0xf);
                ts++;
            }
            SPI.writeBytes(u8LineBuf, 400);
            s += (iPitch * 2); // next pair of lines
        } // per row of this section
        SPI.endTransaction();
        epdSetCs(csx, GPIO_HIGH);
    } // per horizontal section
    free(u8LineBuf);
} /* epdWriteImage() */

unsigned char epdSetPower(void)
{
    unsigned char i, vcomStatus = DONE;
    unsigned char readTscBuf[2], readPwrBuf[5];
    // Read TSC
    epdSetCs(0, GPIO_LOW);
    epdRead(EPD_CMD_TSC, &readTscBuf[0], sizeof(readTscBuf));
    epdCheckBusyHigh();
    epdSetCs(0, GPIO_HIGH);
#if SHOW_LOG
    printf("TSC Data = 0x%02X, 0x%02X \r\n", readTscBuf[0], readTscBuf[1]);
#endif

    // PON without the external power
    epdSetCs(0, GPIO_LOW);
    epdWriteCommand(EPD_CMD_PON);
    epdCheckBusyHigh();
    epdSetCs(0, GPIO_HIGH);

    // Read PWR
    epdSetCs(0, GPIO_LOW);
    epdRead(0x9B, &readPwrBuf[0], 4);
    epdSetCs(0, GPIO_HIGH);
#if SHOW_LOG
    printf("PWR Data = 0x%02X, 0x%02X 0x%02X, 0x%02X \r\n", readPwrBuf[0], readPwrBuf[1], readPwrBuf[2], readPwrBuf[3]);
#endif

    // POF
    epdSetCs(0, GPIO_LOW);
    epdWriteCommand(EPD_CMD_POF);
    epdCheckBusyHigh();
    epdSetCs(0, GPIO_HIGH);

    // Read VCOM
    epdSetCs(0, GPIO_LOW);
    epdRead(0x8A, &readPwrBuf[4], 1);
    epdSetCs(0, GPIO_HIGH);

    if (readPwrBuf[4] == 0x00)
    {
#if SHOW_LOG
        printf("VCOM Data = 0x%02X \r\n", readPwrBuf[4]);
        printf("Data NG! \r\n");
#endif
        vcomStatus = ERROR;
    }
    else
    {
        for (i = 0; i < 4; i++)
        {
            if (readPwrBuf[i] > 120)
            {
                vcomStatus = ERROR;
#if SHOW_LOG
                printf("PWM Data [%d] = 0x%02X \r\n", i, readPwrBuf[4]);
                printf("Data NG! \r\n");
#endif
            }
        }

        if (vcomStatus == DONE)
        {
            readPwrBuf[4] = readPwrBuf[4] - 128;
            epdSetPwrToPmic(readPwrBuf);
#if SHOW_LOG
            printf("PWM data and VCOM data are both in range! \r\n");
#endif
        }
    }

    return vcomStatus;
}

//
// Manage the entire update process for displaying an image in the bb_epaper framebuffer
//
void FrameUpdate(uint8_t *pFramebuffer)
{
    Log_info("entering FrameUpdate");
    if (epdCheckDriverICStatus() == DONE) {
        epdInit();
        if (epdSetPower() == DONE) {
            epdWriteImage(pFramebuffer);
            epdDisplay();
            Log_info("Frame update complete.\n");
        }
    } else {
        Log_error("WARNING: Some driver ICs did not respond\n");
    }
    // Read battery again before sleep (shows state after display update)
    batteryLog(batteryRead());
    // Please turn off VDDP, VDDN & VNCP when the EPD does not update the screen within 50 seconds.
    // delayms(50000); // Waiting for PMIC discharge to be 0V.//debug only
    powerSwitchDisable();
} /* FrameUpdate() */