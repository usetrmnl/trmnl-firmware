#include <modem_control.h>

#ifdef BOARD_TRMNL_X

#include <Arduino.h>

#include "FastEPD.h"

extern FASTEPD bbep; // display.cpp

#define PIN_ESP32C5_SPI_BOOT 4
#define PIN_ESP32C5_USB_BOOT 5
#define PIN_ESP32C5_EN       6

void modem_enter_bootloader(void) {
  // Disable target and wait for full power down
  bbep.ioWrite(PIN_ESP32C5_EN, 0);
  delay(100);

  // Configure boot pins for bootloader mode
  bbep.ioPinMode(PIN_ESP32C5_SPI_BOOT, OUTPUT);
  bbep.ioPinMode(PIN_ESP32C5_USB_BOOT, OUTPUT);
  bbep.ioWrite(PIN_ESP32C5_SPI_BOOT, 0);
  bbep.ioWrite(PIN_ESP32C5_USB_BOOT, 1);
  delay(50);

  // Enable target in bootloader mode
  bbep.ioWrite(PIN_ESP32C5_EN, 1);
  delay(300);
}

void modem_reset_target(void) {
  // Disable target
  bbep.ioWrite(PIN_ESP32C5_EN, 0);
  delay(50);

  // Release boot pins (set to input for normal boot)
  bbep.ioPinMode(PIN_ESP32C5_SPI_BOOT, INPUT);
  bbep.ioPinMode(PIN_ESP32C5_USB_BOOT, INPUT);
  delay(50);

  // Enable target (normal boot)
  bbep.ioWrite(PIN_ESP32C5_EN, 1);
}

#endif // BOARD_TRMNL_X
