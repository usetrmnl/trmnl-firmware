#include <power.h>

#ifdef INCLUDE_TCA9535_POWER

#include <Arduino.h>

#include "FastEPD.h"

TCA9535Power::TCA9535Power(FASTEPD &bbep, uint8_t pgPin, uint8_t statPin)
    : _bbep(bbep), _pgPin(pgPin), _statPin(statPin) {}

UsbStatus TCA9535Power::usbStatus() {
  _bbep.ioPinMode(_pgPin, INPUT);
  return (_bbep.ioRead(_pgPin) == 0) ? UsbStatus::CONNECTED : UsbStatus::DISCONNECTED;
}

ChargingStatus TCA9535Power::chargingStatus() {
  // BQ25616 STAT: LOW = actively charging, HIGH = charge complete/disabled.
  // TODO: detect ChargingStatus::FAULT (a fault blinks STAT, which reads HIGH).
  _bbep.ioPinMode(_statPin, INPUT);
  return (_bbep.ioRead(_statPin) == 0) ? ChargingStatus::CHARGING : ChargingStatus::NOT_CHARGING;
}

#endif // INCLUDE_TCA9535_POWER
