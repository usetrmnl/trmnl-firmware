#include <Arduino.h>
#include <power.h>

GPIOPower::GPIOPower(uint8_t pgPin, uint8_t statPin) : _pgPin(pgPin), _statPin(statPin) {}

UsbStatus GPIOPower::usbStatus() {
  // BQ25616 PG is open-drain; LOW = VBUS present.
  pinMode(_pgPin, INPUT);
  return (digitalRead(_pgPin) == 0) ? UsbStatus::CONNECTED : UsbStatus::DISCONNECTED;
}

ChargingStatus GPIOPower::chargingStatus() {
  // BQ25616 STAT: LOW = actively charging, HIGH = charge complete/disabled.
  // TODO: detect ChargingStatus::FAULT (a fault blinks STAT, which reads HIGH).
  pinMode(_statPin, INPUT);
  return (digitalRead(_statPin) == 0) ? ChargingStatus::CHARGING : ChargingStatus::NOT_CHARGING;
}
