#include <Arduino.h>
#include <power.h>

#ifdef INCLUDE_ZECTRIX_POWER

ZectrixPower::ZectrixPower(uint8_t chargingPin, uint8_t chargedPin)
    : _chargingPin(chargingPin), _chargedPin(chargedPin) {}

UsbStatus ZectrixPower::usbStatus() {
  pinMode(_chargingPin, INPUT_PULLUP);
  pinMode(_chargedPin, INPUT);
  return (digitalRead(_chargingPin) == LOW || digitalRead(_chargedPin) == HIGH) ? UsbStatus::CONNECTED
                                                                                : UsbStatus::DISCONNECTED;
}

ChargingStatus ZectrixPower::chargingStatus() {
  pinMode(_chargingPin, INPUT_PULLUP);
  return (digitalRead(_chargingPin) == LOW) ? ChargingStatus::CHARGING : ChargingStatus::NOT_CHARGING;
}

#endif // INCLUDE_ZECTRIX_POWER
