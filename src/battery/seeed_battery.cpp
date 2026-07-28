#include <Arduino.h>
#include <battery.h>

SeeedBattery::SeeedBattery(uint8_t pin, uint8_t switchPin, uint8_t switchOnLevel)
    : ADCBattery(pin), _switchPin(switchPin), _switchOnLevel(switchOnLevel) {}

float SeeedBattery::readVoltage() {
  pinMode(_switchPin, OUTPUT);
  digitalWrite(_switchPin, _switchOnLevel);
  delay(10); // Wait for the switch to stabilize

  float voltage = ADCBattery::readVoltage();

  digitalWrite(_switchPin, _switchOnLevel == HIGH ? LOW : HIGH);
  return voltage;
}
