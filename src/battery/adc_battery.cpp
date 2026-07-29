#include <Arduino.h>
#include <ArduinoLog.h>
#include <battery.h>

ADCBattery::ADCBattery(uint8_t pin) : _pin(pin) {}

float ADCBattery::readVoltage() {
  Log.info("%s [%d]: Battery voltage reading...\r\n", __FILE__, __LINE__);

  int32_t adc = 0;
  analogRead(_pin); // This is needed to properly initialize the ADC BEFORE calling analogReadMilliVolts()
  for (uint8_t i = 0; i < 8; i++) {
    adc += analogReadMilliVolts(_pin);
  }
  int32_t sensorValue = (adc / 8) * 2;
  Log.info("%s [%d]: Battery sensorValue = %d\r\n", __FILE__, __LINE__, (int)sensorValue);
  return sensorValue / 1000.0;
}
