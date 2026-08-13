#include <Arduino.h>
#include <ArduinoLog.h>
#include <battery.h>

//ADCBattery::ADCBattery() {}

float ADCBattery::readVoltage(uint8_t pin, uint8_t enable) {
  Log.info("%s [%d]: Battery voltage reading...\r\n", __FILE__, __LINE__);
  if (pin == 0xff) {
    Log.info("Undefined battery pin; read as a fake voltage of 4.2V\r\n");
    return 4.2f;
  }
  if (enable != 0xff) {
    pinMode(enable, OUTPUT);
    digitalWrite(enable, HIGH);
    delay(10); // Wait for the switch to stabilize
  }
  int32_t adc = 0;
  analogRead(pin); // This is needed to properly initialize the ADC BEFORE calling analogReadMilliVolts()
  for (uint8_t i = 0; i < 8; i++) {
    adc += analogReadMilliVolts(pin);
  }
  if (enable != 0xff) {
    digitalWrite(enable, LOW);
  }
  int32_t sensorValue = (adc / 8) * 2;
  Log.info("%s [%d]: Battery sensorValue = %d\r\n", __FILE__, __LINE__, (int)sensorValue);
  return sensorValue / 1000.0;
} /* readVoltage() */
