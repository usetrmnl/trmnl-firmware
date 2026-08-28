#include <Arduino.h>
#include <ArduinoLog.h>
#include <Wire.h>
#include <battery.h>
#include <config.h>

static uint16_t readReg16(uint8_t u8Addr, uint8_t u8Reg) {
  uint16_t u16;
  Wire.beginTransmission(u8Addr);
  Wire.write(u8Reg);
  Wire.endTransmission();
  Wire.requestFrom(u8Addr, 2);
  u16 = Wire.read();
  u16 |= (Wire.read() << 8);
  return u16;
} /* readReg16() */

// ADCBattery::ADCBattery() {}
float ADCBattery::readVoltage() {
  // stub function
  return 0.0f;
}

float ADCBattery::readVoltage(TRMNL_DEVICE *pDevice) {
  Log.info("%s [%d]: Battery voltage reading...\r\n", __FILE__, __LINE__);
  if (pDevice->batt_type == BATT_NONE) {
    Log.info("No battery defined; read as a fake voltage of 4.2V\r\n");
    return 4.2f;
  } else if (pDevice->batt_type == BATT_ADC) {
    if (pDevice->batt_en_pin != 0xff) {
      pinMode(pDevice->batt_en_pin, OUTPUT);
      digitalWrite(pDevice->batt_en_pin, HIGH);
      delay(10); // Wait for the switch to stabilize
    }
    int32_t adc = 0;
    analogRead(
      pDevice->batt_pin); // This is needed to properly initialize the ADC BEFORE calling analogReadMilliVolts()
    for (uint8_t i = 0; i < 8; i++) {
      adc += analogReadMilliVolts(pDevice->batt_pin);
    }
    if (pDevice->batt_en_pin != 0xff) {
      digitalWrite(pDevice->batt_en_pin, LOW);
    }
    int32_t sensorValue = (adc / 8) * 2;
    Log.info("%s [%d]: Battery sensorValue = %d\r\n", __FILE__, __LINE__, (int)sensorValue);
    return (float)sensorValue / 1000.0f;
  } else if (pDevice->batt_type == BATT_BQ27220) { // BQ27220
    Wire.begin(pDevice->sensor_sda, pDevice->sensor_scl);
    int16_t sensorValue = readReg16(0x55, 8); // current battery voltage in millivolts (registers 8+9)
    return (float)sensorValue / 1000.0f;
  } else { // BQ2742x
    Wire.begin(pDevice->sensor_sda, pDevice->sensor_scl);
    int16_t sensorValue = readReg16(0x55, 4); // current battery voltage in millivolts (registers 4+5)
    return (float)sensorValue / 1000.0f;
  }
} /* readVoltage() */
