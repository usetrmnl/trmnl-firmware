#include <sht4x_read.h>
#include <config.h>

#ifdef HAS_ONBOARD_SHT4X
#include <Arduino.h>
#include <Wire.h>
#include <sht4x.h>

bool sht4x_read(float &tempC, float &humidity)
{
  Wire.begin(SHT4X_SDA_PIN, SHT4X_SCL_PIN);

  Wire.beginTransmission(SHT4X_I2C_ADDR);
  Wire.write(0xFD); // measure T & RH, high precision
  if (Wire.endTransmission() != 0)
    return false;

  delay(10); // datasheet: high-precision conversion ~8.3 ms max

  uint8_t raw[6];
  if (Wire.requestFrom((int)SHT4X_I2C_ADDR, 6) != 6)
    return false;
  for (int i = 0; i < 6; i++)
    raw[i] = (uint8_t)Wire.read();

  return sht4x_convert(raw, tempC, humidity);
}
#endif // HAS_ONBOARD_SHT4X
