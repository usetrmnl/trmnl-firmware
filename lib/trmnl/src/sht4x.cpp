#include <sht4x.h>

uint8_t sht4x_crc8(const uint8_t *data, int len)
{
  uint8_t crc = 0xFF;
  for (int i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) {
      if (crc & 0x80)
        crc = (uint8_t)((crc << 1) ^ 0x31);
      else
        crc = (uint8_t)(crc << 1);
    }
  }
  return crc;
}

bool sht4x_convert(const uint8_t raw[6], float &tempC, float &humidity)
{
  if (sht4x_crc8(&raw[0], 2) != raw[2]) return false;
  if (sht4x_crc8(&raw[3], 2) != raw[5]) return false;

  uint16_t rawT = (uint16_t)((raw[0] << 8) | raw[1]);
  uint16_t rawH = (uint16_t)((raw[3] << 8) | raw[4]);

  tempC = -45.0f + 175.0f * (float)rawT / 65535.0f;

  float rh = -6.0f + 125.0f * (float)rawH / 65535.0f;
  if (rh < 0.0f)   rh = 0.0f;
  if (rh > 100.0f) rh = 100.0f;
  humidity = rh;

  return true;
}
