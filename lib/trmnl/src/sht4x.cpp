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
