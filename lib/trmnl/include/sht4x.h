#pragma once
#include <stdint.h>

// Sensirion CRC-8: polynomial 0x31, init 0xFF, no final XOR.
uint8_t sht4x_crc8(const uint8_t *data, int len);
