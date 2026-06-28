#pragma once
#include <stdint.h>

// Sensirion CRC-8: polynomial 0x31, init 0xFF, no final XOR.
uint8_t sht4x_crc8(const uint8_t *data, int len);

// Validate and convert a 6-byte SHT4x measurement frame.
// raw[0..1]=temperature, raw[2]=CRC; raw[3..4]=humidity, raw[5]=CRC.
// Returns false (leaving outputs unchanged) if either CRC fails.
// Humidity is clamped to [0, 100].
bool sht4x_convert(const uint8_t raw[6], float &tempC, float &humidity);
