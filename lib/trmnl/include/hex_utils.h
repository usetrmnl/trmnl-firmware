#pragma once

#include <Arduino.h>
#include <stdint.h>
#include <stddef.h>

inline String bytesToHex(const uint8_t *bytes, size_t len)
{
  static const char hexChars[] = "0123456789abcdef";
  String hex;
  hex.reserve(len * 2);
  for (size_t i = 0; i < len; i++) {
    hex += hexChars[(bytes[i] >> 4) & 0x0F];
    hex += hexChars[bytes[i] & 0x0F];
  }
  return hex;
}
