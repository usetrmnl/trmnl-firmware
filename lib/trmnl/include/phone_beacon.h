#pragma once
#include <stddef.h>
#include <stdint.h>
#include <string.h>

// BLE advertises 128-bit service UUIDs least-significant byte first.
inline bool phoneBeaconUuid(const char *sessionId, uint8_t out[16]) {
  if (!sessionId || strlen(sessionId) != 36) return false;
  unsigned digits = 0;
  uint8_t byte = 0;
  for (unsigned i = 0; i < 36; ++i) {
    if (i == 8 || i == 13 || i == 18 || i == 23) {
      if (sessionId[i] != '-') return false;
      continue;
    }
    char c = sessionId[i];
    int n = c >= '0' && c <= '9'   ? c - '0'
            : c >= 'a' && c <= 'f' ? c - 'a' + 10
            : c >= 'A' && c <= 'F' ? c - 'A' + 10
                                   : -1;
    if (n < 0) return false;
    byte = (byte << 4) | n;
    if (++digits % 2 == 0) out[16 - digits / 2] = byte;
  }
  return digits == 32;
}

inline bool matchesPhoneBeacon(const uint8_t *data, size_t size, const uint8_t uuid[16]) {
  for (size_t i = 0; i < size;) {
    size_t length = data[i++];
    if (!length) break;
    if (length > size - i) return false;
    // Complete or incomplete 128-bit service UUID list, including scan response.
    if ((data[i] == 6 || data[i] == 7) && (length - 1) % 16 == 0) {
      for (size_t j = 1; j < length; j += 16)
        if (memcmp(data + i + j, uuid, 16) == 0) return true;
    }
    i += length;
  }
  return false;
}
