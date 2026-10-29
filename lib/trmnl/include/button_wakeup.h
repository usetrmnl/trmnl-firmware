#pragma once

#include <stdint.h>

// Maps an ext1 wake mask to the value reported in the Update-Source header:
// mainPin reports "button", buttonAPin "button_a", buttonBPin "button_b".
// mainPin takes precedence when the mask names more than one. Returns nullptr
// when it names none of the three. The pins are arguments rather than board
// macros so the mapping can be tested natively.
inline const char *updateSourceForWakeBits(uint64_t wakeBits, int mainPin, int buttonAPin, int buttonBPin) {
  if (wakeBits & (1ULL << mainPin)) return "button";
  if (wakeBits & (1ULL << buttonAPin)) return "button_a";
  if (wakeBits & (1ULL << buttonBPin)) return "button_b";
  return nullptr;
}
