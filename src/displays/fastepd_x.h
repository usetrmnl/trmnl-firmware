#pragma once

#ifdef BOARD_TRMNL_X

#include <FastEPD.h>
#include <stdint.h>

// Exposes the protected FASTEPDSTATE that the fast 4bpp render needs to drive
// the panel through FastEPD's low-level primitives.
class FASTEPDX : public FASTEPD {
public:
  FASTEPDSTATE *state(void) { return &_state; }
};

extern FASTEPDX bbep;

namespace fastepd_x {
  void init();
  void update(bool bWait, bool bSkipClear, int iUpdateCount, uint32_t iTempProfile);
} // namespace fastepd_x

#endif // BOARD_TRMNL_X
