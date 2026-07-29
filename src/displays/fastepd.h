#pragma once

#ifdef BOARD_X_CLASS

#include <FastEPD.h>
#include <stdint.h>

extern FASTEPD bbep;

namespace fastepd {
  void init();
  void update(bool bWait, bool bSkipClear, int iUpdateCount, uint32_t iTempProfile, int data_size);
} // namespace fastepd

#endif // BOARD_X_CLASS
