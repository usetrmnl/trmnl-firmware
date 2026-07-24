#pragma once

#if defined(BOARD_X_CLASS) && !defined(BOARD_TRMNL_X)

#include <FastEPD.h>
#include <stdint.h>

extern FASTEPD bbep;

namespace fastepd_stock {
  void init();
  void update(bool bWait, bool bSkipClear, int iUpdateCount, uint32_t iTempProfile);
} // namespace fastepd_stock

#endif // BOARD_X_CLASS && !BOARD_TRMNL_X
