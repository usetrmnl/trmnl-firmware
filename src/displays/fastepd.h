#pragma once

// Unified FastEPD entry point for BOARD_X_CLASS targets.
// Callers use fastepd::{init,update} and bbep; board selection stays here.

#ifdef BOARD_TRMNL_X
#include "fastepd_x.h"
namespace fastepd = fastepd_x;
#elif defined(BOARD_X_CLASS)
#include "fastepd_stock.h"
namespace fastepd = fastepd_stock;
#endif
