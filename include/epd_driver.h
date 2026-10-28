#pragma once

//
// Picks the e-paper library for this board and declares the one panel instance,
// defined in display.cpp. The TRMNL X also reaches its IO expander through it.
//

#include <Arduino.h>

#ifdef PARALLEL_EPD
#include "FastEPD.h"
extern FASTEPD bbep;
#else
#define BB_EPAPER
#include <bb_epaper.h>
extern BBEPAPER bbep;
#endif // !PARALLEL_EPD
