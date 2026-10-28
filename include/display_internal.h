#pragma once

//
// Shared by the display modules (and the TRMNL X code that drives the IO expander
// through the panel driver). Picks the e-paper library for this board and declares
// the one panel instance, defined in display.cpp. Not for general application code:
// use display.h instead.
//

#include <Arduino.h>
#include <display.h>

#ifdef PARALLEL_EPD
#include "FastEPD.h"
extern FASTEPD bbep;
#else
#define BB_EPAPER
#include <bb_epaper.h>
extern BBEPAPER bbep;
#endif // !PARALLEL_EPD

#ifdef BB_EPAPER
bool display_update_epaper(int refreshMode, bool wait, bool writePlane = false, uint8_t plane = PLANE_0);
void display_select_one_bit_panel(bool only_if_changed = false);
void display_prepare_message(void);
#endif // BB_EPAPER
