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

// Decode an image straight into the panel driver's buffer/RAM; return the refresh mode to use, or an error
int png_to_epd(const uint8_t *pPNG, int iDataSize, bool bPrevious);
int jpeg_to_epd(const uint8_t *pJPEG, int iDataSize);

#ifdef BB_EPAPER
// 1-bit and 2-bit panel types for each panel set (pDevice->panel_set) and temperature profile
extern const DISPLAY_PROFILE dpList[12][3];

bool display_update_epaper(int refreshMode, bool wait, bool writePlane = false, uint8_t plane = PLANE_0);
void display_select_one_bit_panel(bool only_if_changed = false);
void display_prepare_message(void);

// Reads the panel ID (REV command) over bit-banged SPI; 0 when unknown or not readable
uint32_t get_panel_rev(void);
#endif // BB_EPAPER
