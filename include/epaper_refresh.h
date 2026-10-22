#pragma once

#ifndef PARALLEL_EPD
#include <bb_epaper.h>

// Defined in display.cpp; the message screens share them with display_show_image
bool display_update_epaper(int refreshMode, bool wait, bool writePlane = false, uint8_t plane = PLANE_0);
void display_prepare_message(void);
#endif // !PARALLEL_EPD
