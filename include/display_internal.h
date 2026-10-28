#pragma once

//
// display.cpp's helpers shared with the other display modules (message_screens.cpp).
// Not for general application code: use display.h instead.
//

#include <display.h>
#include <epd_driver.h>

// Waits u32Millis, in light sleep unless DO_NOT_LIGHT_SLEEP
void display_sleep(uint32_t u32Millis);

#ifdef BB_EPAPER
bool display_update_epaper(int refreshMode, bool wait, bool writePlane = false, uint8_t plane = PLANE_0);
void display_prepare_message(void);
#endif // BB_EPAPER
