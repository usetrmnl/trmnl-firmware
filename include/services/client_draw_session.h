#pragma once

#include <stdint.h>

/// @brief Keep the client-draw clock live until the content refresh interval runs out.
/// While it runs, a touchbar tap browses the playlist and the new image keeps its own
/// clock. Returns at once when no clock is active.
/// @param startup_time_ms millis() timestamp of this wake, used to measure awake time
void run_client_draw_clock_session(uint32_t startup_time_ms);

/// @brief Release the client-draw clock and shorten the deep sleep by the time it ran.
/// @param time_to_sleep_s deep sleep the refresh interval asks for
/// @return the deep sleep to use, unchanged when no clock ran
uint32_t end_client_draw_clock_session(uint32_t time_to_sleep_s);
