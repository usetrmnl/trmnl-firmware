#include <services/client_draw_session.h>

#ifdef BOARD_TRMNL_X

#include <Arduino.h>
#include <WiFi.h>
#include <bl.h>
#include <client_draw/clock.h>
#include <display.h>
#include <globals.h>
#include <trmnl_log.h>

#define CLIENT_DRAW_CLOCK_SLEEP_RESERVE_S 30 // tail of the interval left for goToSleep()
#define CLIENT_DRAW_CLOCK_MIN_SLEEP_S     10 // shortest worthwhile deep sleep

void run_client_draw_clock_session(uint32_t startup_time_ms) {
  // A tap ends run_loop(); the image it selects has its own clock, so run the loop again.
  while (client_draw_clock_active()) {
    const uint32_t interval_s = refreshInterval.seconds();
    const uint32_t awake_s = (millis() - startup_time_ms) / 1000;
    if (interval_s <= awake_s + CLIENT_DRAW_CLOCK_SLEEP_RESERVE_S) {
      break;
    }

    // No WiFi during clock maintenance. Do not use WiFi.mode(WIFI_OFF) here;
    // goToSleep() skips it on X-class because it crashes the WiFi stack.
    WiFi.disconnect(true);
    client_draw_clock_run_loop(interval_s - awake_s - CLIENT_DRAW_CLOCK_SLEEP_RESERVE_S);

    // The clock loop exits on a touchbar tap. Run the touchbar action so the
    // bar stays functional while the clock is live.
    switch (client_draw_clock_touched_zone()) {
    case CLIENT_DRAW_CLOCK_TOUCH_LEFT:
      display_draw_touchbar_indicator(TOUCHBAR_LEFT, false);
      Log_info("Back button tapped during client-draw clock");
      show_cached_image_by_offset(-1, false); // sleep_after=false: stay awake for the next pass
      break;
    case CLIENT_DRAW_CLOCK_TOUCH_RIGHT:
      display_draw_touchbar_indicator(TOUCHBAR_RIGHT, false);
      Log_info("Next button tapped during client-draw clock");
      show_cached_image_by_offset(+1, false);
      break;
    case CLIENT_DRAW_CLOCK_TOUCH_MIDDLE:
      display_draw_touchbar_indicator(TOUCHBAR_MIDDLE, false);
      Log_info("Middle button tapped during client-draw clock");
      break; // no new image, so the loop ends and the device sleeps
    default:
      break; // no touch / budget exhausted
    }
  }
}

uint32_t end_client_draw_clock_session(uint32_t time_to_sleep_s) {
  const uint32_t clock_s = client_draw_clock_consumed_s();
  if (clock_s == 0 && !client_draw_clock_active()) {
    return time_to_sleep_s;
  }

  const uint32_t awake_s = iPrevWakeTime / 1000;
  time_to_sleep_s = (time_to_sleep_s > awake_s + CLIENT_DRAW_CLOCK_MIN_SLEEP_S) ? time_to_sleep_s - awake_s
                                                                                : CLIENT_DRAW_CLOCK_MIN_SLEEP_S;
  Log_info("client_draw_clock ran %u s; deep sleep reduced to %u s", (unsigned)clock_s, (unsigned)time_to_sleep_s);
  client_draw_clock_release();
  return time_to_sleep_s;
}

#else // !BOARD_TRMNL_X

void run_client_draw_clock_session(uint32_t startup_time_ms) { (void)startup_time_ms; }

uint32_t end_client_draw_clock_session(uint32_t time_to_sleep_s) { return time_to_sleep_s; }

#endif // BOARD_TRMNL_X
