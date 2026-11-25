#include <Arduino.h>
#include <display.h>
#include <epd_driver.h>
#include <message_screens.h>
#include <panel_power_check.h>
#include <trmnl_log.h>

#ifdef BOARD_TRMNL_X
#include "logo_small.h"

static const int TRIES = 30;
// FastEPD's default, so a unit fails here when an ordinary draw would time out. Healthy units in
// staging never took over 991 ms.
static const int LIMIT_MS = 1000;
// So a slow unit (1 to 20 s) still draws the check's screens
static const int SCREEN_MS = 30000;

bool checkPanelPower(void) {
  bbep.setPowerGoodTimeout(SCREEN_MS);
  display_show_msg(const_cast<uint8_t *>(logo_small), PANEL_POWER_CHECKING);

  uint32_t slowest_ms = 0;
  int timed_out = 0;
  bbep.setPowerGoodTimeout(LIMIT_MS);
  for (int i = 0; i < TRIES; i++) {
    bbep.einkPower(0);
    delay(2000); // let the rails discharge so each try is a cold start
    uint32_t start_ms = millis();
    if (bbep.einkPower(1) != BBEP_SUCCESS) timed_out++;
    slowest_ms = max(slowest_ms, millis() - start_ms);
  }
  bbep.einkPower(0);

  char line[96];
  snprintf(line, sizeof(line), "%s  slowest %" PRIu32 " ms, %d of %d tries timed out", timed_out ? "FAIL" : "PASS",
           slowest_ms, timed_out, TRIES);
  Log_info_submit("Panel power check %s", line);
  bbep.setPowerGoodTimeout(SCREEN_MS);
  display_show_msg(const_cast<uint8_t *>(logo_small), PANEL_POWER_RESULT, line);
  bbep.setPowerGoodTimeout(0); // FastEPD's default
  return timed_out == 0;
}
#endif // BOARD_TRMNL_X
