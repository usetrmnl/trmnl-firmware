#pragma once

#include <persistence_interface.h>
#include <stdint.h>

// Single owner of the device's stored refresh interval ("refresh_rate" in NVS):
// the number of seconds goToSleep() arms the deep-sleep timer with. Covers the
// server-provided rate, the fast-poll backoff used while waiting for setup or
// content, the API/WiFi retry ladders, and the fixed fallback.
class RefreshInterval {
public:
  static constexpr const char *SLEEP_KEY = "refresh_rate";
  static constexpr const char *STREAK_KEY = "fast_polls";

  static constexpr uint32_t DEFAULT_SECONDS = 900;

  explicit RefreshInterval(Persistence &persistence);

  // Stored interval in seconds; returns DEFAULT_SECONDS (or defaultValue) when unset.
  uint32_t seconds();
  uint32_t seconds(uint32_t defaultValue);

  // Each apply* method stores its interval and returns the seconds stored.

  // Server-provided rate from /api/display.
  uint32_t applyServerRate(uint32_t rate);

  // Fast refresh with 5-second sleeps for quick updates during initial device
  // setup, backing off in case the server never returns a good response.
  uint32_t applyFastPoll();
  void resetFastPollStreak();

  // Retry ladders; attempt is 1-based.
  uint32_t applyApiRetry(uint8_t attempt);  // 15 / 30 / 60, then DEFAULT_SECONDS
  uint32_t applyWifiRetry(uint8_t attempt); // 60 / 180, then 300
  uint32_t applyDefault();                  // fixed fallback, e.g. /api/setup 404

private:
  static uint32_t fastPollSeconds(uint32_t streak);
  bool writeIfChanged(uint32_t value);

  Persistence &persistence;
};
