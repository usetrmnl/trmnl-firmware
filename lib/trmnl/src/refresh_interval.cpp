#include <refresh_interval.h>
#include <trmnl_log.h>

RefreshInterval::RefreshInterval(Persistence &persistence) : persistence(persistence) {}

uint32_t RefreshInterval::seconds() { return seconds(DEFAULT_SECONDS); }

uint32_t RefreshInterval::seconds(uint32_t defaultValue) { return persistence.readUint(SLEEP_KEY, defaultValue); }

uint32_t RefreshInterval::applyServerRate(uint32_t rate) {
  writeIfChanged(rate);
  return rate;
}

uint32_t RefreshInterval::applyFastPoll() {
  uint32_t streak = persistence.readUint(STREAK_KEY, 0) + 1;
  persistence.writeUint(STREAK_KEY, streak);
  uint32_t sleep = fastPollSeconds(streak);
  writeIfChanged(sleep);
  return sleep;
}

void RefreshInterval::resetFastPollStreak() {
  if (persistence.readUint(STREAK_KEY, 0) != 0) persistence.writeUint(STREAK_KEY, 0);
}

uint32_t RefreshInterval::applyApiRetry(uint8_t attempt) {
  uint32_t sleep;
  switch (attempt) {
  case 1:
    sleep = 15;
    break;
  case 2:
    sleep = 30;
    break;
  case 3:
    sleep = 60;
    break;
  default:
    sleep = DEFAULT_SECONDS;
    break;
  }
  writeIfChanged(sleep);
  return sleep;
}

uint32_t RefreshInterval::applyWifiRetry(uint8_t attempt) {
  if (attempt > WIFI_SHORT_ATTEMPTS + WIFI_OFFLINE_HOURS) return 0;

  uint32_t sleep;
  switch (attempt) {
  case 1:
    sleep = 60;
    break;
  case 2:
    sleep = 180;
    break;
  case 3:
    sleep = 300;
    break;
  default:
    sleep = WIFI_OFFLINE_SECONDS;
    break;
  }
  writeIfChanged(sleep);
  return sleep;
}

uint32_t RefreshInterval::applyDefault() {
  writeIfChanged(DEFAULT_SECONDS);
  return DEFAULT_SECONDS;
}

uint32_t RefreshInterval::fastPollSeconds(uint32_t streak) {
  if (streak <= 50) return 5;   // 5 sec
  if (streak <= 60) return 60;  // 1 min
  if (streak <= 70) return 900; // 15 min
  return 3600;                  // 1 hour
}

bool RefreshInterval::writeIfChanged(uint32_t value) {
  if (persistence.recordExists(SLEEP_KEY) && persistence.readUint(SLEEP_KEY, 0) == value) return false;
  persistence.writeUint(SLEEP_KEY, value);
#ifndef PIO_UNIT_TESTING
  Log_info_serial("write new refresh rate: %u", value);
#endif
  return true;
}
