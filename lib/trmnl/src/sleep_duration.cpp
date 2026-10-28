#include <sleep_duration.h>

uint32_t sleepDurationSeconds(uint32_t now, uint32_t lastSleep) {
  if (lastSleep == 0 || now < lastSleep) return 0;
  return now - lastSleep;
}
