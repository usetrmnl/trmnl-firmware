#include <fast_poll_backoff.h>

uint32_t fastPollSleepSeconds(uint32_t streak)
{
  if (streak <= FAST_POLL_SNAPPY_POLLS)
    return FAST_POLL_SLEEP_SECONDS;
  if (streak <= 60)
    return 60;
  if (streak <= 70)
    return 900;
  return 3600;
}

uint32_t fastPollNextSleep(Persistence &persistence)
{
  uint32_t streak = persistence.readUint(FAST_POLL_STREAK_KEY, 0) + 1;
  persistence.writeUint(FAST_POLL_STREAK_KEY, streak);
  return fastPollSleepSeconds(streak);
}

void fastPollStreakReset(Persistence &persistence)
{
  if (persistence.readUint(FAST_POLL_STREAK_KEY, 0) != 0)
    persistence.writeUint(FAST_POLL_STREAK_KEY, 0);
}
