#include <fast_poll.h>

FastPoll::FastPoll(Persistence &persistence) : persistence(persistence) {}

uint32_t FastPoll::sleepSeconds(uint32_t streak) {
  if (streak <= 50) return 5;   // 5 sec
  if (streak <= 60) return 60;  // 1 min
  if (streak <= 70) return 900; // 15 min
  return 3600;                  // 1 hour
}

uint32_t FastPoll::nextSleep() {
  uint32_t streak = persistence.readUint(STREAK_KEY, 0) + 1;
  persistence.writeUint(STREAK_KEY, streak);
  return sleepSeconds(streak);
}

void FastPoll::reset() {
  if (persistence.readUint(STREAK_KEY, 0) != 0) persistence.writeUint(STREAK_KEY, 0);
}
