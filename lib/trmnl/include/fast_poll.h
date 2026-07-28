#pragma once

#include <persistence_interface.h>
#include <stdint.h>

// Fast-refresh with 5-second sleep for quick updates during initial device setup, but
// with a backoff in case the server never returns a good response.
class FastPoll {
public:
  static constexpr const char *STREAK_KEY = "fast_polls";

  explicit FastPoll(Persistence &persistence);

  uint32_t nextSleep();
  void reset();

private:
  static uint32_t sleepSeconds(uint32_t streak);

  Persistence &persistence;
};
