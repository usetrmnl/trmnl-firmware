#pragma once

#include <stdint.h>
#include <persistence_interface.h>

// A flat 5s sleep on "keep polling" responses (202 / 500 / empty_state) has no
// ceiling: a state that never resolves (abandoned setup, playlist with nothing
// servable) keeps a device at ~6,600 wakes/day until the battery dies. The
// ladder keeps setup snappy for the first ~4 minutes, then backs off to a 1h
// ceiling. The streak persists across deep sleep and resets on real content.
constexpr uint32_t FAST_POLL_SLEEP_SECONDS = 5;
constexpr uint32_t FAST_POLL_SNAPPY_POLLS = 50;
#define FAST_POLL_STREAK_KEY "fast_polls"

uint32_t fastPollSleepSeconds(uint32_t streak);
uint32_t fastPollNextSleep(Persistence &persistence);
void fastPollStreakReset(Persistence &persistence);
