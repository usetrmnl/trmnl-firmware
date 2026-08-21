#pragma once

#include <Arduino.h>
#include <Preferences.h>
#include <stdint.h>

//
// derive device-specific clock flags
//
#if defined(BOARD_TRMNL_X)
#define INCLUDE_CLOCK_X
#elif defined(BOARD_TRMNL_GEN2)
#define INCLUDE_CLOCK_GEN2
#endif

/// @brief System clock: epoch time reads and NTP synchronization.
class Clock {
public:
  virtual ~Clock() = default;

  /// @brief Current epoch time in seconds, or 0 if the clock has not been synced.
  uint32_t getTime();

  /// @brief Synchronize the clock, skipping the sync when the last one was
  ///        under 24 hours ago.
  /// @return true if the clock is synced
  bool setTimeFromNTP();

protected:
  /// @brief Perform the synchronization and save `last_sync` on success.
  virtual bool sync(Preferences &prefs, const String &ntpServer);

  /// @brief Wait hook between starting SNTP and checking the result.
  virtual void waitForSync() {}
};

/// @brief gen2 (ESP32-C5): NTP never succeeds unless we busy-wait for a valid
///        time after configTime().
class ClockGen2 : public Clock {
protected:
  void waitForSync() override;
};

#ifdef INCLUDE_CLOCK_X
/// @brief TRMNL X: syncs via the 5 GHz modem's SNTP when connected to a 5 GHz
///        network; falls back to regular NTP otherwise.
class ClockX : public Clock {
protected:
  bool sync(Preferences &prefs, const String &ntpServer) override;
};
#endif // INCLUDE_CLOCK_X

/// @brief Clock for the running board, selected at compile time.
///        (Named to avoid colliding with libc clock().)
Clock &systemClock();
