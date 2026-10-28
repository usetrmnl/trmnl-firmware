#pragma once
#include <stdint.h>

// RAM-only inactivity lease. Callers serialize access. Each
// authenticated hold renews it; expiry/release seals it until the next boot.
class BleWakeWindow {
public:
  void hold(uint64_t now) {
    if (used && now >= deadline) return;
    used = true;
    deadline = now + 30000;
  }
  void release() {
    used = true;
    deadline = 0;
  }
  uint32_t remaining(uint64_t now) const { return now < deadline ? static_cast<uint32_t>(deadline - now) : 0; }

private:
  bool used = false;
  uint64_t deadline = 0;
};

// Ignore a wake button still held at session entry; accept a fresh, debounced press.
class BleWakeExitButton {
public:
  bool pressed(uint64_t now, bool down) {
    if (!down) {
      armed = true;
      tracking = false;
      return false;
    }
    if (!armed) return false;
    if (!tracking) {
      since = now;
      tracking = true;
    }
    return now - since >= 30;
  }

private:
  bool armed = false;
  bool tracking = false;
  uint64_t since = 0;
};
