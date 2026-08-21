#include <battery_soc.h>

/// @brief Per-cell pack capacity (mAh) assumed when charge gauging is bypassed.
#define BYPASS_CELL_CAPACITY_MAH 6000

int batteryVoltageToPercent(float voltage) {
  // Mirrors the server's percent_charged_calculation: map 3.0 V onto 0 % at
  // 0.012 V per percent, with plateaus near full charge (4.08 V follows a
  // full charge) and a 1 % floor.
  float pct = (voltage - 3.0f) / 0.012f;
  if (pct >= 88.0f) return 100;
  if (pct >= 85.0f) return 95;
  if (pct >= 83.0f) return 90;
  if (pct >= 10.0f) return (int)(pct + 0.5f);
  return 1;
}

BypassCapacity bypassCapacity(int cellCount, int percent) {
  BypassCapacity capacity;
  capacity.full = cellCount * BYPASS_CELL_CAPACITY_MAH;
  capacity.remain = capacity.full * percent / 100;
  return capacity;
}
