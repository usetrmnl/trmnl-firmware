#pragma once

/// @brief Estimate state of charge from a LiPo cell voltage.
/// @param voltage cell voltage in Volts
/// @return estimated state of charge, 0-100 %
int batteryVoltageToPercent(float voltage);

/// @brief Pack capacities (mAh) to report when charge gauging is bypassed.
struct BypassCapacity {
  int full;   // mAh
  int remain; // mAh
};

/// @brief Assume a fixed capacity per cell.
/// @param cellCount number of cells in the pack
/// @param percent state of charge, 0-100 %
/// @return the full and remaining capacities to report
BypassCapacity bypassCapacity(int cellCount, int percent);
