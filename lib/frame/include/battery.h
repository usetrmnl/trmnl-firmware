#pragma once
#include <stdint.h>

// BQ27427 fuel gauge (I2C 0x55) + BQ25616 charger status via expander1
struct BatteryStatus {
    uint16_t voltage_mV;   // cell voltage
    int16_t  current_mA;   // average current, negative = discharging
    uint8_t  soc_pct;      // state of charge 0–100 %
    bool     charging;     // BQ25616 STAT pin low → charging in progress
    bool     power_good;   // BQ25616 PG# pin low → external power present
    bool     valid;        // false if I2C read failed
};

// Wire must be initialised before calling.
BatteryStatus batteryRead();
void          batteryLog(const BatteryStatus& s);
