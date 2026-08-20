#pragma once
#include <stdint.h>

struct Gen2BatteryStatus {
    uint16_t voltage_mV;   // cell voltage (mV)
    int16_t  current_mA;   // average current, negative = discharging
    uint8_t  soc_pct;      // state of charge 0-100 %
    bool     charging;     // BQ25616 STAT pin LOW = charging in progress
    bool     power_good;   // BQ25616 PG# pin LOW  = external VBUS present
    bool     valid;        // false if I2C read failed
};

// Wire must be initialised (gen2_i2cInit) before calling.
Gen2BatteryStatus gen2_batteryRead(void);
void              gen2_batteryLog(const Gen2BatteryStatus &s);
