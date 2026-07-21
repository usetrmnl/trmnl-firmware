// BQ27427-G1 fuel gauge + BQ25616 charger status for TRMNL Gen2.
// BQ27427 at I2C 0x55 — reads via Wire directly (no external library needed).
// BQ25616 STAT/PG are open-drain outputs wired to expander1 pins 14/15.

#include "gen2_battery.h"
#include "gen2_comm.h"
#include "gen2_pins.h"
#include <Arduino.h>
#include <Wire.h>

#define BQ27427_ADDR            0x55
#define BQ27427_REG_VOLTAGE     0x04
#define BQ27427_REG_FLAGS       0x06
#define BQ27427_REG_AVG_CURRENT 0x10
#define BQ27427_REG_SOC         0x28

static bool bq27427_read16(uint8_t reg, uint16_t *out)
{
    Wire.beginTransmission(BQ27427_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint8_t)BQ27427_ADDR, (uint8_t)2) != 2) return false;
    uint16_t lo = Wire.read();
    uint16_t hi = Wire.read();
    *out = lo | (hi << 8);
    return true;
}

Gen2BatteryStatus gen2_batteryRead(void)
{
    Gen2BatteryStatus s{};

    uint16_t voltage = 0, flags = 0, current = 0, soc = 0;
    bool ok = bq27427_read16(BQ27427_REG_VOLTAGE,     &voltage)
           && bq27427_read16(BQ27427_REG_FLAGS,       &flags)
           && bq27427_read16(BQ27427_REG_AVG_CURRENT, &current)
           && bq27427_read16(BQ27427_REG_SOC,         &soc);

    if (!ok || voltage == 0 || voltage > 5000) {
        s.valid = false;
        return s;
    }

    s.voltage_mV = voltage;
    s.current_mA = (int16_t)current;
    s.soc_pct    = (uint8_t)(soc & 0xFF);
    s.valid      = true;

    // BQ25616 charger status via TCA9555 expander1 (open-drain, active LOW)
    s.charging   = (expander1.read1(EXP1_CHARGER_status) == LOW);
    s.power_good = (expander1.read1(EXP1_CHARGER_pg)     == LOW);

    return s;
}

void gen2_batteryLog(const Gen2BatteryStatus &s)
{
    if (!s.valid) {
        Serial.println("[BAT] gauge read failed");
        return;
    }
    Serial.printf("[BAT] %u mV  %u%%  %+d mA  %s  %s\n",
                  s.voltage_mV, s.soc_pct, s.current_mA,
                  s.charging   ? "CHARGING"    : "DISCHARGING",
                  s.power_good ? "EXT_PWR"     : "BATTERY");
}
