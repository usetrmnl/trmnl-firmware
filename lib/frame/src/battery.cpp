#include <trmnl_log.h>
#include "battery.h"
//#include "globals.h"
#include "pindefine.h"
#include <Arduino.h>
#include <Wire.h>

#include "TCA9555.h"

// Global TCA9555 expander objects
// These are instantiated in main.cpp and available to all modules
extern TCA9555 expander1;  // First expander at address 0x20
extern TCA9555 expander2;  // Second expander at address 0x21

// Global interrupt flags
extern volatile bool expander1_interrupt;
extern volatile bool expander2_interrupt;

#define BQ27427_ADDR        0x55

// BQ27427-G1 standard commands (16-bit, little-endian)
#define BQ27427_REG_VOLTAGE     0x04   // mV, unsigned
#define BQ27427_REG_FLAGS       0x06   // status bits
#define BQ27427_REG_AVG_CURRENT 0x10   // mA, signed
#define BQ27427_REG_SOC         0x28   // 0–100 %

// Flags register bits
#define BQ27427_FLAG_DSG  (1 << 0)    // 1 = discharging
#define BQ27427_FLAG_FC   (1 << 8)    // 1 = fully charged

static bool bq27427_read16(uint8_t reg, uint16_t* out)
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

BatteryStatus batteryRead()
{
    BatteryStatus s{};

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

    // BQ25616 charger status via TCA9555 expander1
    // STAT (open-drain, active LOW): LOW = charging, HIGH/HiZ = not charging
    // PG#  (open-drain, active LOW): LOW = external power good
    s.charging   = (expander1.read1(EXP1_CHARGER_status) == LOW);
    s.power_good = (expander1.read1(EXP1_CHARGER_pg)     == LOW);

    return s;
}

void batteryLog(const BatteryStatus& s)
{
    if (!s.valid) {
        Log_error("[BAT] gauge read failed (no battery or I2C error)");
        return;
    }
    Log_info("[BAT] %u mV  %u%%  %+d mA  %s  %s\n",
                  s.voltage_mV,
                  s.soc_pct,
                  s.current_mA,
                  s.charging   ? "CHARGING"    : "DISCHARGING",
                  s.power_good ? "EXT_PWR"     : "BATTERY");
}
