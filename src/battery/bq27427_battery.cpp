#include <battery.h>

#ifdef INCLUDE_BQ27427

#include <Arduino.h>
#include <config.h>
#include <display.h>
#include <globals.h>
#include <trmnl_log.h>

#include "BQ27427.h"
#include "driver/gpio.h"
#include "iqs323_task.h"

/// @brief Per-cell pack capacity (mAh) assumed when BYPASS_BQ27427_SOC is enabled.
#define BQ27427_BYPASS_CELL_CAPACITY_MAH 6000

/// Connect to the BQ27427 and fill in a validated snapshot of its readings.
static bool connectAndRead(bool oneCellPack, BQ27427Snapshot &snap) {
#ifdef BYPASS_BQ27427_SOC
  // Charge gauging is bypassed: the IT algorithm's SoC/capacity values are
  // never used, so skip the golden-file configuration and ITPOR wait. The
  // chip only supplies direct measurements (voltage, current, temperature);
  // SoC comes from the voltage and capacity from a fixed per-cell pack size.
  (void)oneCellPack;

// State of charge (%) from the BQ27427. With BYPASS_BQ27427_SOC the gauge's
// SoC is ignored and estimated from the measured voltage instead.

  // Mirrors the server's percent_charged_calculation: map 3.0 V onto 0 % at
  // 0.012 V per percent, with plateaus near full charge (4.08 V follows a
  // full charge) and a 1 % floor.

  int soc;
  float voltage = lipo.voltage() / 1000.0f;
  float pct = (voltage - 3.0f) / 0.012f;
  if (pct >= 88.0f)
    soc = 100;
  else if (pct >= 85.0f)
    soc = 95;
  else if (pct >= 83.0f)
    soc = 90;
  else if (pct >= 10.0f)
    soc = (int)(pct + 0.5f);
  else
    soc = 1;

  if (!lipo.begin(PIN_INTERNAL_SDA, PIN_INTERNAL_SCL)) return false;
  lipo._initialized = true;
  snap.flags = lipo.flags();
  snap.energyScale = 1;
  snap.voltage = lipo.voltage();                                     // mV
  snap.current = lipo.current(AVG);                                  // mA
  snap.temperature = float(lipo.temperature(BATTERY) - 2732) / 10.0; // C
  snap.soc = soc;
  snap.capacityFull = battery_count * BQ27427_BYPASS_CELL_CAPACITY_MAH;
  snap.capacityRemain = snap.capacityFull * snap.soc / 100;
  snap.health = -1; // State-of-health unavailable without gauging
  // A failed I2C transaction reads back as 0xFFFF (65535 mV), well outside a
  // plausible pack voltage.
  return snap.voltage <= 10000 && snap.temperature >= -40.0 && snap.temperature <= 100.0;
#else
  return lipo.connectAndConfigure(PIN_INTERNAL_SDA, PIN_INTERNAL_SCL, oneCellPack) && lipo.readSnapshot(snap);
#endif // BYPASS_BQ27427_SOC
}

void BQ27427Battery::gaugeInit() {
  if (battery_count == BATTERY_NONE) {
    Log_info("No battery detected - skipping BQ27427 initialization");
    return;
  }

  bool oneCellPack = (battery_count == BATTERY_ONE);

  iqs323_task_i2c_lock();

  BQ27427Snapshot snap;
  bool readingsValid = connectAndRead(oneCellPack, snap);

  if (!readingsValid) {
    Log_warn("BQ27427: init failed or invalid readings - resetting and retrying");

    BQ27427_reset();
    iqs323_task_i2c_unlock();
    delay(300); // BQ27427 needs 250 ms to power up
    iqs323_task_i2c_lock();

    readingsValid = connectAndRead(oneCellPack, snap);

    if (!readingsValid) {
      Log_error("BQ27427: still not initialized or invalid readings after retry.");
      gpio_dump_io_configuration(stdout, (1ULL << PIN_INTERNAL_SDA));
      gpio_dump_io_configuration(stdout, (1ULL << PIN_INTERNAL_SCL));
    }
  }

  iqs323_task_i2c_unlock();

  if (!readingsValid) {
    _soc = -1;
    _health = -1;
    _current = -1;
    _temperature = -1;
    _capacityRemain = -1;
    _capacityFull = -1;
    _voltage = -1;
    return;
  }

  _soc = snap.soc;
  _health = snap.health;
  _current = snap.current;
  _temperature = snap.temperature;
  _capacityRemain = snap.capacityRemain;
  _capacityFull = snap.capacityFull;
  _voltage = snap.voltage / 1000.0; // Convert mV to V

  // Assemble a string to print
  String toPrint = "[" + String(millis() / 1000) + "] ";
  toPrint += String(_soc) + "% | ";
  toPrint += String(_temperature, 1) + " C | ";
  toPrint += String(snap.voltage) + " mV | ";
  toPrint += String(_current) + " mA | ";
  toPrint += String(_capacityRemain) + " / ";
  toPrint += String(_capacityFull) + " mAh | ";
  toPrint += String(_health) + "% | scale=" + String(snap.energyScale);
  if (snap.flags & BQ27427_FLAG_CHG) toPrint += " CHG"; // fast charging allowed
  if (snap.flags & BQ27427_FLAG_FC) toPrint += " FC";   // full charge detected
  if (snap.flags & BQ27427_FLAG_DSG) toPrint += " DSG"; // battery is discharging
  Serial.println(toPrint);

  if (snap.flags & BQ27427_FLAG_FC) {
    Log_info("BATTERY IS FULL"); // full, charger connected but not drawing current
  } else if (snap.flags & BQ27427_FLAG_CHG) {
    Log_info("BATTERY IS CHARGING");
  } else if (snap.flags & BQ27427_FLAG_DSG) {
    Log_info("BATTERY IS DISCHARGING");
  }
}

#endif // INCLUDE_BQ27427
