#include <battery.h>
#include <config.h>

#ifdef INCLUDE_BQ27427

#include <Arduino.h>
#include <display.h>
#include <globals.h>
#include <trmnl_log.h>

#include "BQ27427.h"
#include "driver/gpio.h"
#ifdef BOARD_TRMNL_X
#include "iqs323_task.h"
#else // TRMNL OG gen2
// stub functions to make the code less cluttered
extern void iqs323_task_i2c_unlock(void);
extern void iqs323_task_i2c_lock(void);
#endif

/// @brief Per-cell pack capacity (mAh) assumed when BYPASS_BQ27427_SOC is enabled.
#define BQ27427_BYPASS_CELL_CAPACITY_MAH 6000

/// Connect to the BQ27427, run the golden-file/Impedance Track path, and
/// fill in a validated snapshot of its readings.
static bool connectAndRead(bool oneCellPack, BQ27427Snapshot &snap) {
  bool ok = lipo.connectAndConfigure(pDevice->sensor_sda, pDevice->sensor_scl, oneCellPack) && lipo.readSnapshot(snap);

  // Real gas-gauge values, kept for the Gauge-* comparison headers even when
  // BYPASS_BQ27427_SOC below overrides soc/capacity.
  snap.gaugeSoc = ok ? snap.soc : -1;
  snap.gaugeHealth = ok ? snap.health : -1;
  snap.gaugeCapacityRemain = ok ? snap.capacityRemain : -1;
  snap.gaugeCapacityFull = ok ? snap.capacityFull : -1;

#ifdef BYPASS_BQ27427_SOC
  if (ok) {
  // Estimate SoC from the voltage just read. Mirrors the server's
  // percent_charged_calculation: map 3.0 V onto 0 % at 0.012 V per percent,
  // with plateaus near full charge (4.08 V follows a full charge) and a
  // 1 % floor.
    float voltage = snap.voltage / 1000.0f;
    float pct = (voltage - 3.0f) / 0.012f;
    if (pct >= 88.0f)
      snap.soc = 100;
    else if (pct >= 85.0f)
      snap.soc = 95;
    else if (pct >= 83.0f)
      snap.soc = 90;
    else if (pct >= 10.0f)
      snap.soc = (int)(pct + 0.5f);
    else
      snap.soc = 1;
    snap.capacityFull = battery_count * BQ27427_BYPASS_CELL_CAPACITY_MAH;
    snap.capacityRemain = snap.capacityFull * snap.soc / 100;
    snap.health = -1; // State-of-health unavailable without gauging
  }
#endif // BYPASS_BQ27427_SOC

  return ok;
}

//
// Specific code needed for the TRMNL OG Gen2
//
#ifdef BOARD_TRMNL_GEN2
//
// The green LED on the OG Gen2 PCB is controlled by the ESP32, not the
// battery management chip. We must explicitly turn it off when charging has
// completed or the cable is unplugged.
//
void IRAM_ATTR statChange(void)
{
  digitalWrite(GEN2_LED, 1-digitalRead(BATT_STATUS));
} /* statChange() */


battery_count_t detect_battery_count(void) {
    return BATTERY_ONE;
}

void BQ27427_reset() {
  pinMode(7, OUTPUT); // BQ27427 INT/PWR
  digitalWrite(7, LOW);
  delay(10);
  digitalWrite(7, HIGH);
  Log_info("BQ27427 reset performed");
}
#endif // GEN2

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
      gpio_dump_io_configuration(stdout, (1ULL << pDevice->sensor_sda));
      gpio_dump_io_configuration(stdout, (1ULL << pDevice->sensor_scl));
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
    _gaugeSoc = -1;
    _gaugeHealth = -1;
    _gaugeCapacityRemain = -1;
    _gaugeCapacityFull = -1;
    return;
  }

  _soc = snap.soc;
  _health = snap.health;
  _current = snap.current;
  _temperature = snap.temperature;
  _capacityRemain = snap.capacityRemain;
  _capacityFull = snap.capacityFull;
  _voltage = snap.voltage / 1000.0; // Convert mV to V
  _gaugeSoc = snap.gaugeSoc;
  _gaugeHealth = snap.gaugeHealth;
  _gaugeCapacityRemain = snap.gaugeCapacityRemain;
  _gaugeCapacityFull = snap.gaugeCapacityFull;

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

#ifdef BOARD_TRMNL_GEN2
// Set up an interrupt to monitor the battery charge status
// We need this to manually control the green LED
  pinMode(GEN2_LED, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(BATT_STATUS), statChange, CHANGE);
#endif // GEN2
  if (snap.flags & BQ27427_FLAG_FC) {
    Log_info("BATTERY IS FULL"); // full, charger connected but not drawing current
  } else if ((snap.flags & BQ27427_FLAG_CHG) && _current > 0) {
    Log_info("BATTERY IS CHARGING");
  } else if (snap.flags & BQ27427_FLAG_DSG) {
    Log_info("BATTERY IS DISCHARGING");
  } else {
    Log_info("BATTERY NOT CONNECTED?");
  }
}

#endif // INCLUDE_BQ27427
