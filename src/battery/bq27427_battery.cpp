#include <battery.h>

#ifdef INCLUDE_BQ27427

#include <Arduino.h>
#include <Wire.h>
#include <config.h>
#include <display.h>
#include <globals.h>
#include <trmnl_log.h>

#include "BQ27427.h"
#include "driver/gpio.h"
#include "iqs323_task.h"

/// @brief Per-cell pack capacity (mAh) assumed when BYPASS_BQ27427_SOC is enabled.
#define BQ27427_BYPASS_CELL_CAPACITY_MAH 6000

// Raw I2C helpers with error checking (the library's own transactions ignore
// NACKs). The gauge shares the bus already initialized by lipo.begin().
static bool gaugeWrite(uint8_t reg, const uint8_t *data, uint8_t len) {
  Wire.beginTransmission(BQ27427_I2C_ADDRESS);
  Wire.write(reg);
  Wire.write(data, len);
  return Wire.endTransmission(true) == 0;
}

static bool gaugeRead(uint8_t reg, uint8_t *data, uint8_t len) {
  Wire.beginTransmission(BQ27427_I2C_ADDRESS);
  Wire.write(reg);
  if (Wire.endTransmission(true) != 0) return false;
  uint8_t addr = BQ27427_I2C_ADDRESS;
  if (Wire.requestFrom(addr, len) != len) return false;
  for (uint8_t i = 0; i < len; i++)
    data[i] = Wire.read();
  return true;
}

static uint16_t gaugeFlags() {
  uint8_t d[2] = {0xFF, 0xFF};
  gaugeRead(BQ27427_COMMAND_FLAGS, d, 2);
  return ((uint16_t)d[1] << 8) | d[0];
}

static bool gaugeControl(uint16_t subcommand) {
  uint8_t cmd[2] = {(uint8_t)(subcommand & 0xFF), (uint8_t)(subcommand >> 8)};
  return gaugeWrite(0x00, cmd, 2);
}

/// Connect to the BQ27427 and fill in a validated snapshot of its readings.
bool BQ27427Battery::connectAndRead(bool oneCellPack, BQ27427Snapshot &snap) {
#ifdef BYPASS_BQ27427_SOC
  // Charge gauging is bypassed: the IT algorithm's SoC/capacity values are
  // never used, so skip the golden-file configuration and ITPOR wait. The
  // chip only supplies direct measurements (voltage, current, temperature);
  // SoC comes from the voltage and capacity from a fixed per-cell pack size.
  (void)oneCellPack;

  if (!lipo.begin(PIN_INTERNAL_SDA, PIN_INTERNAL_SCL)) return false;
  lipo._initialized = true;
  snap.flags = lipo.flags();
  snap.energyScale = 1;
  snap.voltage = lipo.voltage();                                     // mV
  snap.current = lipo.current(AVG);                                  // mA
  snap.temperature = float(lipo.temperature(BATTERY) - 2732) / 10.0; // C

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
  // A failed I2C transaction reads back as 0xFFFF (65535 mV), well outside a
  // plausible pack voltage.
  return snap.voltage <= 10000 && snap.temperature >= -40.0 && snap.temperature <= 100.0;
#else
  bool ready = lipo.connectAndConfigure(PIN_INTERNAL_SDA, PIN_INTERNAL_SCL, oneCellPack);
  if (ready) {
    // A reset/golden-file reload restores the inverted ROM default sign.
    correctCurrentPolarity();
  }
  return ready && lipo.readSnapshot(snap);
#endif // BYPASS_BQ27427_SOC
}

void BQ27427Battery::gaugeInit() {
  if (battery_count == BATTERY_NONE) {
    Log_info("No battery detected - skipping BQ27427 initialization");
    return;
  }

  bool oneCellPack = (battery_count == BATTERY_ONE);

  iqs323_task_i2c_lock();

#ifndef BYPASS_BQ27427_SOC
  // Early bq27427 batches ship with an inverted coulomb-counter calibration
  // sign, making the gauge count discharge as charge: SOC climbs while
  // draining and FullChargeCapacity is "learned" above design capacity.
  // A gauge that has been running that way must be reset BEFORE
  // connectAndConfigure(), so the resulting ITPOR triggers its golden-file
  // reload and Impedance Track relearns from scratch.
  if (lipo.begin(PIN_INTERNAL_SDA, PIN_INTERNAL_SCL)) {
    resetIfPolarityInverted();
  }
#endif

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

bool BQ27427Battery::readGaugeBlockVerified(uint8_t classID, uint8_t blockNum, uint8_t data[32]) {
  for (int attempt = 0; attempt < 3; attempt++) {
    uint8_t zero = 0x00;
    if (!gaugeWrite(BQ27427_EXTENDED_CONTROL, &zero, 1)) // BlockDataControl(): enable access
      return false;
    delayMicroseconds(200);
    if (!gaugeWrite(BQ27427_EXTENDED_DATACLASS, &classID, 1))
      return false;
    delayMicroseconds(200);
    gaugeWrite(BQ27427_EXTENDED_DATABLOCK, &blockNum, 1);
    delay(5); // allow the selected block to load into the BlockData() window

    uint8_t deviceCsum;
    if (!gaugeRead(BQ27427_EXTENDED_BLOCKDATA, data, 32) ||
        !gaugeRead(BQ27427_EXTENDED_CHECKSUM, &deviceCsum, 1))
      continue;

    uint8_t csum = 0;
    for (int i = 0; i < 32; i++)
      csum += data[i];
    csum = 255 - csum;

    // A mismatch can mean a glitched read or the gauge updating the block
    // mid-read — retry either way
    if (csum == deviceCsum)
      return true;
  }
  return false;
}

int32_t BQ27427Battery::readCCCalSignByte() {
  uint8_t block[32];
  if (!readGaugeBlockVerified(BQ27427_ID_CC_CAL, 0, block))
    return -1;
  return block[5];
}

bool BQ27427Battery::writeCCCalSignByte(uint8_t value) {
  // Data memory update sequence from TRM section 4.1: enter CONFIG UPDATE,
  // select the block, replace the byte, commit via checksum, SOFT_RESET out.
  if (!gaugeControl(BQ27427_CONTROL_SET_CFGUPDATE))
    return false;
  unsigned long t0 = millis();
  while (!(gaugeFlags() & BQ27427_FLAG_CFGUPMODE)) {
    if (millis() - t0 > 2000)
      return false; // never entered CONFIG UPDATE (sealed?)
    delay(10);
  }

  bool wrote = false;
  uint8_t block[32];
  if (readGaugeBlockVerified(BQ27427_ID_CC_CAL, 0, block)) {
    block[5] = value;
    uint8_t csum = 0;
    for (int i = 0; i < 32; i++)
      csum += block[i];
    csum = 255 - csum;

    // The block is still selected from the verified read above. Writing the
    // checksum to 0x60 is what commits the block to data memory.
    wrote = gaugeWrite(BQ27427_EXTENDED_BLOCKDATA + 5, &value, 1) &&
            gaugeWrite(BQ27427_EXTENDED_CHECKSUM, &csum, 1);
    delay(10);
  }

  // Exit CONFIG UPDATE whether or not the write landed
  gaugeControl(BQ27427_CONTROL_SOFT_RESET);
  t0 = millis();
  while (gaugeFlags() & BQ27427_FLAG_CFGUPMODE) {
    if (millis() - t0 > 2000)
      break;
    delay(10);
  }

  return wrote && readCCCalSignByte() == value;
}

void BQ27427Battery::resetIfPolarityInverted() {
  int32_t ccSign = readCCCalSignByte();
  if (ccSign >= 0 && !(ccSign & 0x80)) {
    Log_info("BQ27427: CC calibration sign OK (CC_CAL[5]=0x%02lX) - no reset required",
             (unsigned long)ccSign);
  } else if (ccSign >= 0 && !(gaugeFlags() & BQ27427_FLAG_ITPOR)) {
    // The gauge has been integrating with the wrong sign: everything it has
    // learned is untrustworthy.
    Log_info("BQ27427: inverted CC calibration (CC_CAL[5]=0x%02lX) with learned state - resetting gauge",
             (unsigned long)ccSign);
    lipo.reset();
    delay(300); // POR + INITIALIZATION time before the gauge responds again
  }
}

void BQ27427Battery::correctCurrentPolarity() {
  int32_t ccSign = readCCCalSignByte();
  if (ccSign < 0) {
    Log_error("BQ27427: could not read CC calibration - current polarity unverified");
  } else if (ccSign & 0x80) {
    uint8_t corrected = (uint8_t)ccSign & 0x7F;
    if (writeCCCalSignByte(corrected)) {
      Log_info("BQ27427: CC calibration sign corrected (0x%02lX -> 0x%02X)",
               (unsigned long)ccSign, corrected);
    } else {
      Log_error("BQ27427: CC calibration sign correction FAILED");
    }
  }
}

#endif // INCLUDE_BQ27427
