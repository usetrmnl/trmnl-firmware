#include <battery.h>

#ifdef INCLUDE_BQ27427

#include <ArduinoLog.h>
#include <Wire.h>
#include <trmnl_log.h>

#include "BQ27427.h"

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
  if (Wire.endTransmission(true) != 0)
    return false;
  uint8_t addr = BQ27427_I2C_ADDRESS;
  if (Wire.requestFrom(addr, len) != len)
    return false;
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
  } else if (ccSign >= 0 && !(lipo.flags() & BQ27427_FLAG_ITPOR)) {
    // The gauge has been integrating with the wrong sign: SOC climbs while
    // draining, DSG/CHG flags invert, and FullChargeCapacity is "learned"
    // above design capacity. Everything learned is untrustworthy.
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

float BQ27427Battery::readVoltage() {
  if (lipo._initialized) {
    float voltage = lipo.voltage() / 1000.0; // Convert mV to V
    Log.info("%s [%d]: Battery voltage reading from BQ27427: %.3f V\r\n", __FILE__, __LINE__, voltage);
    return voltage;
  } else {
    Log.error("%s [%d]: BQ27427 not initialized. Cannot read battery voltage.\r\n", __FILE__, __LINE__);
    return -1.0;
  }
}

#endif // INCLUDE_BQ27427
