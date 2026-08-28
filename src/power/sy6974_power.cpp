#include <Arduino.h>
#include <Wire.h>
#include <config.h>
#include <power.h>

#ifdef CHARGER_SY6974

#ifndef CHARGER_I2C_ADDR
#define CHARGER_I2C_ADDR 0x6B
#endif

static constexpr uint8_t SY6974_REG_STATUS = 0x08;
static constexpr uint8_t SY6974_VBUS_STAT_MASK = 0xE0;
static constexpr uint8_t SY6974_CHRG_STAT_MASK = 0x18;
static constexpr uint8_t SY6974_CHRG_STAT_SHIFT = 3;

bool SY6974Power::readStatus(uint8_t &status) {
  if (!busStarted) {
    CHARGER_I2C_BUS.begin(CHARGER_I2C_SDA, CHARGER_I2C_SCL);
    CHARGER_I2C_BUS.setClock(100000);
    busStarted = true;
  }

  CHARGER_I2C_BUS.beginTransmission(CHARGER_I2C_ADDR);
  CHARGER_I2C_BUS.write(SY6974_REG_STATUS);
  if (CHARGER_I2C_BUS.endTransmission(false) != 0) return false;
  if (CHARGER_I2C_BUS.requestFrom((int)CHARGER_I2C_ADDR, 1) != 1) return false;

  status = CHARGER_I2C_BUS.read();
  return true;
}

UsbStatus SY6974Power::usbStatus() {
  uint8_t status;
  if (!readStatus(status)) return UsbStatus::UNKNOWN;

  // VBUS_STAT is zero only when no input source is connected. PG_STAT cannot
  // be used because this chip keeps it asserted while running from battery.
  return (status & SY6974_VBUS_STAT_MASK) ? UsbStatus::CONNECTED : UsbStatus::DISCONNECTED;
}

ChargingStatus SY6974Power::chargingStatus() {
  uint8_t status;
  if (!readStatus(status)) return ChargingStatus::UNKNOWN;

  uint8_t chargeStatus = (status & SY6974_CHRG_STAT_MASK) >> SY6974_CHRG_STAT_SHIFT;
  return (chargeStatus == 1 || chargeStatus == 2) ? ChargingStatus::CHARGING : ChargingStatus::NOT_CHARGING;
}

#endif // CHARGER_SY6974