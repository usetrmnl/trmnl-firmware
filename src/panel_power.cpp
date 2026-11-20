// Limits how long the TRMNL X waits for its panel power supply (TPS65185) to report power good
#include <panel_power.h>

#ifdef BOARD_TRMNL_X
#include <Wire.h>
#include <epd_driver.h>
#include <trmnl_log.h>

#include "driver/gpio.h"

#define POWER_GOOD_PIN        14 // TCA9535 P1_6, high once every panel rail is in regulation
// Longer than the TPS65185 gives its 6 supplies to come up: 6 x 50 ms + 4 x 6 ms between them = 324 ms
#define POWER_GOOD_TIMEOUT_MS 400

// TCA9535 pins in the order FastEPD's EPDiyV7EinkPower() switches them
static const uint8_t POWER_UP_PINS[] = {8, 9, 13, 11, 12}; // OE, GMOD, WAKEUP, PWRUP, VCOM_CTRL
static const uint8_t POWER_DOWN_PINS[] = {11, 12, 8, 9};   // WAKEUP goes low last

static BB_EINK_POWER *library_power; // FastEPD's own power function
static bool power_failed;            // stays set until the next wake, so later draws fail at once

// FASTEPD keeps its state protected and has no setter for the power function
struct PanelState : FASTEPD {
  static FASTEPDSTATE &of(FASTEPD &panel) { return panel.*(&PanelState::_state); }
};

static bool powerGoodArrives() {
  uint32_t start = millis();
  while (millis() - start < POWER_GOOD_TIMEOUT_MS) {
    if (bbep.ioRead(POWER_GOOD_PIN)) return true;
    delay(1);
  }
  return false;
}

// A TPS65185 register, or -1 when the chip does not answer. Only while WAKEUP is high.
static int powerChipRegister(uint8_t reg) {
  Wire.beginTransmission(0x68);
  Wire.write(reg);
  if (Wire.endTransmission() != 0 || Wire.requestFrom(0x68, 1) != 1) return -1;
  return Wire.read();
}

static void powerDown(FASTEPDSTATE *panel) {
  for (uint8_t pin : POWER_DOWN_PINS)
    bbep.ioWrite(pin, LOW);
  gpio_set_level((gpio_num_t)panel->panelDef.ioSPV, 0);
  delay(1);
  bbep.ioWrite(13, LOW);
}

// Powers up exactly as FastEPD does, so its own wait finds power good already there
static int powerWithTimeout(void *state, int on) {
  FASTEPDSTATE *panel = (FASTEPDSTATE *)state;
  if (!on || panel->pwr_on) return library_power(state, on);
  if (power_failed) return BBEP_IO_ERROR;

  gpio_set_level((gpio_num_t)panel->panelDef.ioSPV, 1);
  for (uint8_t pin : POWER_UP_PINS)
    bbep.ioWrite(pin, HIGH);
  if (powerGoodArrives()) return library_power(state, on);

  // INT2 names the rail that is out of regulation, PG the rails that are good (FA = all of them)
  int faults = powerChipRegister(0x07), rail_faults = powerChipRegister(0x08), rails_good = powerChipRegister(0x0F);
  powerDown(panel);
  power_failed = true;
  Log_error("Panel power good timed out after %d ms, not drawing (TPS65185 INT1 %02X INT2 %02X PG %02X)",
            POWER_GOOD_TIMEOUT_MS, faults & 0xFF, rail_faults & 0xFF, rails_good & 0xFF);
  return BBEP_IO_ERROR;
}

void panel_power_limit_wait() {
  BB_EINK_POWER *&power = PanelState::of(bbep).pfnEinkPower;
  if (!power || power == powerWithTimeout) return;
  library_power = power;
  power = powerWithTimeout;
}
#endif // BOARD_TRMNL_X
