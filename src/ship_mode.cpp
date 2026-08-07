#include <config.h>

#ifdef SHIP_MODE_SUPPORTED

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include "esp_sleep.h"
#include "trmnl_log.h"
#include "display.h"
#include "ship_mode.h"
#include "logo_medium.h"
#include "qa.h"

// SY6974B I2C charger PMIC (shared by reTerminal E1001 and E1003). Only the
// status register is needed here to know whether USB / charger power is present.
#ifndef CHARGER_I2C_ADDR
#define CHARGER_I2C_ADDR 0x6B // SY6974B 7-bit I2C address
#endif

#define SY6974_REG_STATUS 0x08 // REG08: system status register
#define SY6974_PG_STAT    0x04 // REG08[2] = 1 when VBUS power is good
#define SY6974_VBUS_STAT_MASK 0xE0 // REG08[7:5] = input source (0 = no input)

// How long to deep-sleep between charger polls while in shipping mode.
#ifndef SHIP_MODE_POLL_INTERVAL_S
#define SHIP_MODE_POLL_INTERVAL_S 3
#endif

static bool s_charger_bus_started = false;

static void charger_bus_begin()
{
    if (s_charger_bus_started) {
        return;
    }
    CHARGER_I2C_BUS.begin(CHARGER_I2C_SDA, CHARGER_I2C_SCL);
    CHARGER_I2C_BUS.setClock(100000);
    s_charger_bus_started = true;
}

static bool sy6974_read_reg(uint8_t reg, uint8_t *value)
{
    CHARGER_I2C_BUS.beginTransmission(CHARGER_I2C_ADDR);
    CHARGER_I2C_BUS.write(reg);
    if (CHARGER_I2C_BUS.endTransmission(false) != 0) {
        return false;
    }
    if (CHARGER_I2C_BUS.requestFrom((int)CHARGER_I2C_ADDR, 1) != 1) {
        return false;
    }
    *value = CHARGER_I2C_BUS.read();
    return true;
}

// Returns true only when the SY6974B reports an active VBUS input source
// (REG08[7:5] VBUS_STAT != 0). The PG_STAT bit is NOT used: on the SY6974B it
// stays asserted whenever the system rail is powered (including from battery),
// so it cannot distinguish "plugged in" from "running on battery".
bool check_usb_power()
{
    charger_bus_begin();

    uint8_t status = 0;
    if (!sy6974_read_reg(SY6974_REG_STATUS, &status)) {
        Log_error("SY6974B: failed to read status register");
        return false;
    }
    uint8_t vbus_stat = (status & SY6974_VBUS_STAT_MASK) >> 5;
    Log_info("SY6974B REG08=0x%02X (PG=%d VBUS_STAT=%d)", status,
             (status & SY6974_PG_STAT) ? 1 : 0, vbus_stat);
    return vbus_stat != 0; // non-zero VBUS source = USB / charger present
}

// Deep-sleep until the next poll or a button press. Deep sleep keeps draw
// minimal and guarantees a clean I2C re-init on the next boot. The e-paper
// retains the shipping screen without power. Does not return.
static void ship_mode_deep_sleep()
{
    // Make sure the button is released first, otherwise the active-low ext0 wake
    // would fire immediately and look like a manual exit.
    pinMode(PIN_INTERRUPT, INPUT);
    while (digitalRead(PIN_INTERRUPT) == LOW) {
        delay(10);
    }
    delay(50);

    Log_info("Ship: deep sleeping (poll in %d s, or press button to exit)", SHIP_MODE_POLL_INTERVAL_S);
    Serial.flush();

    esp_sleep_enable_timer_wakeup((uint64_t)SHIP_MODE_POLL_INTERVAL_S * 1000000ULL);
    // Green button (active low) as a manual wake / exit fallback.
    esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_INTERRUPT, 0);

    esp_deep_sleep_start();
}

// Called from setup() on every boot. If a shipment is in progress, either
// complete it (charger present, or the user pressed the button to exit) or go
// back to deep sleep to keep waiting. May not return.
void ship_mode_boot_check()
{
    bool shipped = checkIfAlreadyShipped();
    bool started = checkIfShipmentStarted();
    Log_info("Ship boot check: ship_done=%d ship_started=%d cause=%d",
             shipped, started, (int)esp_sleep_get_wakeup_cause());
    if (shipped) {
        return; // already shipped — normal boot
    }

    if (!started) {
        // Never shipped and no shipment in progress (e.g. freshly flashed at the
        // factory): default into shipping mode so the device can be boxed and
        // shipped. Completing the shipment (charger reconnect or button) sets
        // ship_done, so this only happens until the unit is first powered by the
        // customer.
        Log_info("Ship boot check: defaulting into shipping mode after flash");
        display_init();
        enter_ship_mode(); // does not return
    }

    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    bool woke_by_button = (cause == ESP_SLEEP_WAKEUP_EXT0 || cause == ESP_SLEEP_WAKEUP_GPIO);

    if (woke_by_button) {
        Log_info("Ship: button wake — exiting shipping mode");
        saveShipmentDone();
        return; // continue normal boot
    }
    if (check_usb_power()) {
        Log_info("Ship: charger detected — exiting shipping mode");
        saveShipmentDone();
        return; // continue normal boot
    }

    // Still shipping and unplugged. On a fresh / brown-out boot (anything other
    // than our own timer poll) re-draw the shipping screen so the display is
    // correct, then go back to sleep.
    if (cause != ESP_SLEEP_WAKEUP_TIMER) {
        Log_info("Ship: resuming shipping mode after reset");
        display_init();
        display_show_msg(const_cast<uint8_t *>(logo_medium), SHIPPING_MODE);
    }

    ship_mode_deep_sleep(); // does not return
}

void enter_ship_mode()
{
    Log_info("Entering shipping mode");

    // Persist shipment-in-progress immediately. Clear any stale "shipped" flag
    // from a previous shipment so this run isn't treated as already complete.
    // Removing USB can brown-out and reset the board before we reach deep sleep,
    // so this must be saved first — ship_mode_boot_check() resumes on next boot.
    clearShipmentStatus();
    saveShipmentStarted();

    // Stop the WiFi radio / setup soft-AP before settling into low-power sleep.
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);

    // If USB is still plugged in, ask the user to unplug it first.
    if (check_usb_power()) {
        display_show_msg(const_cast<uint8_t *>(logo_medium), READY_TO_SHIP);
        while (check_usb_power()) {
            Log_info("USB still connected, waiting for unplug...");
            delay(1000);
        }
        Log_info("USB unplugged");
    }

    display_show_msg(const_cast<uint8_t *>(logo_medium), SHIPPING_MODE);

    ship_mode_deep_sleep(); // does not return
}

#endif // SHIP_MODE_SUPPORTED
