#include <Arduino.h>
#include <ArduinoLog.h>
#include <WifiCaptive.h>
#include <battery.h>
#include <board_config.h>
#include <button.h>
#include <display.h>
#include <filesystem.h>
#include <globals.h>
#include <misc/buzzer.h>
#include <misc/sensor.h>
#include <pins.h>
#include <power.h>
#include <preferences_persistence.h>
#include <touch_i2c_lock.h>
#include <trmnl_log.h>
#include <wake_init.h>

#include "messages.h"

#ifdef BOARD_TRMNL_X
#include <Wire.h>
#include <cached_images.h>
#include <status_screens.h>

#include "accelerometer.h"
#include "touchbar_actions.h"
#else
// Create stub functions for the touchbar workaround
void iqs323_task_i2c_lock(void) {}
void iqs323_task_i2c_unlock(void) {}
#endif // !BOARD_TRMNL_X

void wait_for_serial() {
#ifdef WAIT_FOR_SERIAL
  int idx = 0;
  unsigned long start = millis();
  while (millis() - start < 2000) {
      if (Serial)
        break;
      delay(100);
      idx++;
    }
  Log_info("## Waited for serial.. %d ms", idx * 100);
#endif
}

uint32_t bootEarly(void)
{
#ifdef BOARD_SEEED_STICKY
  pinMode(45, OUTPUT); // power hold (DATA)
  pinMode(46, OUTPUT); // power lock (CLK)
  digitalWrite(45, 1);
  digitalWrite(46, 0);
  digitalWrite(46, 1); // Hold the battery power enabled for after the user releases the power button
#endif
#ifdef BOARD_TRMNL_X
  uint32_t init_time = esp_cpu_get_cycle_count() / esp_rom_get_cpu_ticks_per_us();
#else
  uint32_t init_time = micros();
  startup_time = init_time/1000L; // convert to milliseconds
#endif
#ifdef DEV_FIRMWARE
  Serial.begin(115200);
  wait_for_serial();
  Log.begin(LOG_LEVEL_VERBOSE, &Serial);
#endif
  Log_info("BL init success");
  return init_time;
}

void initPeripherals(void)
{
  hw_config_init();
  pins_init();
  buzzer().init();
  sensor().init(pDevice);
#ifdef BOARD_TRMNL_X
  touchbar_log_wake_stub_status();
#endif

#if defined(BOARD_SEEED_XIAO_ESP32C3)
  delay(2000);

  if (digitalRead(PIN_INTERRUPT) == LOW) {
    Log_info("Boot button pressed during startup, resetting WiFi credentials...");
    WifiCaptivePortal.resetSettings();
    Log_info("WiFi credentials reset completed");
  }
#endif
}

WakeContext readWakeContext(void)
{
  wakeup_reason = esp_sleep_get_wakeup_cause();
  bool should_show_error_now = (wakeup_reason != ESP_SLEEP_WAKEUP_TIMER); // don't immediately show errors on timer wakeup
  bool gpio_wakeup = (wakeup_reason == ESP_SLEEP_WAKEUP_GPIO ||
                      wakeup_reason == ESP_SLEEP_WAKEUP_EXT0 ||
                      wakeup_reason == ESP_SLEEP_WAKEUP_EXT1);
  Log.info("%s [%d]: Wake reason: %d\r\n", __FILE__, __LINE__, (int)wakeup_reason);
  return WakeContext{wakeup_reason, gpio_wakeup, should_show_error_now};
}

void openPreferences(void)
{
  Log_info("preferences start");
  bool res = preferences.begin("data", false);
  if (res)
  {
    Log_info("preferences init success (%d free entries)", preferences.freeEntries());
  }
  else
  {
    Log_fatal("preferences init failed");
    ESP.restart();
  }
  Log_info("preferences end");
}

#ifndef BOARD_TRMNL_X
void startBoardHardware(const WakeContext &wake, uint32_t init_time)
{
  bool double_click = false;
  if (wake.gpio)
  {
    Log_info("GPIO wakeup detected (%d)", wake.reason);
    ButtonPressResult button = read_button_presses();
    wait_for_serial();
    double_click = button_wake_act(button);
  }
  else
  {
    wait_for_serial();
    Log_info("Non-GPIO wakeup (%d) -> didn't read buttons", wake.reason);
  }

  if (double_click)
    button_wake_run_special_function();
  // Read the battery voltage BEFORE the display or WiFi is turned on
  battery_voltage = battery().readVoltage(pDevice);

  // EPD init
  // EPD clear
  Log.info("%s [%d]: Display init\r\n", __FILE__, __LINE__);
  display_init();

  // Mount SPIFFS
  filesystem_init();
}
#else // BOARD_TRMNL_X
static void handle_orientation_change_confirmation(struct bma5_dev *dev, uint8_t previous_orientation)
{
  const unsigned long ORIENTATION_POLL_INTERVAL_MS = 1000;
  int dots_filled = 0;
  bool reverted = false;
  uint8_t *confirm_logo = storedLogoOrDefault(1);

  display_show_orientation_confirm(confirm_logo, dots_filled);
  unsigned long confirm_start = millis();

  while (dots_filled < ORIENTATION_CONFIRM_DOT_COUNT) {
    delay(ORIENTATION_POLL_INTERVAL_MS);

    uint8_t polled_orientation;
    if (bma530_read_orientation(dev, &polled_orientation) == BMA530_ORIENT_VALID &&
        polled_orientation == previous_orientation) {
      reverted = true;
      break;
    }

    dots_filled++;
    display_show_orientation_confirm(confirm_logo, dots_filled);
  }

  if (reverted) {
    Log_info("Orientation change reverted within confirm window - aborting, restoring previous orientation");
    device_orientation = previous_orientation;
    display_set_orientation(device_orientation);
    show_last_image_and_sleep(); // does not return
  }

  Log_info("Orientation change confirmed after %lums", millis() - confirm_start);
}

void startBoardHardware(const WakeContext &wake, uint32_t init_time)
{
  Log.info("%s [%d]: Display init\r\n", __FILE__, __LINE__);
  iqs323_task_i2c_lock();
  display_init();
  otg_turn_off(); // Since OTG function was commented out, need to ensure that OTG is turned off
  iqs323_task_i2c_unlock();

  // Apply the last-known (RTC-persisted) orientation
  display_set_orientation(device_orientation);

  filesystem_init();

  Wire.setClock(100000);

  touchbar_start(wake.reason, wake.gpio);

  Log_info("init time: %ld us", init_time);

  // I2C already initialized by IQS323 - do not call Wire.begin() again as it corrupts the bus on ESP32S3
  Serial.printf("Using I2C bus already initialized (SDA: %d, SCL: %d)\n\n", 39, 40);

  struct bma5_dev bma530_dev;
  bma530_wake_and_init(&bma530_dev);

  config_bma530_interrupt();
  pinMode(GPIO_NUM_38, INPUT);

  uint8_t orientation_before_confirm = device_orientation;
  {
    uint8_t computed_orientation;
    if (bma530_read_orientation(&bma530_dev, &computed_orientation) == BMA530_ORIENT_VALID) {
      device_orientation = computed_orientation;
      display_set_orientation(device_orientation);
    }
  }

  if (device_orientation != orientation_before_confirm) {
    Log_info("Device orientation changed: %d -> %d", orientation_before_confirm, device_orientation);
    handle_orientation_change_confirmation(&bma530_dev, orientation_before_confirm);
  }

  // Read the gauge before the panel draws load current, and before the logo
  // is drawn so its battery icon has a snapshot to read.
  battery_count = detect_battery_count();
  battery_charging = (power().chargingStatus() == ChargingStatus::CHARGING);
  Log_info("BATTERY COUNT: %d", battery_count);
  Log_info("BATTERY CHARGING: %s", battery_charging ? "YES" : "NO");

  battery().gaugeInit();
  battery_voltage = battery().readVoltage(); // Read the battery voltage BEFORE WiFi is turned on
}
#endif // BOARD_TRMNL_X

void logVersions(void)
{
  Log_info("Firmware version %s", Messages::firmware_version().c_str());
  Log_info("Arduino version %d.%d.%d", ESP_ARDUINO_VERSION_MAJOR, ESP_ARDUINO_VERSION_MINOR, ESP_ARDUINO_VERSION_PATCH);
  Log_info("ESP-IDF version %d.%d.%d", ESP_IDF_VERSION_MAJOR, ESP_IDF_VERSION_MINOR, ESP_IDF_VERSION_PATCH);
  list_files();
  log_nvs_usage();
}
