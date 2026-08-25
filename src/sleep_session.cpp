#include <Arduino.h>
#include <ArduinoLog.h>
#include <WiFi.h>
#include <config.h>
#include <display.h>
#include <filesystem.h>
#include <globals.h>
#include <misc/clock.h>
#include <sleep_session.h>
#include <trmnl_log.h>

#include "driver/gpio.h"

#ifdef BOARD_TRMNL_X
#include <IQS323.h>
#include <iqs323_task.h>

#include "rtc_wake_stub_trmnl_x.h"
#endif

// --- Helpers still owned by bl.cpp ---
void submitStoredLogs(void);

/**
 * @brief Function to sleep preparing and go to sleep
 * @param none
 * @return none
 */
void goToSleep(void) {
  Log.info("%s [%d]: go to sleep\r\n", __FILE__, __LINE__);
  submitStoredLogs();

// DEBUG - workaround to prevent crash in the WiFi stack of unknown origin
#ifndef BOARD_X_CLASS
  if (WiFi.status() == WL_CONNECTED) {
    WiFi.disconnect();
  }
  WiFi.mode(WIFI_OFF);
#endif

#if BOARD_TRMNL_X
  Log_info("Preparing IQS323 for sleep via task...");

  // Use task manager for sleep preparation (sets event mode, checks I2C health)
  if (!iqs323_task_prepare_sleep(5000)) {
    Log.warning("IQS323 sleep preparation timeout - proceeding anyway\n");
  }

  // Configure gesture mode last so prepare_sleep's writeMM() cannot override it
  iqs323_task_i2c_lock();
  iqs323.setGestureConfig(touchbar_tap_mode, STOP);
  iqs323_task_i2c_unlock();

  // Cleanup the task before entering deep sleep
  iqs323_task_deinit();
  Log_info("IQS323 is ready for sleep.");

  esp_set_deep_sleep_wake_stub(*wakeup_stub);
  display_sleep();
  config_tca95535_pins_for_lp();
  config_gpio_for_lp();
#endif

  filesystem_deinit();
  uint32_t time_to_sleep = refreshInterval.seconds();
  iPrevWakeTime = millis() - startup_time; // save for statistics
  Log.info("%s [%d]: total awake time - %d ms\r\n", __FILE__, __LINE__, iPrevWakeTime);
  Log.info("%s [%d]: time to sleep - %d\r\n", __FILE__, __LINE__, time_to_sleep);
  preferences.putUInt(PREFERENCES_LAST_SLEEP_TIME, systemClock().getTime());
  preferences.end();
  esp_sleep_enable_timer_wakeup((uint64_t)time_to_sleep * SLEEP_uS_TO_S_FACTOR);
  // Configure GPIO pin for wakeup
#if CONFIG_IDF_TARGET_ESP32
#define BUTTON_PIN_BITMASK(GPIO) (1ULL << GPIO)  // 2 ^ GPIO_NUMBER in hex
  esp_sleep_enable_ext1_wakeup(BUTTON_PIN_BITMASK(PIN_INTERRUPT), ESP_EXT1_WAKEUP_ALL_LOW);
#elif defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C5)
  pinMode(PIN_INTERRUPT, INPUT); // needed to not immediately wake up
  esp_deep_sleep_enable_gpio_wakeup(1 << PIN_INTERRUPT, ESP_GPIO_WAKEUP_GPIO_LOW);
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
  esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_INTERRUPT, 0);
#else
#error "Unsupported ESP32 target for GPIO wakeup configuration"
#endif
#ifdef BOARD_XTEINK_X4
// The Xteink X4 has a high current draw in deep sleep (3-4mA), so allow the user to select
// if they want to completely shut down the power and only update with a physical button press
// or have short battery life (5-7 days) in the normal TRMNL wakeup mode
#ifdef X4_WAKE_ON_BUTTON
  pinMode(13, OUTPUT);
  digitalWrite(13, 0); // cut off the battery power
  delay(100); // allow it to settle before going into power-off
#else // Keep the battery power on and allow timed wakeup
  gpio_hold_en(GPIO_NUM_13); // MOSFET enabling the battery power
  gpio_deep_sleep_hold_en(); // Needed to keep the battery power enabled during RTC sleep
#endif
#endif
  esp_deep_sleep_start();
}

void goToSleepButtonOnly(void) {
  submitStoredLogs();
  if (WiFi.status() == WL_CONNECTED) {
    WiFi.disconnect();
  }
  WiFi.mode(WIFI_OFF);
  filesystem_deinit();
  preferences.end();
#if CONFIG_IDF_TARGET_ESP32
#define BUTTON_PIN_BITMASK_BTN(GPIO) (1ULL << GPIO)
  esp_sleep_enable_ext1_wakeup(BUTTON_PIN_BITMASK_BTN(PIN_INTERRUPT), ESP_EXT1_WAKEUP_ALL_LOW);
#elif defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C5)
  esp_deep_sleep_enable_gpio_wakeup(1 << PIN_INTERRUPT, ESP_GPIO_WAKEUP_GPIO_LOW);
#elif CONFIG_IDF_TARGET_ESP32S3
  esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_INTERRUPT, 0);
#else
#error "Unsupported ESP32 target for GPIO wakeup configuration"
#endif
#ifdef BOARD_XTEINK_X4
  gpio_hold_en(GPIO_NUM_13);
  gpio_deep_sleep_hold_en();
#endif
  esp_deep_sleep_start();
}
void config_gpio_for_lp() {

#ifdef BOARD_TRMNL_X
  // XCL
  pinMode(GPIO_NUM_4, INPUT);

  // Data pins (d8 to d15)
  pinMode(GPIO_NUM_5, INPUT);
  pinMode(GPIO_NUM_6, INPUT);
  pinMode(GPIO_NUM_7, INPUT);
  pinMode(GPIO_NUM_15, INPUT);
  pinMode(GPIO_NUM_16, INPUT);
  pinMode(GPIO_NUM_17, INPUT);
  pinMode(GPIO_NUM_18, INPUT);
  pinMode(GPIO_NUM_8, INPUT);

  // D+ D-
  // pinMode(GPIO_NUM_19, INPUT);
  // pinMode(GPIO_NUM_20, INPUT);

  // Data pins (d0 to d7)
  pinMode(GPIO_NUM_9, INPUT);
  pinMode(GPIO_NUM_10, INPUT);
  pinMode(GPIO_NUM_11, INPUT);
  pinMode(GPIO_NUM_12, INPUT);
  pinMode(GPIO_NUM_13, INPUT);
  pinMode(GPIO_NUM_14, INPUT);
  pinMode(GPIO_NUM_21, INPUT);
  pinMode(GPIO_NUM_47, INPUT);

  // EP_STV
  pinMode(GPIO_NUM_48, INPUT);

  // CKV
  pinMode(GPIO_NUM_45, INPUT);

  // BTN1
  pinMode(GPIO_NUM_0, INPUT);

  // I2C
  pinMode(GPIO_NUM_39, INPUT); // SDA
  pinMode(GPIO_NUM_40, INPUT); // SCL

  // XSTL
  pinMode(GPIO_NUM_41, INPUT);

  // LEH
  pinMode(GPIO_NUM_42, INPUT);

  // UART0
  pinMode(GPIO_NUM_43, INPUT); // TXD
  pinMode(GPIO_NUM_44, INPUT); // RXD
  pinMode(GPIO_NUM_1, INPUT); // CTS
  pinMode(GPIO_NUM_2, INPUT); // RTS
#endif // BOARD_TRMNL_X
} /* config_gpio_for_lp() */
