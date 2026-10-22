#include <Arduino.h>
#include <ArduinoLog.h>
#include <WiFi.h>
#include <app_logger.h>
#include <bl.h>
#include <config.h>
#include <display.h>
#include <filesystem.h>
#include <globals.h>
#include <misc/clock.h>
#include <pins.h>
#include <refresh_interval.h>
#include <sleep_duration.h>
#include <sleep_session.h>
#include <status_screens.h>
#include <trmnl_log.h>

#include "driver/gpio.h"

#ifdef BOARD_TRMNL_X
#include <iqs323_task.h>
#include <touchbar_gesture.h>

#include "FastEPD.h"
#include "rtc_wake_stub_trmnl_x.h"

extern FASTEPD bbep; // display.cpp
#endif

// --- Helpers / state still owned by bl.cpp ---
extern unsigned long startup_time;
extern TRMNL_DEVICE *pDevice;

/**
 * @brief Function to sleep preparing and go to sleep
 * @param none
 * @return none
 */
void goToSleep(void) {
  Log.info("%s [%d]: go to sleep\r\n", __FILE__, __LINE__);
  submitStoredLogs();

// DEBUG - workaround to prevent crash in the WiFi stack of unknown origin
#ifndef PARALLEL_EPD
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

  touchbar_prepare_for_sleep();

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
  esp_sleep_enable_ext1_wakeup(BUTTON_PIN_BITMASK(pDevice->interrupt_pin), ESP_EXT1_WAKEUP_ALL_LOW);
#elif defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C5)
  pinMode(pDevice->interrupt_pin, INPUT); // needed to not immediately wake up
  esp_deep_sleep_enable_gpio_wakeup(1 << pDevice->interrupt_pin, ESP_GPIO_WAKEUP_GPIO_LOW);
#elif defined(CONFIG_IDF_TARGET_ESP32S3)
#ifdef BOARD_TRMNL_X
  esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_INTERRUPT, 0);
#else
  esp_sleep_enable_ext0_wakeup((gpio_num_t)pDevice->interrupt_pin, 0);
#endif
#else
#error "Unsupported ESP32 target for GPIO wakeup configuration"
#endif
#if defined(BOARD_XTEINK_X4) || defined(BOARD_XTEINK_X3)
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

void updateTimeSinceSleep(void) {
  if (systemClock().setTimeFromNTP()) {
    const uint32_t last_sleep = preferences.getUInt(PREFERENCES_LAST_SLEEP_TIME, 0);
    time_since_sleep = sleepDurationSeconds(systemClock().getTime(), last_sleep);
  } else {
    time_since_sleep = 0;
    Log.info("%s [%d]: Time wasn't synced.\r\n", __FILE__, __LINE__);
  }

  Log.info("%s [%d]: Time since last sleep: %d\r\n", __FILE__, __LINE__, time_since_sleep);
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
  esp_sleep_enable_ext1_wakeup(BUTTON_PIN_BITMASK_BTN(pDevice->interrupt_pin), ESP_EXT1_WAKEUP_ALL_LOW);
#elif defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C5)
  esp_deep_sleep_enable_gpio_wakeup(1 << pDevice->interrupt_pin, ESP_GPIO_WAKEUP_GPIO_LOW);
#elif CONFIG_IDF_TARGET_ESP32S3
#ifdef BOARD_TRMNL_X
  esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_INTERRUPT, 0);
#else
  esp_sleep_enable_ext0_wakeup((gpio_num_t)pDevice->interrupt_pin, 0);
#endif
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

void wifiErrorDeepSleep(void) {
  if (!preferences.isKey(PREFERENCES_CONNECT_WIFI_RETRY_COUNT)) {
    preferences.putInt(PREFERENCES_CONNECT_WIFI_RETRY_COUNT, 1);
  }

  uint8_t retry_count = preferences.getInt(PREFERENCES_CONNECT_WIFI_RETRY_COUNT);

  Log_info("WIFI connection failed! Retry count: %d \n", retry_count);

  refreshInterval.applyWifiRetry(retry_count);

  if (retry_count >= MAX_QUIET_SLOW_RETRIES) {
    preferences.putInt(PREFERENCES_CONNECT_WIFI_RETRY_COUNT, 1);
    showMessageWithLogo(WIFI_FAILED);
    display_sleep();
    goToSleep();
    return;
  }
  retry_count++;
  preferences.putInt(PREFERENCES_CONNECT_WIFI_RETRY_COUNT, retry_count);

  display_sleep();
  goToSleep();
}

#ifdef BOARD_TRMNL_X

#define TCA9535_INT 38

void config_tca95535_pins_for_lp() {
  bbep.ioPinMode(0, INPUT);

  // Pin 1 (OTG) is left as otg_turn_on()/otg_turn_off() set it

  bbep.ioPinMode(2, INPUT);

  bbep.ioPinMode(3, INPUT);

  bbep.ioPinMode(4, INPUT);

  bbep.ioPinMode(5, INPUT);

  bbep.ioPinMode(6, OUTPUT);
  bbep.ioWrite(6, LOW);

  bbep.ioPinMode(7, INPUT);

  bbep.ioPinMode(8, OUTPUT);
  bbep.ioWrite(8, LOW);

  bbep.ioPinMode(9, OUTPUT);
  bbep.ioWrite(9, LOW);

  bbep.ioPinMode(10, INPUT);

  bbep.ioPinMode(11, OUTPUT);
  bbep.ioWrite(11, LOW);

  bbep.ioPinMode(12, OUTPUT);
  bbep.ioWrite(12, LOW);

  bbep.ioPinMode(13, OUTPUT);
  bbep.ioWrite(13, LOW);

  bbep.ioPinMode(14, INPUT);

  bbep.ioPinMode(15, INPUT);
}

void enter_shipment_sleep() {
  config_tca95535_pins_for_lp();
  bbep.ioPinMode(0, INPUT);  // Pin 0 = charger detect

  // Clear any pending TCA9535 interrupts
  for (uint8_t pin = 0; pin < 16; pin++) {
    bbep.ioPinMode(pin, INPUT);
    bbep.ioRead(pin);
  }
  delay(50);

  pinMode(TCA9535_INT, INPUT_PULLUP);

  // Check initial GPIO38 state
  int gpio38_state = digitalRead(TCA9535_INT);

  // display_sleep() leaves its light-sleep timer armed, which would wake us every second.
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);

  // Enable GPIO wakeup for light sleep
  esp_err_t err = esp_sleep_enable_gpio_wakeup();
  if (err != ESP_OK) {
    Serial.printf("ERROR: Failed to enable GPIO wakeup: %d\n", err);
    return;
  }

  err = gpio_wakeup_enable((gpio_num_t)TCA9535_INT, GPIO_INTR_LOW_LEVEL);
  if (err != ESP_OK) {
    Serial.printf("ERROR: Failed to configure GPIO38 wakeup: %d\n", err);
    return;
  }

  Serial.println("=== Entering shipment mode light sleep loop ===");
  delay(100);

  uint32_t sleep_count = 0;

  // Main sleep loop
  while (true) {
    sleep_count++;

    esp_light_sleep_start();

    // Re-initialize Serial after light sleep (USB may have been disabled)
    delay(100);
    Serial.begin(115200);
    delay(100);

    Serial.println("\n=== WAKEUP from light sleep ===");
    Serial.printf("Sleep cycle: %lu\n", sleep_count);

    // Check wakeup cause
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    Serial.printf("Wakeup cause: %d (GPIO=%d)\n", wakeup_reason, ESP_SLEEP_WAKEUP_GPIO);

    if (wakeup_reason == ESP_SLEEP_WAKEUP_GPIO) {
      Serial.println("GPIO wakeup detected");

      // Check GPIO38 state
      gpio38_state = digitalRead(TCA9535_INT);
      Serial.printf("GPIO38 state after wakeup: %d\n", gpio38_state);

      // Check if charger connected (TCA9535 pin 0 LOW)
      bbep.ioPinMode(0, INPUT);
      delay(10);
      uint8_t pin0_state = bbep.ioRead(0);
      Serial.printf("TCA9535 pin 0 (charger detect): %d (0=charger present)\n", pin0_state);

      if (pin0_state == 0) {
        Serial.println("*** CHARGER DETECTED - Exiting shipment mode ***");
        Serial.flush();
        delay(100);
        break;  // Exit shipment mode
      }

      // False wakeup - clear interrupt and continue
      for (uint8_t pin = 0; pin < 16; pin++) {
        bbep.ioPinMode(pin, INPUT);
        bbep.ioRead(pin);
      }
      delay(50);

      gpio38_state = digitalRead(TCA9535_INT);
      Serial.printf("GPIO38 after clear: %d\n", gpio38_state);

    } else {
      Serial.printf("Unexpected wakeup cause: %d\n", wakeup_reason);
    }

    Serial.println("Returning to sleep...\n");
    Serial.flush();
    delay(100);
  }

  gpio_wakeup_disable((gpio_num_t)TCA9535_INT);
  gpio_set_intr_type((gpio_num_t)TCA9535_INT, GPIO_INTR_DISABLE);
  Serial.println("=== Exited shipment mode successfully ===");
  Serial.flush();
}

#endif // BOARD_TRMNL_X
