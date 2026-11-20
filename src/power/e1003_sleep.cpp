// Deep-sleep power fix for the Seeed reTerminal E1003. See include/e1003_sleep.h.
// This file compiles to nothing on other boards.

#include <config.h>

#if defined(BOARD_SEEED_RETERMINAL_E1003)

#include <Arduino.h>
#include <driver/gpio.h>
#include <driver/rtc_io.h>
#include <e1003_sleep.h>
#include <esp_sleep.h>
#include <trmnl_log.h>

// Pins that must be LOW while the device sleeps. All are power-enable lines
// (LOW = off) except GPIO48, which is a reset line (LOW = held in reset).
//
//   GPIO11  EPD_Drive_EN   e-paper high-voltage rail (TPS65185)
//   GPIO21  ITE_VCC_EN     e-paper controller (IT8951)
//   GPIO38  PDM_EN         microphone, unused by this firmware
//   GPIO39  SD_EN          microSD slot, unused by this firmware
//   GPIO48  TOUCH_RES      GT911 touch controller reset. This line has an
//                          external pull-up, so when left alone it sits HIGH
//                          and the GT911 scans for touches all night (~5 mA).
//                          TRMNL firmware never uses touch on this board.
//
// Pins deliberately left out:
//   GPIO16  USER_LED       active-low; holding it LOW would light the LED
//   GPIO40  VBAT_EN        battery ADC switch, managed by the battery code
//   GPIO45  BUZZER_EN      ESP32-S3 strapping pin; unused and already pulled down
//   GPIO19/20 I2C          disabled as inputs in e1003_park_pins_for_sleep()
static const gpio_num_t kSleepLowPins[] = {
    GPIO_NUM_11, GPIO_NUM_21, GPIO_NUM_38, GPIO_NUM_39, GPIO_NUM_48,
};
static const size_t kSleepLowCount = sizeof(kSleepLowPins) / sizeof(kSleepLowPins[0]);

// Drive a pin LOW and latch it so the level survives deep sleep.
// The internal pull-up is switched off first: a hold latches the pull-up too,
// and a pull-up fighting a LOW output leaks about 70 uA per pin all night.
static void hold_low(gpio_num_t pin) {
  gpio_hold_dis(pin);
  pinMode(pin, OUTPUT);
  gpio_set_pull_mode(pin, GPIO_FLOATING);
  digitalWrite(pin, LOW);
  gpio_hold_en(pin);
}

void e1003_release_sleep_holds(void) {
  // A held pin ignores every write until the hold is released, and the hold
  // survives the wake-up reset. The display driver power-cycles GPIO11/21 in
  // initIT8951(), so the holds must be released before it runs.
  for (size_t i = 0; i < kSleepLowCount; i++) {
    gpio_hold_dis(kSleepLowPins[i]);
  }
  gpio_deep_sleep_hold_dis();

  // Keep the touch controller in reset and the SD slot off while awake too;
  // nothing on this board uses them, and a scanning GT911 costs a few mA.
  pinMode(GPIO_NUM_48, OUTPUT);
  digitalWrite(GPIO_NUM_48, LOW);
  pinMode(GPIO_NUM_39, OUTPUT);
  digitalWrite(GPIO_NUM_39, LOW);
}

void e1003_park_pins_for_sleep(void) {
  Log_info("E1003: parking peripheral enables for deep sleep");

  // Release the I2C bus. The GT911 is held in reset, so nothing needs it.
  gpio_set_direction(GPIO_NUM_19, GPIO_MODE_DISABLE);
  gpio_set_direction(GPIO_NUM_20, GPIO_MODE_DISABLE);

  for (size_t i = 0; i < kSleepLowCount; i++) {
    hold_low(kSleepLowPins[i]);
  }

  // gpio_hold_en() alone is enough for RTC pins (GPIO0-21). GPIO38/39/48 are
  // digital pins and also need this call to keep their hold through deep sleep.
  gpio_deep_sleep_hold_en();
}

void e1003_enable_button_wakeup(int interrupt_pin) {
  const gpio_num_t pin = (gpio_num_t)interrupt_pin;

#ifdef E1003_SLEEP_USE_EXT0
  // Stock behaviour, kept as a build-flag fallback.
  esp_sleep_enable_ext0_wakeup(pin, 0);
#else
  // Wake on the button going LOW using ext1. This is what both projects that
  // verified the hold fix on this board use alongside gpio_deep_sleep_hold_en().
  // The RTC peripheral domain stays on so the RTC pull-up is active in sleep;
  // the board also has an external pull-up on the button line.
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
  rtc_gpio_pullup_en(pin);
  rtc_gpio_pulldown_dis(pin);
  esp_err_t err = esp_sleep_enable_ext1_wakeup(1ULL << pin, ESP_EXT1_WAKEUP_ANY_LOW);
  if (err != ESP_OK) {
    Log_error("E1003: ext1 wakeup failed (%d), using ext0", (int)err);
    esp_sleep_enable_ext0_wakeup(pin, 0);
  }
#endif
}

#endif // BOARD_SEEED_RETERMINAL_E1003
