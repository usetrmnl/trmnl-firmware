#include "zectrix.h"

#ifdef BOARD_ZECTRIX

#include <Arduino.h>
#include <DEV_Config.h>
#include <driver/gpio.h>
#include <driver/rtc_io.h>

namespace {

  void configure_output(gpio_num_t pin, uint32_t level) {
    rtc_gpio_hold_dis(pin);
    rtc_gpio_deinit(pin);
    pinMode(static_cast<uint8_t>(pin), OUTPUT);
    digitalWrite(static_cast<uint8_t>(pin), level);
  }

  void hold_rtc_output(gpio_num_t pin, uint32_t level) {
    rtc_gpio_hold_dis(pin);
    rtc_gpio_init(pin);
    rtc_gpio_set_direction(pin, RTC_GPIO_MODE_OUTPUT_ONLY);
    rtc_gpio_set_level(pin, level);
    rtc_gpio_hold_en(pin);
  }

} // namespace

void zectrix_power_init() {
  gpio_deep_sleep_hold_dis();

  configure_output(static_cast<gpio_num_t>(VBAT_HOLD_PIN), HIGH);
  configure_output(static_cast<gpio_num_t>(EPD_POWER_PIN), HIGH);
  delay(10);
}

void zectrix_prepare_deep_sleep() {
  // The panel is bistable, so its power rail can be disabled after refresh.
  hold_rtc_output(static_cast<gpio_num_t>(EPD_POWER_PIN), LOW);

  // GPIO17 controls the board's battery power latch. Letting it fall LOW cuts
  // power to the ESP32-S3 and prevents timer/button deep-sleep wake-up.
  hold_rtc_output(static_cast<gpio_num_t>(VBAT_HOLD_PIN), HIGH);
  gpio_deep_sleep_hold_en();
}

#endif
