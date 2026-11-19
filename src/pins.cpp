#include <Arduino.h>
#include <board_config.h>
#include <config.h>
#if defined(BOARD_SEEED_RETERMINAL_E1001) || defined(BOARD_SEEED_RETERMINAL_E1002)
#include "driver/rtc_io.h"
#endif

void pins_init(void) {
  gpio_config_t io_conf = {};
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_INPUT;
#ifdef BOARD_TRMNL_X
  io_conf.pin_bit_mask = (1ULL << PIN_INTERRUPT);
#elif defined(BOARD_SEEED_RETERMINAL_E1001) || defined(BOARD_SEEED_RETERMINAL_E1002)
  rtc_gpio_deinit(GPIO_NUM_3);
  rtc_gpio_deinit(GPIO_NUM_4);
  rtc_gpio_deinit(GPIO_NUM_5);
  io_conf.pin_bit_mask = (1ULL << 3) | (1ULL << 4) | (1ULL << 5);
#else
  io_conf.pin_bit_mask = (1ULL << pDevice->interrupt_pin);
#endif
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
  gpio_config(&io_conf);
}
