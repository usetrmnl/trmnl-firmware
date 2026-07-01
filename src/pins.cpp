#include <Arduino.h>
#include <pins.h>
#include <config.h>

void pins_init(void)
{
#ifdef BOARD_ESP32_M075_GDP075FW1_NO_BUTTON
    return;
#else
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << PIN_INTERRUPT);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);
#endif
}
