#pragma once
#include <stdint.h>
#include <stdbool.h>

// Initialize gen2 hardware: I2C, SPI, expanders, EPD.
// Call once from display_init() when BOARD_TRMNL_GEN2 is defined.
void gen2_display_init(void);

// Decode an already-downloaded PNG or BMP buffer and display it on the
// 31.5" ACeP panel via the 8-IC E2803 driver chain.
// Returns true if the image was decoded and sent successfully.
bool gen2_display_image(const uint8_t *buf, uint32_t len);

// Download an image from url, decode, and display it.
// Requires WiFi to already be connected.
bool gen2_display_image_url(const char *url);

// Battery reading: returns state-of-charge 0-100, or -1 on failure.
int  gen2_battery_soc(void);
// Battery voltage in mV, or 0 on failure.
uint16_t gen2_battery_voltage_mv(void);
// true if external power (VBUS) is present.
bool gen2_battery_power_good(void);
