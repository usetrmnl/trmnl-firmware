#pragma once

#include <config.h>

// This board's pins and panel, from the supported device list; NULL until hw_config_init()
extern TRMNL_DEVICE *pDevice;

typedef struct dp_tag {
  uint32_t OneBit, TwoBit; // profiles for 1 and 2-bit modes
} DISPLAY_PROFILE;

// 1-bit and 2-bit panel types for each panel set (pDevice->panel_set) and temperature profile (bb_epaper boards only)
extern const DISPLAY_PROFILE dpList[12][3];

// Finds DEVICE_MODEL in the supported device list and points pDevice at it
void hw_config_init(void);

// True when this board lists extra wake buttons (button_a_pin/button_b_pin)
bool has_extra_wake_buttons(void);

// Bit mask of every pin that wakes this board: interrupt_pin plus any extra buttons
uint64_t wake_button_mask(void);
