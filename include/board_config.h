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
