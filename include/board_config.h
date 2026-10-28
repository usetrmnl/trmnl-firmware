#pragma once

#include <config.h>

// This board's pins and panel, from the supported device list; NULL until hw_config_init()
extern TRMNL_DEVICE *pDevice;

// Finds DEVICE_MODEL in the supported device list and points pDevice at it
void hw_config_init(void);
