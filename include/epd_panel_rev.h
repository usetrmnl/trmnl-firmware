#pragma once

#include <Arduino.h>

// Reads the panel ID (REV command) over bit-banged SPI; 0 when unknown or not readable (bb_epaper boards only)
uint32_t get_panel_rev(void);

/**
 * @brief Panel ID (EPD revision) as 8 hex digits for the Panel-Rev request header
 * @return hex string, or empty if the ID is unknown/not read
 */
String display_panel_rev_string(void);
