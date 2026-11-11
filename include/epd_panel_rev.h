#pragma once

#include <Arduino.h>

// Reads the panel ID (REV command) over bit-banged SPI; 0 when unknown or not readable (bb_epaper boards only)
uint32_t get_panel_rev(void);

// Reads the VCOM_DC code (VDCS 0x82) the factory programmed into the OG panel's OTP, once per panel (cached).
// OG's UltraChip panel only. Call right after get_panel_rev(), before bb_epaper takes over the SPI pins.
void read_panel_vcom(uint32_t panel_id);

// The Panel-Vcom request header: the VCOM_DC code, "-1" when the OTP holds none, empty when not read
String display_panel_vcom_string(void);

/**
 * @brief Panel ID (EPD revision) as 8 hex digits for the Panel-Rev request header
 * @return hex string, or empty if the ID is unknown/not read
 */
String display_panel_rev_string(void);
