/**
 * @file spectra6.h
 * @brief Header file for Spectra6 display functions.
 *
 * Initializes and updates the Spectra6 display for Seeed ReTerminal E1002.
 */

#pragma once

/**
 * @brief Initializes the SPI interface for the Spectra6 display.
 * @return true if initialization was successful, false otherwise.
 */
bool spectra6_init_spi();

/**
 * @brief Updates the Spectra6 display.
 * @return true if the update was successful, false otherwise.
 */
bool spectra6_update();

/**
 * @brief Renders a 1-bpp (MSB-first) bitmap onto the Spectra6 display.
 * @param bitmap Pointer to the raw pixel rows (BMP header already skipped).
 * @return true if the framebuffer was allocated and rendering was successful, false otherwise.
 */
bool spectra6_render_1bpp_bitmap(const uint8_t *bitmap);