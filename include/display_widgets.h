#pragma once

#include <Arduino.h>
#include <config.h> // UWORD

// Fills a pill (a rectangle whose left and right ends are half circles) in the image buffer
void display_fill_pill(int x, int y, int w, int h, uint8_t color);

// The battery icon (TRMNL X), DISPLAY_BATTERY_ICON_W x DISPLAY_BATTERY_ICON_H, drawn in the image
// buffer with its top left at (x, y): charging while on USB power, otherwise the charge level from
// the fuel gauge's last reading (battery().gaugeInit()), or a cross when there is none
#define DISPLAY_BATTERY_ICON_W      124
#define DISPLAY_BATTERY_ICON_H      74
#define DISPLAY_BATTERY_ICON_MARGIN 40 // from the top and right edges, where the screens put it
void display_draw_battery_icon(int x, int y);

/**
 * @brief Function to draw multi-line text onto the display
 * @param x_start X coordinate to start drawing
 * @param y_start Y coordinate to start drawing
 * @param message Text message to draw
 * @param max_width Maximum width in pixels for each line
 * @param font_width Width of a single character in pixels
 * @param color_fg Foreground color
 * @param color_bg Background color
 * @param font Font to use
 * @param is_center_aligned If true, center the text; if false, left-align
 * @return none
 */
void Paint_DrawMultilineText(UWORD x_start, UWORD y_start, const char *message, uint16_t max_width, UWORD color_fg,
                             UWORD color_bg, const void *font);
