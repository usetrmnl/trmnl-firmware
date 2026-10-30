#include <battery.h>
#include <display_widgets.h>
#include <epd_driver.h>
#include <math.h>
#include <power.h>
#include <trmnl_log.h>

#include "battery_hollow.h"
#include "battery_small.h"

void display_fill_pill(int x, int y, int w, int h, uint8_t color) {
  float r = h / 2.0f;
  for (int row = 0; row < h; row++) {
    float dy = row + 0.5f - r;
    int inset = (int)lroundf(r - sqrtf(r * r - dy * dy));
    if (w - 2 * inset > 0) {
      bbep.fillRect(x + inset, y + row, w - 2 * inset, 1, color);
    }
  }
}

void display_draw_battery_icon(int x, int y) {
#ifdef BOARD_TRMNL_X
    // The charging icon while on USB power (whether actually charging or not)
  if (power().usbStatus() == UsbStatus::CONNECTED) {
    Log_info("Displaying 'battery is charging' icon");
    bbep.loadG5Image(battery_small, x, y, BBEP_WHITE, BBEP_BLACK);
    return;
  }
  bbep.loadG5Image(battery_hollow, x, y, BBEP_WHITE, BBEP_BLACK);
  Log_info("Displaying 'battery charge level' icon");
  int batt_percent = battery().readSoc(); // -1 if no battery or the gauge reading failed
  if (batt_percent >= 0) {
    if (batt_percent >= 97) batt_percent = 100; // can sometimes report 98% when full
        // Draw a black rectangle to represent the battery charge level
    bbep.fillRect(x + 10, y + 18, (97 * batt_percent) / 100, 39, BBEP_BLACK);
  } else { // a cross over the empty battery: each diagonal a band 6 px tall, column by column
    const int left = x + 10, top = y + 18, w = 97, h = 39, half = 3;
    for (int i = 0; i < w; i++) {
      int down = top + i * h / w;     // top left to bottom right
      int up = top + h - i * h / w;   // bottom left to top right
      for (int c : {down, up}) {
        int y0 = max(c - half, top), y1 = min(c + half, top + h);
        bbep.fillRect(left + i, y0, 1, y1 - y0, BBEP_BLACK);
      }
    }
  }
#endif // BOARD_TRMNL_X
}

/**
 * @brief Function to draw multi-line text onto the display
 * @param x_start X coordinate to start drawing
 * @param y_start Y coordinate to start drawing
 * @param message Text message to draw
 * @param max_width Maximum width in pixels for each line
 * @param color_fg Foreground color
 * @param color_bg Background color
 * @param font Font to use
 * @return none
 */
void Paint_DrawMultilineText(UWORD x_start, UWORD y_start, const char *message, uint16_t max_width, UWORD color_fg,
                             UWORD color_bg, const void *font) {
  char szTemp[80]; // up to 79 characters per line: getStringBox() copies the string to an 80-byte buffer
  char c, *d, *s = (char *)message;
  BB_RECT rect;
  bool bDone = false;
  int iWidthLimit = (max_width * 3) / 4; // don't let the text go all the way to the edges

  bbep.setFont(font);
  bbep.setTextColor(color_fg, color_bg);
  d = szTemp;
  bbep.setCursor(x_start, y_start); // start on the requested col/row
  while (!bDone) {
    c = *s++;
    *d++ = c;
    d[0] = 0; // test the string length
    bbep.getStringBox(szTemp, &rect);
    if (c == 0 || c == '\n' || (c == ' ' && rect.w >= iWidthLimit) || d == &szTemp[sizeof(szTemp) - 1]) {
      if (c == 0 || s[0] == 0) bDone = true;
      if (c == ' ' || c == '\n') {
        d[-1] = 0; // don't print the space/newline
        bbep.getStringBox(szTemp, &rect); // and don't center it either
      }
            // Display the current partial string
      bbep.setCursor(x_start + (max_width - rect.w) / 2, -1);
      bbep.println(szTemp);
      d = szTemp;
    }
  }
} /* Paint_DrawMultilineText() */
