#pragma once

typedef enum {
  TOUCHBAR_LEFT,
  TOUCHBAR_MIDDLE,
  TOUCHBAR_RIGHT,
} touchbar_side_t;

// Draws a black indicator in a white border above the touched region of the touch bar (TRMNL X):
// an arrow pointing left or right at the sides, a pill in the middle
void display_draw_touchbar_indicator(touchbar_side_t side);
