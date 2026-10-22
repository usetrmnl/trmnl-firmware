#include <display.h>
#include <math.h>
#include <trmnl_log.h>

#ifdef BOARD_TRMNL_X
#include <FastEPD.h>

extern FASTEPD bbep; // display.cpp
#endif // BOARD_TRMNL_X

#ifdef PARALLEL_EPD
// partialUpdate() only sends hardware refresh commands where the software
// previousBuffer/currentBuffer actually differ - it has no idea the physical
// panel still shows a stale full-color/grayscale image underneath (e-paper
// keeps its last image through deep sleep; fillScreen() below only touches
// the software buffer, never the panel). The very first indicator drawn each
// boot gets a real diff for free from the mode-switch block below, so its
// region gets correctly cleared - but a second (or third) indicator, drawn
// moments later in the same boot with prev_mode already BB_MODE_1BPP, would
// otherwise be judged "unchanged" against that same white software baseline
// and never get a hardware command for its own region at all, leaving the
// old image showing through. So every side gets this same forced-diff
// treatment on its own first draw, not just whichever side happens to draw
// first.
static bool touchbar_side_seeded[3] = {false, false, false};
#endif // PARALLEL_EPD

#ifdef BOARD_TRMNL_X
#define TOUCHBAR_INDICATOR_MARGIN_BOTTOM 52 // from the bottom of the screen to the indicators' bottom edge
#define TOUCHBAR_INDICATOR_BORDER        6         // white border around the black indicator
// The middle indicator: a pill
#define TOUCHBAR_PILL_H                  48
#define TOUCHBAR_PILL_W                  (TOUCHBAR_PILL_H * 4)
// The left and right indicators: arrows pointing that way
#define TOUCHBAR_ARROW_W                 160      // tail to tip
#define TOUCHBAR_ARROW_HEAD_H            64  // the head's base
#define TOUCHBAR_ARROW_HEAD_W            56  // the head's base to the tip
#define TOUCHBAR_ARROW_SHAFT_H           24

static int touchbar_indicator_center_x(touchbar_side_t side) {
  int w = bbep.width();
  switch (side) {
  case TOUCHBAR_LEFT:
    return w / 6;
  case TOUCHBAR_MIDDLE:
    return w / 2;
  case TOUCHBAR_RIGHT:
    return w * 5 / 6;
  default:
    return -1;
  }
}

// The span [*x0, *x1) of a right-pointing arrow, from its center, on the row dy from its center line
static bool touchbar_arrow_span(float dy, int *x0, int *x1) {
  const float head_half = TOUCHBAR_ARROW_HEAD_H / 2.0f;
  const float ady = fabsf(dy);
  if (ady >= head_half) return false;
  const int base = TOUCHBAR_ARROW_W / 2 - TOUCHBAR_ARROW_HEAD_W; // the head's base
  *x1 = base + (int)lroundf(TOUCHBAR_ARROW_HEAD_W * (1.0f - ady / head_half));
  *x0 = ady < TOUCHBAR_ARROW_SHAFT_H / 2.0f ? -TOUCHBAR_ARROW_W / 2 : base;
  return true;
}

// Fills an arrow centered on (cx, cy) pointing left (dir < 0) or right, grown by `grow` pixels all
// round: each row takes in the rows within `grow` of it, widened by as much as a circle would be.
static void touchbar_fill_arrow(int cx, int cy, int dir, int grow, uint8_t color) {
  const int h = TOUCHBAR_ARROW_HEAD_H;
  for (int row = -grow; row < h + grow; row++) {
    int lo = INT_MAX, hi = INT_MIN;
    for (int k = -grow; k <= grow; k++) {
      int src = row + k;
      int x0, x1;
      if (src < 0 || src >= h || !touchbar_arrow_span(src + 0.5f - h / 2.0f, &x0, &x1)) continue;
      int e = (int)lroundf(sqrtf((float)(grow * grow - k * k)));
      lo = min(lo, x0 - e);
      hi = max(hi, x1 + e);
    }
    if (lo >= hi) continue;
    bbep.fillRect(dir < 0 ? cx - hi : cx + lo, cy - h / 2 + row, hi - lo, 1, color);
  }
}

// Draws a side's indicator centered on (cx, cy): black, in a white border. inverse swaps black and
// white, for the seed.
static void touchbar_draw_indicator(touchbar_side_t side, int cx, int cy, bool inverse) {
  const uint8_t fg = inverse ? BBEP_WHITE : BBEP_BLACK;
  const uint8_t bg = inverse ? BBEP_BLACK : BBEP_WHITE;
  const int b = TOUCHBAR_INDICATOR_BORDER;
  if (side == TOUCHBAR_MIDDLE) {
    int rx = cx - TOUCHBAR_PILL_W / 2;
    int ry = cy - TOUCHBAR_PILL_H / 2;
    display_fill_pill(rx - b, ry - b, TOUCHBAR_PILL_W + 2 * b, TOUCHBAR_PILL_H + 2 * b, bg);
    display_fill_pill(rx, ry, TOUCHBAR_PILL_W, TOUCHBAR_PILL_H, fg);
  } else {
    int dir = side == TOUCHBAR_LEFT ? -1 : 1;
    touchbar_fill_arrow(cx, cy, dir, b, bg);
    touchbar_fill_arrow(cx, cy, dir, 0, fg);
  }
}
#endif // BOARD_TRMNL_X

// Black, in a white border, above the touched region: an arrow pointing left or right at the sides,
// a pill in the middle. Only TRMNL X has a touchbar.
void display_draw_touchbar_indicator(touchbar_side_t side) {
#ifdef BOARD_TRMNL_X
  Log_info("Drawing touchbar indicator %d\n", (int)side);
  int cx = touchbar_indicator_center_x(side);
  if (cx < 0) return;
  int cy = bbep.height() - TOUCHBAR_INDICATOR_MARGIN_BOTTOM - TOUCHBAR_PILL_H / 2;
  if (bbep.getPreviousMode() == BB_MODE_NONE) {
    bbep.setMode(BB_MODE_1BPP);
    bbep.fillScreen(BBEP_WHITE);
    int sz = bbep.width() * bbep.height() / 8; // 1-bit per pixel
    memcpy(bbep.previousBuffer(), bbep.currentBuffer(), sz);
    bbep.setPreviousMode(BB_MODE_1BPP);
  }
  if (!touchbar_side_seeded[side]) {
    touchbar_side_seeded[side] = true;
        // Seed with the exact per-pixel inverse of what we're about to draw so the real draw
        // right after always produces a full diff for partialUpdate() to push.
    touchbar_draw_indicator(side, cx, cy, true);
    int sz = bbep.width() * bbep.height() / 8;
    memcpy(bbep.previousBuffer(), bbep.currentBuffer(), sz);
  }
  touchbar_draw_indicator(side, cx, cy, false);
  bbep.partialUpdate(true); // avoid the costly power-cycle each call
#endif // BOARD_TRMNL_X
}
