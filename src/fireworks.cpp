#include "fireworks.h"

// TRMNL X only, like the menu that shows it
#ifdef BOARD_TRMNL_X

#include <Arduino.h>

#include "FastEPD.h"

extern FASTEPD bbep; // display.cpp

// Fireworks, white on black: the screen is wiped to black, then three rockets arc up from the
// bottom, one after another, and each bursts into a ring of arcs. Each frame is a partial update that
// adds the stretch of every line drawn since the last one, at the positions for the time it is now,
// so a slow frame draws more rather than slowing the animation. The updates only add white.
#define FIREWORKS_LAUNCH_STAGGER 800  // ms between the rockets' launches
#define FIREWORKS_RISE_MS        1400 // from the bottom to the burst
#define FIREWORKS_BURST_MS       2000 // the burst's arcs, from its center to their ends
#define FIREWORKS_TOTAL_MS       (2 * FIREWORKS_LAUNCH_STAGGER + FIREWORKS_RISE_MS + FIREWORKS_BURST_MS) // 5 s
#define FIREWORKS_TRAIL_RADIUS   5 // half the line widths
#define FIREWORKS_ARC_RADIUS     2
#define FIREWORKS_ARCS           32
#define FIREWORKS_ARC_LENGTH     360 // px from the burst
#define FIREWORKS_ARC_DROOP      150 // px the full-length arcs fall by their ends
#define FIREWORKS_ARC_LIFT       0.45f // extra speed of the upward arcs (straight up), so they droop less
#define FIREWORKS_SUBSTEPS       4   // chords per line per frame, to follow the curves

typedef struct {
  float launch_x; // fractions of the screen width and height; it launches from the bottom edge
  float burst_x, burst_y;
} firework_t;

// Launched left, right, center
static const firework_t fireworks[] = {
    {0.10f, 0.28f, 0.30f},
    {0.90f, 0.72f, 0.25f},
    {0.20f, 0.50f, 0.43f},
};
#define FIREWORKS_COUNT (int)(sizeof(fireworks) / sizeof(fireworks[0]))

// A line FIREWORKS_*_RADIUS * 2 thick, with round ends: discs along it
static void fireworks_thick_line(float x0, float y0, float x1, float y1, int radius) {
  const float dx = x1 - x0, dy = y1 - y0;
  const int steps = (int)(max(fabsf(dx), fabsf(dy)) * 2 / radius) + 1;
  for (int i = 0; i <= steps; i++) {
    bbep.fillCircle((int)(x0 + dx * i / steps), (int)(y0 + dy * i / steps), radius, BBEP_WHITE);
  }
}

// Where a firework's rocket is, `u` (0..1) of the way up: steady sideways, slowing as it rises
static void firework_rocket_at(const firework_t &fw, float u, float *x, float *y) {
  const float x0 = fw.launch_x * bbep.width(), y0 = bbep.height() + FIREWORKS_TRAIL_RADIUS;
  const float bx = fw.burst_x * bbep.width(), by = fw.burst_y * bbep.height();
  *x = x0 + (bx - x0) * u;
  *y = y0 + (by - y0) * (1 - (1 - u) * (1 - u));
}

// Arc lengths, as fractions of FIREWORKS_ARC_LENGTH, for perspective: the short ones are going
// towards or away from the viewer. Each firework starts at a different one.
static const float fireworks_arc_scales[] = {1.0f, 0.55f, 0.85f, 0.3f, 0.95f, 0.65f, 0.4f, 0.8f};
#define FIREWORKS_ARC_SCALES (int)(sizeof(fireworks_arc_scales) / sizeof(fireworks_arc_scales[0]))

// Where one of firework `index`'s arcs ends, `s` (0..1) through the burst: fast out, then falling
static void firework_arc_at(int index, int arc, float s, float *x, float *y) {
  const firework_t &fw = fireworks[index];
  const float e = 1 - (1 - s) * (1 - s) * (1 - s);
  const float angle = 2 * (float)M_PI * arc / FIREWORKS_ARCS;
  const float scale = fireworks_arc_scales[(arc + index * 3) % FIREWORKS_ARC_SCALES];
  // The upward ones are faster (sinf is negative up)
  const float lift = 1 + FIREWORKS_ARC_LIFT * max(0.0f, -sinf(angle));
  const float length = FIREWORKS_ARC_LENGTH * scale * lift;
  *x = fw.burst_x * bbep.width() + cosf(angle) * length * e;
  *y = fw.burst_y * bbep.height() + sinf(angle) * length * e + FIREWORKS_ARC_DROOP * scale * e * e;
}

static float fireworks_clamp01(float v) { return v < 0 ? 0 : v > 1 ? 1 : v; }

// Draws a firework's lines from `from` to `to` ms into the animation
static void firework_draw(int index, unsigned long from, unsigned long to) {
  const firework_t &fw = fireworks[index];
  const long launch = (long)index * FIREWORKS_LAUNCH_STAGGER;
  const float u0 = fireworks_clamp01((float)((long)from - launch) / FIREWORKS_RISE_MS);
  const float u1 = fireworks_clamp01((float)((long)to - launch) / FIREWORKS_RISE_MS);
  for (int i = 0; u1 > u0 && i < FIREWORKS_SUBSTEPS; i++) {
    float xa, ya, xb, yb;
    firework_rocket_at(fw, u0 + (u1 - u0) * i / FIREWORKS_SUBSTEPS, &xa, &ya);
    firework_rocket_at(fw, u0 + (u1 - u0) * (i + 1) / FIREWORKS_SUBSTEPS, &xb, &yb);
    fireworks_thick_line(xa, ya, xb, yb, FIREWORKS_TRAIL_RADIUS);
  }

  const long burst = launch + FIREWORKS_RISE_MS;
  const float s0 = fireworks_clamp01((float)((long)from - burst) / FIREWORKS_BURST_MS);
  const float s1 = fireworks_clamp01((float)((long)to - burst) / FIREWORKS_BURST_MS);
  if (s1 <= s0) return;
  for (int arc = 0; arc < FIREWORKS_ARCS; arc++) {
    for (int i = 0; i < FIREWORKS_SUBSTEPS; i++) {
      float xa, ya, xb, yb;
      firework_arc_at(index, arc, s0 + (s1 - s0) * i / FIREWORKS_SUBSTEPS, &xa, &ya);
      firework_arc_at(index, arc, s0 + (s1 - s0) * (i + 1) / FIREWORKS_SUBSTEPS, &xb, &yb);
      fireworks_thick_line(xa, ya, xb, yb, FIREWORKS_ARC_RADIUS);
    }
  }
}

void fireworks_show(void) {
  // Driven fully to black, so no trace of what was there shows through the fireworks
  bbep.fillScreen(BBEP_BLACK);
  bbep.fullUpdate(CLEAR_BLACK, true);
  const unsigned long start = millis();
  unsigned long drawn = 0;
  while (drawn < FIREWORKS_TOTAL_MS) {
    const unsigned long now = min(millis() - start, (unsigned long)FIREWORKS_TOTAL_MS);
    for (int i = 0; i < FIREWORKS_COUNT; i++) {
      firework_draw(i, drawn, now);
    }
    bbep.partialUpdate(true);
    drawn = now;
  }
}

void fireworks_wipe(void) {
  bbep.fillScreen(BBEP_WHITE);
  bbep.fullUpdate(CLEAR_FAST, false);
}

#endif // BOARD_TRMNL_X
