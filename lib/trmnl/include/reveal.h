#pragma once

#include <ArduinoJson.h>
#include <stddef.h>
#include <stdint.h>

// Framework elements of a screen that the TRMNL X fills in one by one after it shows the screen.
// The server lists them in the PNG comment and sends the timing in the /api/display answer.

#define REVEAL_MAX_REGIONS    48
#define REVEAL_PROGRESS_WHOLE 1000

enum RevealEffect : uint8_t {
  REVEAL_APPEAR,
  REVEAL_FADE,
  REVEAL_WIPE_UP,
  REVEAL_WIPE_RIGHT,
  REVEAL_WIPE_DOWN,
  REVEAL_WIPE_LEFT
};

enum RevealRefresh : uint8_t { REVEAL_REFRESH_SMOOTH, REVEAL_REFRESH_PER_REGION };

struct RevealRegion {
  int16_t x, y, w, h;
  RevealEffect effect;
};

struct RevealTiming {
  bool enabled; // false when the answer carried no reveal
  RevealRefresh refresh;
  uint16_t stagger_ms;
  RevealEffect appear_effect; // REVEAL_APPEAR or REVEAL_FADE, for the regions that do not wipe
  uint16_t appear_ms;
  uint16_t wipe_ms;
  uint16_t max_total_ms;
};

RevealTiming reveal_parse_timing(JsonVariantConst reveal);

// The reveal records of the PNG's Comment text chunk, at most max_regions of them
int reveal_parse_png(const uint8_t *png, size_t size, RevealRegion *regions, int max_regions);

RevealEffect reveal_effect(const RevealRegion &region, const RevealTiming &timing);
uint32_t reveal_duration_ms(const RevealRegion &region, const RevealTiming &timing);

// Fills each region's start; a long list staggers faster so the whole stays within max_total_ms.
// Returns when the last region is whole.
uint32_t reveal_schedule(const RevealRegion *regions, int count, const RevealTiming &timing, uint32_t *start_ms);

// 0 before start_ms, REVEAL_PROGRESS_WHOLE once the region is whole
uint16_t reveal_progress(uint32_t start_ms, uint32_t duration_ms, uint32_t elapsed_ms);

bool reveal_pixel_shown(const RevealRegion &region, RevealEffect effect, uint16_t progress, int x, int y);

// A portrait PNG is drawn turned a quarter (png_draw); turn its region the same way
void reveal_turn_region(RevealRegion *region, int panel_height);
