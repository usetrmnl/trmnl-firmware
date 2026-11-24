#include "reveal_x.h"

#ifdef BOARD_TRMNL_X

#include <Arduino.h>
#include <epd_driver.h>
#include <esp_task_wdt.h>
#include <globals.h>
#include <reveal.h>
#include <string.h>
#include <trmnl_log.h>

#include "display_internal.h"

static const uint8_t BAYER_4X4[16] = {0, 8, 2, 10, 12, 4, 14, 6, 3, 11, 1, 9, 15, 7, 13, 5};
static const uint8_t GRAY_WHITE = 0x0f;

static RevealRegion regions[REVEAL_MAX_REGIONS];
static uint32_t start_ms[REVEAL_MAX_REGIONS];
static uint16_t painted_progress[REVEAL_MAX_REGIONS];

static int gray_pitch() { return bbep.width() / 2; }
static int one_bit_pitch() { return (bbep.width() + 7) / 8; }

static uint8_t gray_at(const uint8_t *buffer, int x, int y) {
  const uint8_t pair = buffer[y * gray_pitch() + x / 2];
  return (x & 1) ? (pair & 0x0f) : (pair >> 4);
}

static void set_gray(uint8_t *buffer, int x, int y, uint8_t gray) {
  uint8_t *pair = &buffer[y * gray_pitch() + x / 2];
  *pair = (x & 1) ? ((*pair & 0xf0) | gray) : ((*pair & 0x0f) | (gray << 4));
}

static bool one_bit_white_at(const uint8_t *buffer, int x, int y) {
  return buffer[y * one_bit_pitch() + x / 8] & (0x80 >> (x & 7));
}

static void set_one_bit(uint8_t *buffer, int x, int y, bool white) {
  uint8_t *byte = &buffer[y * one_bit_pitch() + x / 8];
  const uint8_t mask = 0x80 >> (x & 7);
  *byte = white ? (*byte | mask) : (*byte & ~mask);
}

// The black and white pixel the 1-bit animation shows for the image pixel at x, y
static bool image_white_at(const uint8_t *image, bool image_is_gray, int x, int y) {
  if (!image_is_gray) return one_bit_white_at(image, x, y);
  return gray_at(image, x, y) > BAYER_4X4[(x & 3) + (y & 3) * 4];
}

static int load_regions(const uint8_t *png, int png_size) {
  int count = reveal_parse_png(png, png_size, regions, REVEAL_MAX_REGIONS);
  const int png_width = png_size > 20 ? (png[16] << 24) | (png[17] << 16) | (png[18] << 8) | png[19] : 0;
  const bool turned = png_width == bbep.height() && png_width != bbep.width(); // as png_draw decides
  int kept = 0;
  for (int i = 0; i < count; i++) {
    RevealRegion region = regions[i];
    if (turned) reveal_turn_region(&region, bbep.height());
    const int right = min((int)region.x + region.w, (int)bbep.width());
    const int bottom = min((int)region.y + region.h, (int)bbep.height());
    region.x = max((int)region.x, 0);
    region.y = max((int)region.y, 0);
    region.w = right - region.x;
    region.h = bottom - region.y;
    if (region.w > 0 && region.h > 0) regions[kept++] = region;
  }
  return kept;
}

// Paints the part of each region that is due; false when every region is whole
static bool paint_due_regions(int count, const RevealTiming &timing, uint32_t elapsed_ms, int *top, int *bottom,
                              int *left, int *right, void (*paint)(int x, int y)) {
  bool unfinished = false;
  for (int i = 0; i < count; i++) {
    const RevealRegion &region = regions[i];
    const RevealEffect effect = reveal_effect(region, timing);
    const uint16_t progress = reveal_progress(start_ms[i], reveal_duration_ms(region, timing), elapsed_ms);
    if (progress < REVEAL_PROGRESS_WHOLE) unfinished = true;
    if (progress == painted_progress[i]) continue;
    painted_progress[i] = progress;
    for (int y = region.y; y < region.y + region.h; y++) {
      for (int x = region.x; x < region.x + region.w; x++) {
        if (reveal_pixel_shown(region, effect, progress, x, y)) paint(x, y);
      }
    }
    *top = min(*top, (int)region.y);
    *bottom = max(*bottom, region.y + region.h - 1);
    *left = min(*left, (int)region.x);
    *right = max(*right, region.x + region.w - 1);
  }
  return unfinished;
}

static const uint8_t *image_copy;
static bool image_copy_is_gray;

static void paint_one_bit(int x, int y) {
  set_one_bit(bbep.currentBuffer(), x, y, image_white_at(image_copy, image_copy_is_gray, x, y));
}

static void paint_gray(int x, int y) { set_gray(bbep.currentBuffer(), x, y, gray_at(image_copy, x, y)); }

static void play_one_bit(int count, const RevealTiming &timing, uint32_t total_ms) {
  uint8_t *frame = bbep.currentBuffer();
  bbep.setMode(BB_MODE_1BPP);
  for (int y = 0; y < bbep.height(); y++) {
    for (int x = 0; x < bbep.width(); x++)
      set_one_bit(frame, x, y, image_white_at(image_copy, image_copy_is_gray, x, y));
  }
  for (int i = 0; i < count; i++) {
    for (int y = regions[i].y; y < regions[i].y + regions[i].h; y++) {
      for (int x = regions[i].x; x < regions[i].x + regions[i].w; x++)
        set_one_bit(frame, x, y, true);
    }
  }
  bbep.fullUpdate(CLEAR_FAST, true);
  const uint32_t started = millis();
  bool unfinished = true;
  while (unfinished) {
    int top = bbep.height(), bottom = -1, left = bbep.width(), right = -1;
    unfinished = paint_due_regions(count, timing, millis() - started, &top, &bottom, &left, &right, paint_one_bit);
    if (bottom >= top) bbep.partialUpdate(true, top, bottom); // ~200 ms whatever the rows
    esp_task_wdt_reset();
    if (millis() - started > total_ms + 5000) break; // never hold the wake for a broken timeline
  }
}

// Each frame refreshes the box around what changed, so grays stay but that box flashes.
static void play_gray_per_region(int count, const RevealTiming &timing, uint32_t total_ms) {
  for (int i = 0; i < count; i++) {
    for (int y = regions[i].y; y < regions[i].y + regions[i].h; y++) {
      for (int x = regions[i].x; x < regions[i].x + regions[i].w; x++)
        set_gray(bbep.currentBuffer(), x, y, GRAY_WHITE);
    }
  }
  display_use_gray_matrix();
  bbep.fullUpdate(CLEAR_FAST, true);
  const uint32_t started = millis();
  bool unfinished = true;
  while (unfinished) {
    int top = bbep.height(), bottom = -1, left = bbep.width(), right = -1;
    unfinished = paint_due_regions(count, timing, millis() - started, &top, &bottom, &left, &right, paint_gray);
    if (bottom >= top) {
      // FastEPD's 4-bpp column clip on the X updates the mirrored band, as the clock found
      BB_RECT rect = {bbep.width() - right - 1, top, right - left + 1, bottom - top + 1};
      esp_task_wdt_reset();
      bbep.fullUpdate(CLEAR_FAST, true, &rect);
    }
    esp_task_wdt_reset();
    if (millis() - started > total_ms + 5000) break;
  }
}

RevealPlayOutcome reveal_x_play(const uint8_t *png, int png_size) {
  if (!apiDisplayResult.received || !apiDisplayResult.response.reveal.enabled) return REVEAL_NOT_PLAYED;
  const RevealTiming &timing = apiDisplayResult.response.reveal;
  const int mode = bbep.getMode();
  if (mode != BB_MODE_4BPP && mode != BB_MODE_1BPP) return REVEAL_NOT_PLAYED;
  const int count = load_regions(png, png_size);
  if (count == 0) return REVEAL_NOT_PLAYED;

  image_copy_is_gray = mode == BB_MODE_4BPP;
  const size_t image_size = (image_copy_is_gray ? gray_pitch() : one_bit_pitch()) * bbep.height();
  uint8_t *copy = (uint8_t *)ps_malloc(image_size);
  if (!copy) {
    Log_error("reveal: no memory for a copy of the image");
    return REVEAL_NOT_PLAYED;
  }
  memcpy(copy, bbep.currentBuffer(), image_size);
  image_copy = copy;
  memset(painted_progress, 0, sizeof(painted_progress));
  const uint32_t total_ms = reveal_schedule(regions, count, timing, start_ms);
  Log_info("reveal: %d regions over %u ms, %s", count, (unsigned)total_ms,
           timing.refresh == REVEAL_REFRESH_PER_REGION ? "per region" : "smooth");

  RevealPlayOutcome outcome = REVEAL_PANEL_SHOWS_IMAGE;
  if (image_copy_is_gray && timing.refresh == REVEAL_REFRESH_PER_REGION) {
    play_gray_per_region(count, timing, total_ms);
  } else {
    play_one_bit(count, timing, total_ms);
    if (image_copy_is_gray) { // back to the gray image, for the refresh that brings back its grays
      bbep.setMode(BB_MODE_4BPP);
      memcpy(bbep.currentBuffer(), copy, image_size);
      outcome = REVEAL_NEEDS_GRAY_REFRESH;
    }
  }
  if (outcome == REVEAL_PANEL_SHOWS_IMAGE) bbep.einkPower(0);
  free(copy);
  image_copy = nullptr;
  return outcome;
}

#else

RevealPlayOutcome reveal_x_play(const uint8_t *, int) { return REVEAL_NOT_PLAYED; }

#endif // BOARD_TRMNL_X
