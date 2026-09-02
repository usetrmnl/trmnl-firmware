#include <client_draw/clock.h>

#ifdef PARALLEL_EPD

#include <Arduino.h>
#include <FastEPD.h>
#include <config.h>
#include <display.h>
#include <esp_sleep.h>
#include <esp_task_wdt.h>
#include <globals.h>
#include <math.h>
#include <client_draw/clock.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <trmnl_log.h>

#ifdef BOARD_TRMNL_X
#include "iqs323_task.h" // iqs323_task_i2c_lock/unlock around direct chip reads
#endif

#include "fonts/dseg_fonts.h"

extern FASTEPD bbep;

// Longest light sleep between touch polls; a touch exits after this delay at worst.
// Polls often enough that a touchbar tap feels responsive; the clock only redraws
// when the minute actually changes, so frequent wakes cost little.
#define CLIENT_DRAW_CLOCK_NAP_MS          1000
// Light sleep freezes the IQS323 task, so on wake we must sample the chip directly
// (updateInfoFlags) until the state is stable. The chip re-reports a held touch for
// seconds, so a few samples per wake are enough — and we stop as soon as the read
// is clean, so an untouched minute costs ~5 ms of wake time per check, not 150 ms.
#define CLIENT_DRAW_CLOCK_TOUCH_SAMPLES   5
#define CLIENT_DRAW_CLOCK_TOUCH_SAMPLE_MS 10

static ClientDrawClock s_spec;
static bool s_active = false;
static int s_refresh_tick = 0;
static uint32_t s_consumed_s = 0;
static int s_last_drawn_min = -1;
static int s_touched_zone = CLIENT_DRAW_CLOCK_TOUCH_NONE;

static uint8_t overlay_black() { return BBEP_BLACK; }

static uint8_t overlay_white() {
  switch (bbep.getMode()) {
  case BB_MODE_1BPP:
    return BBEP_WHITE;
  case BB_MODE_2BPP:
    return 0x03;
  default:
    return 0x0f;
  }
}

static void draw_thick_line(int x1, int y1, int x2, int y2, int thickness, uint8_t color) {
  if (thickness <= 1) {
    bbep.drawLine(x1, y1, x2, y2, color);
    return;
  }
  int dx = x2 - x1;
  int dy = y2 - y1;
  int steps = thickness / 2;
  if (abs(dx) >= abs(dy)) {
    for (int t = -steps; t <= steps; t++) {
      bbep.drawLine(x1, y1 + t, x2, y2 + t, color);
    }
  } else {
    for (int t = -steps; t <= steps; t++) {
      bbep.drawLine(x1 + t, y1, x2 + t, y2, color);
    }
  }
}

static void draw_hand(int cx, int cy, float angle_deg, int length, int thickness, uint8_t color) {
  const float rad = (angle_deg - 90.0f) * 0.01745329252f;
  const int x2 = cx + (int)(cosf(rad) * (float)length);
  const int y2 = cy + (int)(sinf(rad) * (float)length);
  draw_thick_line(cx, cy, x2, y2, thickness, color);
}

#define BB_FONT_MARKER       0xBBFF
#define BB_FONT_MARKER_SMALL 0xBBF2

static bool clock_measure_custom_font(const void *pFont, const char *text, int *out_w, int *out_h, int *out_miny) {
  const uint8_t *base = (const uint8_t *)pFont;
  const uint16_t marker = pgm_read_word(base);
  size_t glyph_size;
  uint8_t first;
  uint8_t last;

  if (marker == BB_FONT_MARKER) {
    glyph_size = 12;
    first = pgm_read_byte(base + 2);
    last = pgm_read_byte(base + 4);
  } else if (marker == BB_FONT_MARKER_SMALL) {
    glyph_size = 8;
    first = pgm_read_byte(base + 2);
    last = pgm_read_byte(base + 4);
  } else {
    return false;
  }

  int cx = 0;
  int min_y = 4000;
  int max_y = 0;

  for (const char *p = text; *p; p++) {
    uint8_t c = (uint8_t)*p;
    if (c < first || c > last) {
      continue;
    }
    c -= first;
    const size_t go = 12 + (size_t)c * glyph_size;

    if (marker == BB_FONT_MARKER) {
      cx += (int)pgm_read_word(base + go + 4);
      const int16_t yo = (int16_t)pgm_read_word(base + go + 10);
      const int16_t gh = (int16_t)pgm_read_word(base + go + 6);
      if (yo < min_y) {
        min_y = yo;
      }
      const int bottom = gh + yo;
      if (bottom > max_y) {
        max_y = bottom;
      }
    } else {
      cx += (int)pgm_read_byte(base + go + 3);
      const int8_t yo = (int8_t)pgm_read_byte(base + go + 6);
      const int8_t gh = (int8_t)pgm_read_byte(base + go + 4);
      if (yo < min_y) {
        min_y = yo;
      }
      const int bottom = gh + yo;
      if (bottom > max_y) {
        max_y = bottom;
      }
    }
  }

  if (cx <= 0 || max_y <= min_y) {
    return false;
  }
  *out_w = cx;
  *out_h = max_y - min_y + 1;
  *out_miny = min_y;
  return true;
}

static void clock_font_set_colors(uint8_t fg) {
  // G5 lines: set bits (0x80) are ink → iFG; cleared bits are background → iBG.
  // White field comes from fillRect; only paint ink in black.
  (void)fg;
  bbep.setTextColor(overlay_black(), BBEP_TRANSPARENT);
}

static void clock_font_draw_string(const ClientDrawClock &spec, const void *pFont, const char *text, int font_pt = 0) {
  int w = 0;
  int h = 0;
  int min_y = 0;

  if (!clock_measure_custom_font(pFont, text, &w, &h, &min_y)) {
    Log_info("client_draw_clock font: measure failed for '%s'", text);
    return;
  }

  const int x = spec.x + (spec.w - w) / 2;
  const int y = spec.y + (spec.h - h) / 2 - min_y;
  if (font_pt > 0) {
    Log_info("client_draw_clock font: '%s' %dpt tile %dx%d at %d,%d box %dx%d", text, font_pt, spec.w, spec.h, x, y, w, h);
  } else {
    Log_info("client_draw_clock font: '%s' tile %dx%d at %d,%d box %dx%d", text, spec.w, spec.h, x, y, w, h);
  }

  esp_task_wdt_reset();
  bbep.drawString(text, x, y);
  esp_task_wdt_reset();
}

static const void *clock_pick_dseg_font(const ClientDrawClock &spec, const char *text, int *font_pt) {
  // Use full tile width; keep a little vertical breathing room.
  const int max_w = spec.w;
  const int max_h = (spec.h * 92) / 100;

  for (int i = 0; i < DSEG_FONT_COUNT; i++) {
    const void *font = dseg_fonts[i];
    int w = 0;
    int h = 0;
    int min_y = 0;
    if (!clock_measure_custom_font(font, text, &w, &h, &min_y)) {
      continue;
    }
    if (w <= max_w && h <= max_h) {
      *font_pt = dseg_font_pt[i];
      return font;
    }
  }

  const int last = DSEG_FONT_COUNT - 1;
  int w = 0;
  int h = 0;
  int min_y = 0;
  clock_measure_custom_font(dseg_fonts[last], text, &w, &h, &min_y);
  Log_error("client_draw_clock: '%s' does not fit %dx%d; smallest font is %dx%d at %dpt", text, spec.w,
            spec.h, w, h, dseg_font_pt[last]);
  *font_pt = dseg_font_pt[last];
  return dseg_fonts[last];
}

static void draw_digital(const ClientDrawClock &spec, const struct tm &now, uint8_t fg, uint8_t bg) {
  bbep.fillRect(spec.x, spec.y, spec.w, spec.h, bg);

  char buf[6];
  snprintf(buf, sizeof(buf), "%02d:%02d", now.tm_hour, now.tm_min);

  int font_pt = 0;
  const void *font = clock_pick_dseg_font(spec, buf, &font_pt);
  bbep.setFont(font);
  clock_font_set_colors(fg);
  clock_font_draw_string(spec, font, buf, font_pt);
}

static void draw_analog(const ClientDrawClock &spec, const struct tm &now, uint8_t fg, uint8_t bg) {
  const int cx = spec.x + spec.w / 2;
  const int cy = spec.y + spec.h / 2;
  int r = spec.w < spec.h ? spec.w / 2 : spec.h / 2;
  r -= 4;
  if (r < 16) {
    return;
  }

  bbep.fillRect(spec.x, spec.y, spec.w, spec.h, bg);

  bbep.drawCircle(cx, cy, r, fg);
  bbep.drawCircle(cx, cy, r - 1, fg);

  for (int i = 0; i < 12; i++) {
    const float rad = (i * 30.0f - 90.0f) * 0.01745329252f;
    const int inner = (i % 3 == 0) ? (int)(r * 0.78f) : (int)(r * 0.88f);
    const int x1 = cx + (int)(cosf(rad) * (float)inner);
    const int y1 = cy + (int)(sinf(rad) * (float)inner);
    const int x2 = cx + (int)(cosf(rad) * (float)(r - 3));
    const int y2 = cy + (int)(sinf(rad) * (float)(r - 3));
    draw_thick_line(x1, y1, x2, y2, (i % 3 == 0) ? 4 : 2, fg);
  }

  // Minute-resolution hands only (no second hand).
  const float hour_ang = (now.tm_hour % 12) * 30.0f + now.tm_min * 0.5f;
  const float min_ang = now.tm_min * 6.0f;

  draw_hand(cx, cy, hour_ang, (int)(r * 0.50f), 6, fg);
  draw_hand(cx, cy, min_ang, (int)(r * 0.72f), 4, fg);
  bbep.fillCircle(cx, cy, r / 18 < 4 ? 4 : r / 18, fg);
}

// UTC + metadata tz offset (device local TZ is ignored).
static void client_draw_clock_localtime(const ClientDrawClock &spec, struct tm *out) {
  client_draw_clock_wall_time(&spec, time(nullptr), out);
}

// Bench: compare utc, utc+metadata tz (drawn time), and device localtime.
static void log_clock_time(const char *label, const ClientDrawClock &spec) {
  const time_t utc = time(nullptr);
  struct tm utc_tm;
  struct tm wall_tm;
  struct tm local_tm;
  gmtime_r(&utc, &utc_tm);
  client_draw_clock_wall_time(&spec, utc, &wall_tm);
  localtime_r(&utc, &local_tm);
  Log_info(
    "client_draw_clock %s: epoch %ld | utc %02d:%02d:%02d | wall(utc+tz=%ld) %02d:%02d:%02d | device_local %02d:%02d:%02d",
    label, (long)utc, utc_tm.tm_hour, utc_tm.tm_min, utc_tm.tm_sec, (long)spec.tz, wall_tm.tm_hour, wall_tm.tm_min,
    wall_tm.tm_sec, local_tm.tm_hour, local_tm.tm_min, local_tm.tm_sec);
}

static void draw_clock_now(const ClientDrawClock &spec) {
  struct tm now;
  client_draw_clock_localtime(spec, &now);
  const uint8_t fg = overlay_black();
  const uint8_t bg = overlay_white();
  if (spec.style == CLIENT_DRAW_CLOCK_ANALOG) {
    draw_analog(spec, now, fg, bg);
  } else {
    draw_digital(spec, now, fg, bg);
  }
}

// Read the touch channels straight from the chip (the IQS323 task is frozen in
// light sleep, so its cache is stale on wake). Returns a zone or ..._TOUCH_NONE.
static int read_touch_zone() {
#ifdef BOARD_TRMNL_X
  iqs323_task_i2c_lock();
  iqs323.updateInfoFlags(STOP);
  iqs323_task_i2c_unlock();
  if (iqs323.channel_touchState(IQS323_CH0)) {
    return CLIENT_DRAW_CLOCK_TOUCH_LEFT;
  }
  if (iqs323.channel_touchState(IQS323_CH1)) {
    return CLIENT_DRAW_CLOCK_TOUCH_MIDDLE;
  }
  if (iqs323.channel_touchState(IQS323_CH2)) {
    return CLIENT_DRAW_CLOCK_TOUCH_RIGHT;
  }
  return CLIENT_DRAW_CLOCK_TOUCH_NONE;
#else
  pinMode(PIN_INTERRUPT, INPUT);
  return (digitalRead(PIN_INTERRUPT) == LOW) ? CLIENT_DRAW_CLOCK_TOUCH_MIDDLE : CLIENT_DRAW_CLOCK_TOUCH_NONE;
#endif
}

// Poll the touchbar until the read is stable. Right after wake the chip can report
// a stale touch, so keep sampling (armed once clean) and only accept a touch that
// persists; a held touch is re-reported, so it is caught on a later wake if missed.
static int poll_touch_zone(bool *armed) {
#ifdef BOARD_TRMNL_X
  for (int i = 0; i < CLIENT_DRAW_CLOCK_TOUCH_SAMPLES; i++) {
    const int zone = read_touch_zone();
    if (zone == CLIENT_DRAW_CLOCK_TOUCH_NONE) {
      *armed = true; // clean read: state has settled, trust touches from here on
      return CLIENT_DRAW_CLOCK_TOUCH_NONE;
    }
    if (*armed) {
      return zone; // settled and a real touch is present
    }
    delay(CLIENT_DRAW_CLOCK_TOUCH_SAMPLE_MS); // stale touch on wake: re-sample
  }
  return CLIENT_DRAW_CLOCK_TOUCH_NONE;
#else
  (void)armed;
  return read_touch_zone();
#endif
}

static bool clamp_spec(ClientDrawClock *spec) {
  if (spec->x < 0) {
    spec->w += spec->x;
    spec->x = 0;
  }
  if (spec->y < 0) {
    spec->h += spec->y;
    spec->y = 0;
  }
  if (spec->x + spec->w > bbep.width()) {
    spec->w = bbep.width() - spec->x;
  }
  if (spec->y + spec->h > bbep.height()) {
    spec->h = bbep.height() - spec->y;
  }
  return spec->w >= 16 && spec->h >= 16;
}

static BB_RECT clock_refresh_rect_4bpp() {
  // FastEPD 4-bpp column clip on TRMNL X (MIRROR_X) updates the opposite
  // horizontal band — mirror x so the physical bottom-right clock tile refreshes.
  BB_RECT rect = {bbep.width() - s_spec.x - s_spec.w, s_spec.y, s_spec.w, s_spec.h};
  return rect;
}

// bKeepOn=false cuts the panel rails after each tick; FastEPD powers them back
// up on the next update.
static int refresh_clock_region(bool first_show) {
  (void)first_show;
  if (bbep.getMode() == BB_MODE_1BPP || bbep.getMode() == BB_MODE_2BPP) {
    if (bbep.getPreviousMode() == BB_MODE_NONE) {
      bbep.backupPlane();
    }
    return bbep.partialUpdate(false, s_spec.y, s_spec.y + s_spec.h - 1);
  }

  BB_RECT rect = clock_refresh_rect_4bpp();
  Log_info("client_draw_clock 4bpp refresh rect %d,%d %dx%d clear=%d tick=%d", rect.x, rect.y, rect.w, rect.h, CLEAR_FAST,
           s_refresh_tick);
  esp_task_wdt_reset();
  const int rc = bbep.fullUpdate(CLEAR_FAST, false, &rect);
  esp_task_wdt_reset();
  return rc;
}

static uint32_t ms_to_next_minute(const ClientDrawClock &spec) {
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  struct tm now;
  client_draw_clock_wall_time(&spec, (time_t)tv.tv_sec, &now);
  const uint32_t into_minute_ms = (uint32_t)now.tm_sec * 1000u + (uint32_t)(tv.tv_usec / 1000);
  return (into_minute_ms >= 60000u) ? 1000u : (60000u - into_minute_ms);
}

static const char *wake_cause_name(void) {
  switch (esp_sleep_get_wakeup_cause()) {
  case ESP_SLEEP_WAKEUP_TIMER:
    return "timer";
  case ESP_SLEEP_WAKEUP_GPIO:
    return "gpio";
  case ESP_SLEEP_WAKEUP_EXT0:
    return "ext0";
  case ESP_SLEEP_WAKEUP_EXT1:
    return "ext1";
  case ESP_SLEEP_WAKEUP_TOUCHPAD:
    return "touchpad";
  case ESP_SLEEP_WAKEUP_UART:
    return "uart";
  case ESP_SLEEP_WAKEUP_ULP:
    return "ulp";
  case ESP_SLEEP_WAKEUP_UNDEFINED:
    return "none";
  default:
    return "other";
  }
}

bool client_draw_clock_prepare(const uint8_t *png, int size) {
  s_active = false;
  s_last_drawn_min = -1;
  s_touched_zone = CLIENT_DRAW_CLOCK_TOUCH_NONE;
  memset(&s_spec, 0, sizeof(s_spec));

  if (!client_draw_clock_parse_png(png, (size_t)size, &s_spec)) {
    Log_info("client_draw_clock: no clock metadata in PNG (%d bytes)", size);
    // Log text chunks to distinguish stripped Comment vs wrong metadata format.
    size_t i = 8;
    int found = 0;
    if (png && size >= 8) {
      while (i + 12 <= (size_t)size) {
        uint32_t len =
          ((uint32_t)png[i] << 24) | ((uint32_t)png[i + 1] << 16) | ((uint32_t)png[i + 2] << 8) | png[i + 3];
        if (!client_draw_png_chunk_fits((uint32_t)i, len, (uint32_t)size)) {
          break;
        }
        const char *type = (const char *)(png + i + 4);
        if (memcmp(type, "IEND", 4) == 0) {
          break;
        }
        if ((memcmp(type, "tEXt", 4) == 0 || memcmp(type, "iTXt", 4) == 0 || memcmp(type, "zTXt", 4) == 0) && len > 0) {
          char preview[96];
          uint32_t n = len < 95 ? len : 95;
          for (uint32_t k = 0; k < n; k++) {
            uint8_t c = png[i + 8 + k];
            preview[k] = (c >= 32 && c < 127) ? (char)c : '.';
          }
          preview[n] = 0;
          Log_info("PNG %c%c%c%c: %s", type[0], type[1], type[2], type[3], preview);
          found++;
        }
        i += 12 + len;
      }
    }
    if (!found) {
      Log_info("PNG has no tEXt/iTXt/zTXt chunks");
    }
    return false;
  }

  Log_info("client_draw_clock metadata: %s %d,%d %dx%d tz=%ld (mode=%d)",
           s_spec.style == CLIENT_DRAW_CLOCK_ANALOG ? "analog" : "digital", s_spec.x, s_spec.y, s_spec.w, s_spec.h,
           (long)s_spec.tz, bbep.getMode());

  time_t now = time(nullptr);
  if (now < 1700000000) {
    Log_info("client_draw_clock skipped: time not synced (epoch=%ld)", (long)now);
    return false;
  }
  log_clock_time("prepare", s_spec);

  if (!clamp_spec(&s_spec)) {
    Log_info("client_draw_clock skipped: rect too small after clamp (%d,%d %dx%d)", s_spec.x, s_spec.y, s_spec.w, s_spec.h);
    return false;
  }

  s_active = true;
  return true;
}

void client_draw_clock_show(void) {
  if (!s_active) {
    return;
  }

  draw_clock_now(s_spec);
  {
    struct tm now;
    client_draw_clock_localtime(s_spec, &now);
    s_last_drawn_min = now.tm_min;
  }
  s_refresh_tick = 0;
  int rc = refresh_clock_region(true);
  if (rc != BBEP_SUCCESS) {
    Log_info("client_draw_clock initial refresh failed rc=%d mode=%d", rc, bbep.getMode());
    s_active = false;
    return;
  }
  s_refresh_tick = 1;
  log_clock_time("show", s_spec);
  Log_info("client_draw_clock: first frame on screen");
}

bool client_draw_clock_active(void) { return s_active; }

uint32_t client_draw_clock_consumed_s(void) { return s_consumed_s; }

int client_draw_clock_touched_zone(void) { return s_touched_zone; }

void client_draw_clock_release(void) {
  if (!s_active) {
    return;
  }
  s_active = false;
}

void client_draw_clock_run_loop(uint32_t budget_s) {
  if (!s_active || budget_s == 0) {
    return;
  }

  Log_info("client_draw_clock POC loop: %s at %d,%d %dx%d for %u s (minute ticks, touch to exit)",
           s_spec.style == CLIENT_DRAW_CLOCK_ANALOG ? "analog" : "digital", s_spec.x, s_spec.y, s_spec.w, s_spec.h,
           (unsigned)budget_s);

  bbep.backupPlane();

  const uint32_t start_ms = millis();
  const uint32_t budget_ms = budget_s * 1000u;
  int last_min = s_last_drawn_min;
  bool exit_armed = false;

  {
    // show() may have run near a minute boundary; refresh if the minute rolled before the loop.
    struct tm now;
    client_draw_clock_localtime(s_spec, &now);
    if (last_min >= 0 && now.tm_min != last_min) {
      draw_clock_now(s_spec);
      int rc = refresh_clock_region(false);
      if (rc != BBEP_SUCCESS) {
        Log_info("client_draw_clock catch-up refresh failed rc=%d mode=%d", rc, bbep.getMode());
        s_active = false;
        return;
      }
      last_min = now.tm_min;
      s_last_drawn_min = last_min;
      s_refresh_tick++;
      log_clock_time("catch-up", s_spec);
    }
  }

  while (millis() - start_ms < budget_ms) {
    // Cap nap at CLIENT_DRAW_CLOCK_NAP_MS so touch is polled even between minute boundaries.
    uint32_t nap_ms = ms_to_next_minute(s_spec);
    const uint32_t remaining_ms = budget_ms - (millis() - start_ms);
    if (nap_ms > remaining_ms) nap_ms = remaining_ms;
    if (nap_ms > CLIENT_DRAW_CLOCK_NAP_MS) nap_ms = CLIENT_DRAW_CLOCK_NAP_MS;
    if (nap_ms < 20) nap_ms = 20;

    const uint32_t nap_start_ms = millis();
#ifdef DO_NOT_LIGHT_SLEEP
    Log_info_serial("client_draw_clock: delay %u ms (DO_NOT_LIGHT_SLEEP, no light sleep)", (unsigned)nap_ms);
#else
    Log_info_serial("client_draw_clock: light sleep %u ms", (unsigned)nap_ms);
#endif
    Serial.flush(); // UART clock stops in light sleep
    display_sleep(nap_ms);
    Log_info_serial("client_draw_clock: awake after %u ms, cause=%s", (unsigned)(millis() - nap_start_ms),
                    wake_cause_name());

    esp_task_wdt_reset();
    const int zone = poll_touch_zone(&exit_armed);
    if (zone != CLIENT_DRAW_CLOCK_TOUCH_NONE) {
      s_touched_zone = zone;
      Log_info("client_draw_clock stopped by touchbar zone %d", zone);
      break;
    }

    struct tm now;
    client_draw_clock_localtime(s_spec, &now);

    if (now.tm_min != last_min) {
      last_min = now.tm_min;
      s_last_drawn_min = last_min;
      draw_clock_now(s_spec);
      int rc = refresh_clock_region(false);
      if (rc != BBEP_SUCCESS) {
        Log_info("client_draw_clock refresh failed rc=%d mode=%d prev=%d", rc, bbep.getMode(), bbep.getPreviousMode());
        break;
      }
      s_refresh_tick++;
      log_clock_time("tick", s_spec);
    }
  }

  s_consumed_s += (millis() - start_ms) / 1000u;
  Log_info("client_draw_clock loop done: %d ticks, %u s consumed", s_refresh_tick, (unsigned)s_consumed_s);

  s_active = false;
  bbep.einkPower(0);
}

#else // !PARALLEL_EPD

bool client_draw_clock_prepare(const uint8_t *png, int size) {
  (void)png;
  (void)size;
  return false;
}

void client_draw_clock_show(void) {}

bool client_draw_clock_active(void) { return false; }

void client_draw_clock_run_loop(uint32_t budget_s) { (void)budget_s; }

uint32_t client_draw_clock_consumed_s(void) { return 0; }

int client_draw_clock_touched_zone(void) { return CLIENT_DRAW_CLOCK_TOUCH_NONE; }

void client_draw_clock_release(void) {}

#endif // PARALLEL_EPD
