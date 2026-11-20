#include <reveal.h>
#include <string.h>

static const uint8_t PNG_SIGNATURE[8] = {0x89, 'P', 'N', 'G', '\r', '\n', 0x1a, '\n'};
static const char COMMENT_KEYWORD[] = "Comment";
static const uint8_t BAYER_4X4[16] = {0, 8, 2, 10, 12, 4, 14, 6, 3, 11, 1, 9, 15, 7, 13, 5};

static uint32_t read_be32(const uint8_t *p) {
  return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}

static RevealEffect effect_from(const char *name) {
  if (!name) return REVEAL_APPEAR;
  if (!strcmp(name, "fade")) return REVEAL_FADE;
  if (!strcmp(name, "wipe_up")) return REVEAL_WIPE_UP;
  if (!strcmp(name, "wipe_right")) return REVEAL_WIPE_RIGHT;
  if (!strcmp(name, "wipe_down")) return REVEAL_WIPE_DOWN;
  if (!strcmp(name, "wipe_left")) return REVEAL_WIPE_LEFT;
  return REVEAL_APPEAR;
}

static bool is_wipe(RevealEffect effect) { return effect >= REVEAL_WIPE_UP; }

RevealTiming reveal_parse_timing(JsonVariantConst reveal) {
  RevealTiming timing = {};
  if (!reveal.is<JsonObjectConst>()) return timing;
  timing.enabled = true;
  const char *refresh = reveal["refresh"] | "smooth";
  timing.refresh = strcmp(refresh, "per_region") ? REVEAL_REFRESH_SMOOTH : REVEAL_REFRESH_PER_REGION;
  timing.stagger_ms = reveal["stagger_ms"] | 0;
  timing.appear_effect = effect_from(reveal["appear_effect"] | "appear") == REVEAL_FADE ? REVEAL_FADE : REVEAL_APPEAR;
  timing.appear_ms = reveal["appear_ms"] | 0;
  timing.wipe_ms = reveal["wipe_ms"] | 0;
  timing.max_total_ms = reveal["max_total_ms"] | 0;
  return timing;
}

static int parse_records(const char *text, size_t length, RevealRegion *regions, int max_regions) {
  JsonDocument doc;
  if (deserializeJson(doc, text, length) || !doc.is<JsonArrayConst>()) return 0;
  int count = 0;
  for (JsonObjectConst record : doc.as<JsonArrayConst>()) {
    if (count >= max_regions) break;
    const char *name = record["name"];
    JsonArrayConst rect = record["rect"];
    if (!name || strcmp(name, "reveal") || rect.size() != 4) continue;
    const RevealEffect effect = effect_from(record["effect"]); // the timing picks fade, never a record
    RevealRegion region = {rect[0], rect[1], rect[2], rect[3], effect == REVEAL_FADE ? REVEAL_APPEAR : effect};
    if (region.w <= 0 || region.h <= 0) continue;
    regions[count++] = region;
  }
  return count;
}

int reveal_parse_png(const uint8_t *png, size_t size, RevealRegion *regions, int max_regions) {
  if (!png || size < sizeof(PNG_SIGNATURE) || memcmp(png, PNG_SIGNATURE, sizeof(PNG_SIGNATURE))) return 0;
  const size_t keyword_size = sizeof(COMMENT_KEYWORD); // the keyword and its NUL
  size_t offset = sizeof(PNG_SIGNATURE);
  while (offset + 12 <= size) {
    const uint32_t length = read_be32(png + offset);
    if (length > size - offset - 12) break; // a chunk that runs past the file
    const uint8_t *type = png + offset + 4;
    const uint8_t *data = png + offset + 8;
    if (!memcmp(type, "IEND", 4)) break;
    if (!memcmp(type, "tEXt", 4) && length > keyword_size && !memcmp(data, COMMENT_KEYWORD, keyword_size)) {
      return parse_records((const char *)data + keyword_size, length - keyword_size, regions, max_regions);
    }
    offset += length + 12;
  }
  return 0;
}

RevealEffect reveal_effect(const RevealRegion &region, const RevealTiming &timing) {
  return is_wipe(region.effect) ? region.effect : timing.appear_effect;
}

uint32_t reveal_duration_ms(const RevealRegion &region, const RevealTiming &timing) {
  return is_wipe(region.effect) ? timing.wipe_ms : timing.appear_ms;
}

uint32_t reveal_schedule(const RevealRegion *regions, int count, const RevealTiming &timing, uint32_t *start_ms) {
  uint32_t longest = 0;
  for (int i = 0; i < count; i++) {
    if (reveal_duration_ms(regions[i], timing) > longest) longest = reveal_duration_ms(regions[i], timing);
  }
  uint32_t stagger = timing.stagger_ms;
  if (count > 1) {
    const uint32_t room = timing.max_total_ms > longest ? timing.max_total_ms - longest : 0;
    if (room / (count - 1) < stagger) stagger = room / (count - 1);
  }
  uint32_t total = 0;
  for (int i = 0; i < count; i++) {
    start_ms[i] = i * stagger;
    if (start_ms[i] + reveal_duration_ms(regions[i], timing) > total)
      total = start_ms[i] + reveal_duration_ms(regions[i], timing);
  }
  return total;
}

uint16_t reveal_progress(uint32_t start_ms, uint32_t duration_ms, uint32_t elapsed_ms) {
  if (elapsed_ms < start_ms) return 0;
  if (duration_ms == 0 || elapsed_ms - start_ms >= duration_ms) return REVEAL_PROGRESS_WHOLE;
  return (uint16_t)((elapsed_ms - start_ms) * REVEAL_PROGRESS_WHOLE / duration_ms);
}

bool reveal_pixel_shown(const RevealRegion &region, RevealEffect effect, uint16_t progress, int x, int y) {
  if (progress >= REVEAL_PROGRESS_WHOLE) return true;
  const int32_t whole = REVEAL_PROGRESS_WHOLE;
  switch (effect) {
  case REVEAL_WIPE_RIGHT:
    return (int32_t)(x - region.x) * whole < (int32_t)region.w * progress;
  case REVEAL_WIPE_LEFT:
    return (int32_t)(region.x + region.w - x) * whole <= (int32_t)region.w * progress;
  case REVEAL_WIPE_DOWN:
    return (int32_t)(y - region.y) * whole < (int32_t)region.h * progress;
  case REVEAL_WIPE_UP:
    return (int32_t)(region.y + region.h - y) * whole <= (int32_t)region.h * progress;
  case REVEAL_FADE:
    return (int32_t)BAYER_4X4[(x & 3) + (y & 3) * 4] * whole < (int32_t)progress * 16;
  default:
    return false;
  }
}

void reveal_turn_region(RevealRegion *region, int panel_height) {
  const RevealRegion png = *region;
  region->x = png.y;
  region->y = panel_height - png.x - png.w;
  region->w = png.h;
  region->h = png.w;
  switch (png.effect) { // a quarter turn counterclockwise
  case REVEAL_WIPE_RIGHT:
    region->effect = REVEAL_WIPE_UP;
    break;
  case REVEAL_WIPE_UP:
    region->effect = REVEAL_WIPE_LEFT;
    break;
  case REVEAL_WIPE_LEFT:
    region->effect = REVEAL_WIPE_DOWN;
    break;
  case REVEAL_WIPE_DOWN:
    region->effect = REVEAL_WIPE_RIGHT;
    break;
  default:
    break;
  }
}
