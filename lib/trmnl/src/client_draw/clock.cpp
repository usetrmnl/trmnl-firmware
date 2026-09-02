#include <ArduinoJson.h>
#include <client_draw/spec.h>
#include <string.h>
#include <time.h>

void client_draw_clock_wall_time(const ClientDrawClock *spec, time_t utc_now, struct tm *out) {
  if (!spec || !out) {
    return;
  }
  const time_t t = utc_now + (time_t)spec->tz;
  gmtime_r(&t, out);
}

bool client_draw_png_chunk_fits(uint32_t off, uint32_t len, uint32_t size) {
  if (size < 12u || off > size - 12u) {
    return false;
  }
  return len <= size - off - 12u;
}

static uint32_t be32(const uint8_t *p) {
  return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | (uint32_t)p[3];
}

static bool ieq(const char *a, const char *b) {
  while (*a && *b) {
    char ca = *a++;
    char cb = *b++;
    if (ca >= 'A' && ca <= 'Z') {
      ca = (char)(ca + 32);
    }
    if (cb >= 'A' && cb <= 'Z') {
      cb = (char)(cb + 32);
    }
    if (ca != cb) {
      return false;
    }
  }
  return *a == *b;
}

static bool style_from_type(const char *type, ClientDrawClockStyle *style) {
  if (!type || !style) {
    return false;
  }
  if (ieq(type, "digital")) {
    *style = CLIENT_DRAW_CLOCK_DIGITAL;
    return true;
  }
  if (ieq(type, "analog")) {
    *style = CLIENT_DRAW_CLOCK_ANALOG;
    return true;
  }
  return false;
}

bool client_draw_clock_parse_json(const char *text, ClientDrawClock *out) {
  if (!text || !out) {
    return false;
  }

  JsonDocument doc;
  if (deserializeJson(doc, text)) {
    return false;
  }
  if (!doc.is<JsonArray>()) {
    return false;
  }

  for (JsonObject obj : doc.as<JsonArray>()) {
    const char *name = obj["name"];
    if (!name || !ieq(name, "clock")) {
      continue;
    }
    if ((obj["v"] | 0) != 1) {
      continue;
    }

    JsonArray rect = obj["rect"].as<JsonArray>();
    if (rect.size() != 4) {
      continue;
    }

    const int w = rect[2].as<int>();
    const int h = rect[3].as<int>();
    if (w <= 0 || h <= 0) {
      continue;
    }

    ClientDrawClockStyle style = CLIENT_DRAW_CLOCK_NONE;
    const char *type = obj["type"];
    if (!style_from_type(type, &style)) {
      continue;
    }

    ClientDrawClock parsed;
    memset(&parsed, 0, sizeof(parsed));
    parsed.style = style;
    parsed.x = rect[0].as<int>();
    parsed.y = rect[1].as<int>();
    parsed.w = w;
    parsed.h = h;
    if (obj["tz"].isNull()) {
      continue;
    }
    parsed.tz = obj["tz"].as<int32_t>();
    *out = parsed;
    return true;
  }

  return false;
}

static bool try_text_payload(const uint8_t *text, uint32_t text_len, ClientDrawClock *out) {
  if (!text || text_len == 0) {
    return false;
  }
  char buf[256];
  if (text_len >= sizeof(buf)) {
    text_len = sizeof(buf) - 1;
  }
  memcpy(buf, text, text_len);
  buf[text_len] = 0;
  return client_draw_clock_parse_json(buf, out);
}

bool client_draw_clock_parse_png(const uint8_t *png, size_t size, ClientDrawClock *out) {
  static const uint8_t kSig[8] = {0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a};

  if (!png || !out || size < 8 || memcmp(png, kSig, 8) != 0) {
    return false;
  }

  size_t i = 8;
  while (i + 12 <= size) {
    uint32_t len = be32(png + i);
    if (!client_draw_png_chunk_fits((uint32_t)i, len, (uint32_t)size)) {
      break;
    }

    const uint8_t *type = png + i + 4;
    const uint8_t *data = png + i + 8;

    if (memcmp(type, "IEND", 4) == 0) {
      break;
    }

    if (memcmp(type, "tEXt", 4) == 0 && len >= 2) {
      uint32_t kw_end = 0;
      while (kw_end < len && data[kw_end] != 0) {
        kw_end++;
      }
      if (kw_end < len && try_text_payload(data + kw_end + 1, len - kw_end - 1, out)) {
        return true;
      }
    }

    if (memcmp(type, "iTXt", 4) == 0 && len >= 5) { // uncompressed payload only
      uint32_t pos = 0;
      while (pos < len && data[pos] != 0) {
        pos++;
      }
      if (pos + 2 < len) {
        // keyword \0 | flag | method | language \0 | translated \0 | text
        const uint8_t compression = data[pos + 1];
        pos += 3; // keyword NUL, flag, method
        while (pos < len && data[pos] != 0) {
          pos++;
        }
        if (pos < len) {
          pos++;
        }
        while (pos < len && data[pos] != 0) {
          pos++;
        }
        if (pos < len) {
          pos++;
        }
        if (compression == 0 && pos < len && try_text_payload(data + pos, len - pos, out)) {
          return true;
        }
      }
    }

    i += 12 + len;
  }

  return false;
}
