#include <client_draw/clock.h>
#include <string.h>
#include <unity.h>

static const uint8_t kPngSig[8] = {0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a};

static void put_be32(uint8_t *p, uint32_t v) {
  p[0] = (uint8_t)(v >> 24);
  p[1] = (uint8_t)(v >> 16);
  p[2] = (uint8_t)(v >> 8);
  p[3] = (uint8_t)v;
}

static int append_chunk(uint8_t *out, int pos, const char type[4], const uint8_t *data, int len) {
  put_be32(out + pos, (uint32_t)len);
  memcpy(out + pos + 4, type, 4);
  if (len && data) {
    memcpy(out + pos + 8, data, (size_t)len);
  }
  put_be32(out + pos + 8 + len, 0);
  return pos + 12 + len;
}

static int make_png_with_comment(uint8_t *out, int out_max, const char *comment) {
  uint8_t ihdr[13] = {0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 0};
  uint8_t text[160];
  const char *kw = "Comment";
  int kw_len = (int)strlen(kw);
  int c_len = (int)strlen(comment);
  TEST_ASSERT_LESS_THAN(out_max, 8 + 12 + 13 + 12 + kw_len + 1 + c_len + 12);

  memcpy(out, kPngSig, 8);
  int pos = 8;
  pos = append_chunk(out, pos, "IHDR", ihdr, 13);
  memcpy(text, kw, (size_t)kw_len);
  text[kw_len] = 0;
  memcpy(text + kw_len + 1, comment, (size_t)c_len);
  pos = append_chunk(out, pos, "tEXt", text, kw_len + 1 + c_len);
  pos = append_chunk(out, pos, "IEND", NULL, 0);
  return pos;
}

void test_parse_digital_spec(void) {
  ClientDrawClock spec;
  TEST_ASSERT_TRUE(client_draw_clock_parse_json(
      "[{\"v\":1,\"name\":\"clock\",\"rect\":[176,16,448,448],\"type\":\"digital\",\"tz\":0}]", &spec));
  TEST_ASSERT_EQUAL(CLIENT_DRAW_CLOCK_DIGITAL, spec.style);
  TEST_ASSERT_EQUAL(176, spec.x);
  TEST_ASSERT_EQUAL(16, spec.y);
  TEST_ASSERT_EQUAL(448, spec.w);
  TEST_ASSERT_EQUAL(448, spec.h);
  TEST_ASSERT_EQUAL(0, spec.tz);
}

void test_parse_spec_allows_spaces(void) {
  ClientDrawClock spec;
  TEST_ASSERT_TRUE(client_draw_clock_parse_json(
      "[ { \"v\" : 1, \"name\" : \"clock\", \"rect\" : [176, 16, 448, 448], \"type\" : \"digital\", \"tz\" : 0 } ]",
      &spec));
  TEST_ASSERT_EQUAL(CLIENT_DRAW_CLOCK_DIGITAL, spec.style);
  TEST_ASSERT_EQUAL(176, spec.x);
  TEST_ASSERT_EQUAL(448, spec.h);
}

void test_parse_analog_spec(void) {
  ClientDrawClock spec;
  TEST_ASSERT_TRUE(client_draw_clock_parse_json(
      "[{\"v\":1,\"name\":\"clock\",\"rect\":[0,0,400,240],\"type\":\"analog\",\"tz\":11800}]", &spec));
  TEST_ASSERT_EQUAL(CLIENT_DRAW_CLOCK_ANALOG, spec.style);
  TEST_ASSERT_EQUAL(0, spec.x);
  TEST_ASSERT_EQUAL(0, spec.y);
  TEST_ASSERT_EQUAL(400, spec.w);
  TEST_ASSERT_EQUAL(240, spec.h);
  TEST_ASSERT_EQUAL(11800, spec.tz);
}

void test_parse_spec_ignores_non_clock_entries(void) {
  ClientDrawClock spec;
  TEST_ASSERT_TRUE(client_draw_clock_parse_json(
      "[{\"v\":1,\"name\":\"title\",\"rect\":[0,0,1,1],\"type\":\"digital\",\"tz\":0},"
      "{\"v\":1,\"name\":\"clock\",\"rect\":[1,2,3,4],\"type\":\"digital\",\"tz\":19800}]",
      &spec));
  TEST_ASSERT_EQUAL(CLIENT_DRAW_CLOCK_DIGITAL, spec.style);
  TEST_ASSERT_EQUAL(1, spec.x);
  TEST_ASSERT_EQUAL(2, spec.y);
  TEST_ASSERT_EQUAL(3, spec.w);
  TEST_ASSERT_EQUAL(4, spec.h);
  TEST_ASSERT_EQUAL(19800, spec.tz);
}

void test_parse_spec_rejects_non_json(void) {
  ClientDrawClock spec;
  TEST_ASSERT_FALSE(client_draw_clock_parse_json("clock,digital,176,16,448,448", &spec));
}

void test_parse_spec_rejects_unknown_style(void) {
  ClientDrawClock spec;
  memset(&spec, 0xff, sizeof(spec));
  TEST_ASSERT_FALSE(client_draw_clock_parse_json(
      "[{\"v\":1,\"name\":\"clock\",\"rect\":[0,0,10,10],\"type\":\"binary\",\"tz\":0}]", &spec));
}

void test_parse_spec_rejects_missing_tz(void) {
  ClientDrawClock spec;
  TEST_ASSERT_FALSE(client_draw_clock_parse_json(
      "[{\"v\":1,\"name\":\"clock\",\"rect\":[0,0,10,10],\"type\":\"digital\"}]", &spec));
}

void test_clock_wall_time_applies_tz(void) {
  ClientDrawClock spec = {};
  spec.tz = 19800; // UTC+5:30
  struct tm out;
  client_draw_clock_wall_time(&spec, 0, &out);
  TEST_ASSERT_EQUAL(5, out.tm_hour);
  TEST_ASSERT_EQUAL(30, out.tm_min);
}

void test_parse_spec_rejects_zero_size(void) {
  ClientDrawClock spec;
  TEST_ASSERT_FALSE(client_draw_clock_parse_json(
      "[{\"v\":1,\"name\":\"clock\",\"rect\":[0,0,0,10],\"type\":\"digital\",\"tz\":0}]", &spec));
}

void test_parse_png_comment_chunk(void) {
  uint8_t png[256];
  int len = make_png_with_comment(
      png, (int)sizeof(png),
      "[{\"v\":1,\"name\":\"clock\",\"rect\":[176,16,448,448],\"type\":\"digital\",\"tz\":0}]");
  ClientDrawClock spec;
  TEST_ASSERT_TRUE(client_draw_clock_parse_png(png, (size_t)len, &spec));
  TEST_ASSERT_EQUAL(CLIENT_DRAW_CLOCK_DIGITAL, spec.style);
  TEST_ASSERT_EQUAL(176, spec.x);
  TEST_ASSERT_EQUAL(16, spec.y);
  TEST_ASSERT_EQUAL(448, spec.w);
  TEST_ASSERT_EQUAL(448, spec.h);
}

void test_parse_png_any_keyword(void) {
  // Same as Comment, but keyword "Description"
  uint8_t out[256];
  uint8_t ihdr[13] = {0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 0};
  auto put_be32 = [](uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)(v >> 24);
    p[1] = (uint8_t)(v >> 16);
    p[2] = (uint8_t)(v >> 8);
    p[3] = (uint8_t)v;
  };
  auto append_chunk = [&](int pos, const char type[4], const uint8_t *data, int len) {
    put_be32(out + pos, (uint32_t)len);
    memcpy(out + pos + 4, type, 4);
    if (len && data) {
      memcpy(out + pos + 8, data, (size_t)len);
    }
    put_be32(out + pos + 8 + len, 0);
    return pos + 12 + len;
  };
  static const uint8_t kPngSig[8] = {0x89, 0x50, 0x4e, 0x47, 0x0d, 0x0a, 0x1a, 0x0a};
  memcpy(out, kPngSig, 8);
  int pos = 8;
  pos = append_chunk(pos, "IHDR", ihdr, 13);
  static const char kJson[] = "[{\"v\":1,\"name\":\"clock\",\"rect\":[5,6,70,80],\"type\":\"analog\",\"tz\":0}]";
  uint8_t text[128];
  memcpy(text, "Description", 11);
  text[11] = 0;
  memcpy(text + 12, kJson, sizeof(kJson));
  pos = append_chunk(pos, "tEXt", text, 11 + 1 + (int)(sizeof(kJson) - 1));
  pos = append_chunk(pos, "IEND", NULL, 0);

  ClientDrawClock spec;
  TEST_ASSERT_TRUE(client_draw_clock_parse_png(out, (size_t)pos, &spec));
  TEST_ASSERT_EQUAL(CLIENT_DRAW_CLOCK_ANALOG, spec.style);
  TEST_ASSERT_EQUAL(5, spec.x);
  TEST_ASSERT_EQUAL(80, spec.h);
}

void test_parse_png_without_clock_comment(void) {
  uint8_t png[256];
  int len = make_png_with_comment(png, (int)sizeof(png), "hello world");
  ClientDrawClock spec;
  TEST_ASSERT_FALSE(client_draw_clock_parse_png(png, (size_t)len, &spec));
}

void test_parse_png_rejects_truncated(void) {
  ClientDrawClock spec;
  const uint8_t junk[] = {0x89, 0x50, 0x4e, 0x47};
  TEST_ASSERT_FALSE(client_draw_clock_parse_png(junk, sizeof(junk), &spec));
  TEST_ASSERT_FALSE(client_draw_clock_parse_png(NULL, 0, &spec));
}

void test_parse_png_itxt_chunk(void) {
  // iTXt layout: keyword \0 | flag | method | language \0 | translated \0 | text
  uint8_t png[256];
  uint8_t chunk[160];
  static const char kJson[] = "[{\"v\":1,\"name\":\"clock\",\"rect\":[7,8,90,100],\"type\":\"analog\",\"tz\":3600}]";
  int n = 0;
  memcpy(chunk, "Comment", 7);
  n = 7;
  chunk[n++] = 0; // keyword terminator
  chunk[n++] = 0; // compression flag: uncompressed
  chunk[n++] = 0; // compression method
  memcpy(chunk + n, "en", 2);
  n += 2;
  chunk[n++] = 0; // language tag terminator
  chunk[n++] = 0; // empty translated keyword
  memcpy(chunk + n, kJson, sizeof(kJson) - 1);
  n += (int)(sizeof(kJson) - 1);

  memcpy(png, kPngSig, 8);
  int pos = 8;
  uint8_t ihdr[13] = {0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 0};
  pos = append_chunk(png, pos, "IHDR", ihdr, 13);
  pos = append_chunk(png, pos, "iTXt", chunk, n);
  pos = append_chunk(png, pos, "IEND", NULL, 0);

  ClientDrawClock spec;
  TEST_ASSERT_TRUE(client_draw_clock_parse_png(png, (size_t)pos, &spec));
  TEST_ASSERT_EQUAL(CLIENT_DRAW_CLOCK_ANALOG, spec.style);
  TEST_ASSERT_EQUAL(7, spec.x);
  TEST_ASSERT_EQUAL(8, spec.y);
  TEST_ASSERT_EQUAL(90, spec.w);
  TEST_ASSERT_EQUAL(100, spec.h);
  TEST_ASSERT_EQUAL(3600, spec.tz);
}

void test_png_chunk_fits_rejects_overflowing_length(void) {
  // 8 + 12 + 0xfffffff8 wraps to 4 in 32-bit arithmetic, so a bounds check
  // written as "off + 12 + len > size" accepts this chunk and reads past the end.
  TEST_ASSERT_FALSE(client_draw_png_chunk_fits(8, 0xfffffff8u, 64));
  TEST_ASSERT_FALSE(client_draw_png_chunk_fits(8, 0xffffffffu, 64));
  TEST_ASSERT_FALSE(client_draw_png_chunk_fits(8, 0xfffffff4u, 64));
}

void test_png_chunk_fits_accepts_exact_fit(void) {
  TEST_ASSERT_TRUE(client_draw_png_chunk_fits(8, 0, 20));  // header ends exactly at the buffer end
  TEST_ASSERT_TRUE(client_draw_png_chunk_fits(8, 10, 30));
  TEST_ASSERT_FALSE(client_draw_png_chunk_fits(8, 11, 30)); // one byte too long
}

void test_png_chunk_fits_rejects_short_buffer(void) {
  TEST_ASSERT_FALSE(client_draw_png_chunk_fits(0, 0, 11));  // smaller than the chunk overhead
  TEST_ASSERT_FALSE(client_draw_png_chunk_fits(60, 0, 64)); // header itself runs past the end
}

void setUp(void) {}
void tearDown(void) {}

void process() {
  UNITY_BEGIN();
  RUN_TEST(test_parse_digital_spec);
  RUN_TEST(test_parse_spec_allows_spaces);
  RUN_TEST(test_parse_analog_spec);
  RUN_TEST(test_parse_spec_ignores_non_clock_entries);
  RUN_TEST(test_parse_spec_rejects_non_json);
  RUN_TEST(test_parse_spec_rejects_unknown_style);
  RUN_TEST(test_parse_spec_rejects_missing_tz);
  RUN_TEST(test_parse_spec_rejects_zero_size);
  RUN_TEST(test_clock_wall_time_applies_tz);
  RUN_TEST(test_parse_png_comment_chunk);
  RUN_TEST(test_parse_png_any_keyword);
  RUN_TEST(test_parse_png_without_clock_comment);
  RUN_TEST(test_parse_png_rejects_truncated);
  RUN_TEST(test_parse_png_itxt_chunk);
  RUN_TEST(test_png_chunk_fits_rejects_overflowing_length);
  RUN_TEST(test_png_chunk_fits_accepts_exact_fit);
  RUN_TEST(test_png_chunk_fits_rejects_short_buffer);
  UNITY_END();
}

int main(int argc, char **argv) {
  process();
  return 0;
}
