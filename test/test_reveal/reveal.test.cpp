#include <ArduinoJson.h>
#include <reveal.h>
#include <string>
#include <unity.h>

// A PNG with an IHDR and one tEXt chunk per keyword/text pair; the parser does not check CRCs.
static std::string png_with_text(const std::string &keyword, const std::string &text) {
  auto chunk = [](const std::string &type, const std::string &data) {
    std::string out;
    uint32_t len = data.size();
    out += (char)(len >> 24);
    out += (char)(len >> 16);
    out += (char)(len >> 8);
    out += (char)len;
    out += type + data + std::string(4, '\0');
    return out;
  };
  std::string png("\x89PNG\r\n\x1a\n", 8);
  png += chunk("IHDR", std::string(13, '\0'));
  if (!keyword.empty()) png += chunk("tEXt", keyword + std::string(1, '\0') + text);
  png += chunk("IEND", "");
  return png;
}

static int parse(const std::string &png, RevealRegion *regions, int max = REVEAL_MAX_REGIONS) {
  return reveal_parse_png((const uint8_t *)png.data(), png.size(), regions, max);
}

static RevealTiming timing_from(const char *json) {
  JsonDocument doc;
  deserializeJson(doc, json);
  return reveal_parse_timing(doc["reveal"]);
}

static const char *LIVELY = "{\"reveal\":{\"refresh\":\"per_region\",\"stagger_ms\":120,\"appear_effect\":\"fade\","
                            "\"appear_ms\":400,\"wipe_ms\":1000,\"max_total_ms\":3600}}";

void test_timing_reads_every_field(void) {
  RevealTiming timing = timing_from(LIVELY);
  TEST_ASSERT_TRUE(timing.enabled);
  TEST_ASSERT_EQUAL(REVEAL_REFRESH_PER_REGION, timing.refresh);
  TEST_ASSERT_EQUAL(120, timing.stagger_ms);
  TEST_ASSERT_EQUAL(REVEAL_FADE, timing.appear_effect);
  TEST_ASSERT_EQUAL(400, timing.appear_ms);
  TEST_ASSERT_EQUAL(1000, timing.wipe_ms);
  TEST_ASSERT_EQUAL(3600, timing.max_total_ms);
}

void test_timing_defaults_to_smooth_and_appear(void) {
  RevealTiming timing = timing_from("{\"reveal\":{\"stagger_ms\":200,\"max_total_ms\":2400}}");
  TEST_ASSERT_EQUAL(REVEAL_REFRESH_SMOOTH, timing.refresh);
  TEST_ASSERT_EQUAL(REVEAL_APPEAR, timing.appear_effect);
}

void test_timing_is_off_without_a_reveal_object(void) { TEST_ASSERT_FALSE(timing_from("{\"status\":0}").enabled); }

void test_png_reads_only_reveal_records(void) {
  RevealRegion regions[REVEAL_MAX_REGIONS];
  std::string comment = "[{\"v\":1,\"name\":\"clock\",\"rect\":[0,0,10,10],\"type\":\"analog\",\"tz\":0},"
                        "{\"v\":1,\"name\":\"reveal\",\"rect\":[18,20,445,112],\"effect\":\"appear\"},"
                        "{\"v\":1,\"name\":\"reveal\",\"rect\":[163,271,115,76],\"effect\":\"wipe_up\"}]";
  TEST_ASSERT_EQUAL(2, parse(png_with_text("Comment", comment), regions));
  TEST_ASSERT_EQUAL(18, regions[0].x);
  TEST_ASSERT_EQUAL(20, regions[0].y);
  TEST_ASSERT_EQUAL(445, regions[0].w);
  TEST_ASSERT_EQUAL(112, regions[0].h);
  TEST_ASSERT_EQUAL(REVEAL_APPEAR, regions[0].effect);
  TEST_ASSERT_EQUAL(REVEAL_WIPE_UP, regions[1].effect);
}

void test_png_reads_a_comment_past_256_bytes(void) {
  RevealRegion regions[REVEAL_MAX_REGIONS];
  std::string comment = "[";
  for (int i = 0; i < 40; i++) {
    if (i) comment += ",";
    comment += "{\"v\":1,\"name\":\"reveal\",\"rect\":[" + std::to_string(i) + ",0,10,10],\"effect\":\"wipe_right\"}";
  }
  comment += "]";
  TEST_ASSERT_EQUAL(40, parse(png_with_text("Comment", comment), regions));
  TEST_ASSERT_EQUAL(39, regions[39].x);
}

void test_png_stops_at_the_region_limit(void) {
  RevealRegion regions[2];
  std::string comment = "[{\"name\":\"reveal\",\"rect\":[0,0,1,1],\"effect\":\"appear\"},"
                        "{\"name\":\"reveal\",\"rect\":[0,0,1,1],\"effect\":\"appear\"},"
                        "{\"name\":\"reveal\",\"rect\":[0,0,1,1],\"effect\":\"appear\"}]";
  TEST_ASSERT_EQUAL(2, parse(png_with_text("Comment", comment), regions, 2));
}

void test_png_falls_back_to_appear_for_an_unknown_effect(void) {
  RevealRegion regions[1];
  parse(png_with_text("Comment", "[{\"name\":\"reveal\",\"rect\":[0,0,4,4],\"effect\":\"spin\"}]"), regions, 1);
  TEST_ASSERT_EQUAL(REVEAL_APPEAR, regions[0].effect);
}

void test_png_skips_a_region_with_no_area(void) {
  RevealRegion regions[1];
  TEST_ASSERT_EQUAL(
    0, parse(png_with_text("Comment", "[{\"name\":\"reveal\",\"rect\":[0,0,0,4],\"effect\":\"appear\"}]"), regions));
}

void test_png_ignores_other_text_chunks(void) {
  RevealRegion regions[1];
  TEST_ASSERT_EQUAL(0, parse(png_with_text("Software", "[{\"name\":\"reveal\",\"rect\":[0,0,4,4]}]"), regions));
}

void test_png_without_a_comment_has_no_regions(void) {
  RevealRegion regions[1];
  TEST_ASSERT_EQUAL(0, parse(png_with_text("", ""), regions));
}

void test_png_with_a_comment_that_is_not_json_has_no_regions(void) {
  RevealRegion regions[1];
  TEST_ASSERT_EQUAL(0, parse(png_with_text("Comment", "made by ImageMagick"), regions));
}

void test_png_with_a_chunk_longer_than_the_file_stops_safely(void) {
  RevealRegion regions[1];
  std::string png = png_with_text("Comment", "[{\"name\":\"reveal\",\"rect\":[0,0,4,4]}]");
  TEST_ASSERT_EQUAL(0, parse(png.substr(0, png.size() - 20), regions));
}

void test_a_file_that_is_not_a_png_has_no_regions(void) {
  RevealRegion regions[1];
  TEST_ASSERT_EQUAL(0, parse("BM not a png at all", regions));
}

void test_schedule_staggers_each_region(void) {
  RevealTiming timing = timing_from(LIVELY);
  RevealRegion regions[3] = {{0, 0, 1, 1, REVEAL_APPEAR}, {0, 0, 1, 1, REVEAL_APPEAR}, {0, 0, 1, 1, REVEAL_WIPE_UP}};
  uint32_t start_ms[3];
  TEST_ASSERT_EQUAL(1240, reveal_schedule(regions, 3, timing, start_ms));
  TEST_ASSERT_EQUAL(0, start_ms[0]);
  TEST_ASSERT_EQUAL(120, start_ms[1]);
  TEST_ASSERT_EQUAL(240, start_ms[2]);
}

void test_schedule_shortens_the_stagger_to_fit_the_total(void) {
  RevealTiming timing = timing_from(LIVELY);
  RevealRegion regions[REVEAL_MAX_REGIONS];
  for (int i = 0; i < REVEAL_MAX_REGIONS; i++)
    regions[i] = {0, 0, 1, 1, REVEAL_WIPE_RIGHT};
  uint32_t start_ms[REVEAL_MAX_REGIONS];
  TEST_ASSERT_EQUAL(47 * 55 + 1000, reveal_schedule(regions, REVEAL_MAX_REGIONS, timing, start_ms));
  TEST_ASSERT_EQUAL(55, start_ms[1]);
}

void test_schedule_of_one_region_has_no_stagger(void) {
  RevealTiming timing = timing_from(LIVELY);
  RevealRegion regions[1] = {{0, 0, 1, 1, REVEAL_WIPE_UP}};
  uint32_t start_ms[1];
  TEST_ASSERT_EQUAL(1000, reveal_schedule(regions, 1, timing, start_ms));
}

void test_progress_runs_from_start_to_end(void) {
  TEST_ASSERT_EQUAL(0, reveal_progress(100, 400, 50));
  TEST_ASSERT_EQUAL(500, reveal_progress(100, 400, 300));
  TEST_ASSERT_EQUAL(1000, reveal_progress(100, 400, 900));
}

void test_progress_of_an_instant_region_is_whole_once_started(void) {
  TEST_ASSERT_EQUAL(0, reveal_progress(100, 0, 99));
  TEST_ASSERT_EQUAL(1000, reveal_progress(100, 0, 100));
}

void test_a_right_wipe_shows_the_left_part_first(void) {
  RevealRegion region = {100, 0, 100, 10, REVEAL_WIPE_RIGHT};
  TEST_ASSERT_TRUE(reveal_pixel_shown(region, REVEAL_WIPE_RIGHT, 500, 149, 5));
  TEST_ASSERT_FALSE(reveal_pixel_shown(region, REVEAL_WIPE_RIGHT, 500, 150, 5));
}

void test_an_upward_wipe_shows_the_bottom_part_first(void) {
  RevealRegion region = {0, 100, 10, 100, REVEAL_WIPE_UP};
  TEST_ASSERT_TRUE(reveal_pixel_shown(region, REVEAL_WIPE_UP, 500, 5, 150));
  TEST_ASSERT_FALSE(reveal_pixel_shown(region, REVEAL_WIPE_UP, 500, 5, 149));
}

void test_a_downward_and_a_left_wipe_mirror_them(void) {
  RevealRegion region = {0, 0, 100, 100, REVEAL_WIPE_DOWN};
  TEST_ASSERT_TRUE(reveal_pixel_shown(region, REVEAL_WIPE_DOWN, 500, 5, 49));
  TEST_ASSERT_FALSE(reveal_pixel_shown(region, REVEAL_WIPE_DOWN, 500, 5, 50));
  TEST_ASSERT_TRUE(reveal_pixel_shown(region, REVEAL_WIPE_LEFT, 500, 50, 5));
  TEST_ASSERT_FALSE(reveal_pixel_shown(region, REVEAL_WIPE_LEFT, 500, 49, 5));
}

void test_a_fade_shows_more_dots_as_it_runs(void) {
  RevealRegion region = {0, 0, 4, 4, REVEAL_APPEAR};
  int shown_half = 0, shown_none = 0, shown_all = 0;
  for (int y = 0; y < 4; y++) {
    for (int x = 0; x < 4; x++) {
      shown_none += reveal_pixel_shown(region, REVEAL_FADE, 0, x, y);
      shown_half += reveal_pixel_shown(region, REVEAL_FADE, 500, x, y);
      shown_all += reveal_pixel_shown(region, REVEAL_FADE, 1000, x, y);
    }
  }
  TEST_ASSERT_EQUAL(0, shown_none);
  TEST_ASSERT_EQUAL(8, shown_half);
  TEST_ASSERT_EQUAL(16, shown_all);
}

void test_an_appearing_region_shows_whole_once_due(void) {
  RevealRegion region = {0, 0, 4, 4, REVEAL_APPEAR};
  TEST_ASSERT_FALSE(reveal_pixel_shown(region, REVEAL_APPEAR, 999, 0, 0));
  TEST_ASSERT_TRUE(reveal_pixel_shown(region, REVEAL_APPEAR, 1000, 0, 0));
}

void test_an_appearing_region_takes_the_timing_effect(void) {
  RevealTiming timing = timing_from(LIVELY);
  RevealRegion appear = {0, 0, 1, 1, REVEAL_APPEAR};
  RevealRegion wipe = {0, 0, 1, 1, REVEAL_WIPE_UP};
  TEST_ASSERT_EQUAL(REVEAL_FADE, reveal_effect(appear, timing));
  TEST_ASSERT_EQUAL(REVEAL_WIPE_UP, reveal_effect(wipe, timing));
}

void test_turning_a_region_matches_a_portrait_png_drawn_turned(void) {
  RevealRegion region = {10, 20, 300, 40, REVEAL_WIPE_RIGHT};
  reveal_turn_region(&region, 1404);
  TEST_ASSERT_EQUAL(20, region.x);
  TEST_ASSERT_EQUAL(1404 - 10 - 300, region.y);
  TEST_ASSERT_EQUAL(40, region.w);
  TEST_ASSERT_EQUAL(300, region.h);
  TEST_ASSERT_EQUAL(REVEAL_WIPE_UP, region.effect);
}

int main(int argc, char **argv) {
  UNITY_BEGIN();
  RUN_TEST(test_timing_reads_every_field);
  RUN_TEST(test_timing_defaults_to_smooth_and_appear);
  RUN_TEST(test_timing_is_off_without_a_reveal_object);
  RUN_TEST(test_png_reads_only_reveal_records);
  RUN_TEST(test_png_reads_a_comment_past_256_bytes);
  RUN_TEST(test_png_stops_at_the_region_limit);
  RUN_TEST(test_png_falls_back_to_appear_for_an_unknown_effect);
  RUN_TEST(test_png_skips_a_region_with_no_area);
  RUN_TEST(test_png_ignores_other_text_chunks);
  RUN_TEST(test_png_without_a_comment_has_no_regions);
  RUN_TEST(test_png_with_a_comment_that_is_not_json_has_no_regions);
  RUN_TEST(test_png_with_a_chunk_longer_than_the_file_stops_safely);
  RUN_TEST(test_a_file_that_is_not_a_png_has_no_regions);
  RUN_TEST(test_schedule_staggers_each_region);
  RUN_TEST(test_schedule_shortens_the_stagger_to_fit_the_total);
  RUN_TEST(test_schedule_of_one_region_has_no_stagger);
  RUN_TEST(test_progress_runs_from_start_to_end);
  RUN_TEST(test_progress_of_an_instant_region_is_whole_once_started);
  RUN_TEST(test_a_right_wipe_shows_the_left_part_first);
  RUN_TEST(test_an_upward_wipe_shows_the_bottom_part_first);
  RUN_TEST(test_a_downward_and_a_left_wipe_mirror_them);
  RUN_TEST(test_a_fade_shows_more_dots_as_it_runs);
  RUN_TEST(test_an_appearing_region_shows_whole_once_due);
  RUN_TEST(test_an_appearing_region_takes_the_timing_effect);
  RUN_TEST(test_turning_a_region_matches_a_portrait_png_drawn_turned);
  return UNITY_END();
}
