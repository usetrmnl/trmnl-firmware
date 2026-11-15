#include <panel_vcom.h>
#include <string.h>
#include <unity.h>

static uint8_t otp[0x1800];
static size_t otp_read_count;
static uint8_t summary[PANEL_OTP_SUMMARY_SIZE];

static uint8_t read_otp_byte(void) { return otp[otp_read_count++]; }

// The desk OG's bank (panel feffff0c): 7 temperature ranges, all at VCOM 0x1E
static void program_desk_og_bank(size_t bank) {
  otp[bank] = 0xA5;
  const uint8_t boundaries[11] = {0x00, 0x05, 0x0A, 0x0F, 0x14, 0x23, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF};
  memcpy(otp + bank + 1, boundaries, sizeof(boundaries));
  for (size_t range = 0; range < 7; range++)
    otp[bank + 0x49 + range * 0xF7 + 5] = 0x1E;
  otp[bank + 0xBE0] = 0xFE; // LUT version
}

void setUp(void) {
  memset(otp, 0xFF, sizeof(otp));
  memset(summary, 0, sizeof(summary));
  otp_read_count = 0;
}

void tearDown(void) {}

void test_summary_lists_bank_boundaries_vcoms_and_versions(void) {
  otp[0xC00] = 0xA5;
  for (size_t i = 0; i < 11; i++)
    otp[0xC01 + i] = i;
  for (size_t range = 0; range < 12; range++)
    otp[0xC00 + 0x49 + range * 0xF7 + 5] = 0x20 + range;
  for (size_t i = 0; i < 6; i++)
    otp[0xC00 + 0xBDD + i] = 0x30 + i;
  const uint8_t expected[PANEL_OTP_SUMMARY_SIZE] = {1,    0,    1,    2,    3,    4,    5,    6,    7,    8,
                                                    9,    10,   0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27,
                                                    0x28, 0x29, 0x2A, 0x2B, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35};
  TEST_ASSERT_TRUE(panelOtpSummaryFromOtp(read_otp_byte, summary));
  TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, summary, PANEL_OTP_SUMMARY_SIZE);
}

void test_ignores_data_in_an_unchecked_first_bank(void) {
  const uint8_t leftovers[16] = {0xa5, 0x0c, 0x64, 0x3b, 0x00, 0x89, 0xc2, 0x40,
                                 0x91, 0x53, 0x18, 0x00, 0x28, 0xf0, 0x00, 0x72}; // the desk OG's bank 0
  memcpy(otp + 0xBF0, leftovers, sizeof(leftovers));
  for (size_t i = 1; i < 0xBE3; i++)
    otp[i] = 0x11;
  program_desk_og_bank(0xC00);
  TEST_ASSERT_TRUE(panelOtpSummaryFromOtp(read_otp_byte, summary));
  TEST_ASSERT_EQUAL_UINT8(1, summary[0]);
  TEST_ASSERT_EQUAL_UINT8(0x05, summary[2]);
  TEST_ASSERT_EQUAL_UINT8(0xFF, summary[19]);
  TEST_ASSERT_EQUAL_INT(0x1E, panelVcomFromOtpSummary(summary));
}

void test_prefers_the_first_bank_and_stops_reading_after_it(void) {
  program_desk_og_bank(0x000);
  otp[0x4E] = 0x1C;
  program_desk_og_bank(0xC00);
  TEST_ASSERT_TRUE(panelOtpSummaryFromOtp(read_otp_byte, summary));
  TEST_ASSERT_EQUAL_UINT8(0, summary[0]);
  TEST_ASSERT_EQUAL_INT(0x1C, panelVcomFromOtpSummary(summary));
  TEST_ASSERT_EQUAL_size_t(0xBE3, otp_read_count);
}

void test_unprogrammed_otp_has_no_summary(void) {
  TEST_ASSERT_FALSE(panelOtpSummaryFromOtp(read_otp_byte, summary));
  TEST_ASSERT_EQUAL_size_t(0xC00 + 0xBE3, otp_read_count);
}

void test_vcom_is_none_when_the_first_range_is_unprogrammed(void) {
  otp[0xC00] = 0xA5;
  TEST_ASSERT_TRUE(panelOtpSummaryFromOtp(read_otp_byte, summary));
  TEST_ASSERT_EQUAL_INT(-1, panelVcomFromOtpSummary(summary));
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_summary_lists_bank_boundaries_vcoms_and_versions);
  RUN_TEST(test_ignores_data_in_an_unchecked_first_bank);
  RUN_TEST(test_prefers_the_first_bank_and_stops_reading_after_it);
  RUN_TEST(test_unprogrammed_otp_has_no_summary);
  RUN_TEST(test_vcom_is_none_when_the_first_range_is_unprogrammed);
  return UNITY_END();
}
