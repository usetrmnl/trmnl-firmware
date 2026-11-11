#include <panel_vcom.h>
#include <string.h>
#include <unity.h>

static uint8_t otp[0x1000];
static size_t otp_read_count;

static uint8_t read_otp_byte(void) { return otp[otp_read_count++]; }

// A bank's check code and the first bytes of its first temperature range, as read from a TRMNL OG's panel
static void program_bank(size_t bank, uint8_t vcom) {
  otp[bank] = 0xA5;
  const uint8_t range[6] = {0x6f, 0xff, 0x3f, 0x03, 0x00, vcom};
  memcpy(otp + bank + 0x49, range, sizeof(range));
}

void setUp(void) {
  memset(otp, 0xFF, sizeof(otp));
  otp_read_count = 0;
}

void tearDown(void) {}

void test_reads_vcom_from_the_second_bank(void) {
  program_bank(0xC00, 0x1E);
  TEST_ASSERT_EQUAL_INT(0x1E, panelVcomFromOtp(read_otp_byte));
}

void test_prefers_the_first_bank(void) {
  program_bank(0x000, 0x1C);
  program_bank(0xC00, 0x1E);
  TEST_ASSERT_EQUAL_INT(0x1C, panelVcomFromOtp(read_otp_byte));
}

void test_unprogrammed_otp_has_no_vcom(void) { TEST_ASSERT_EQUAL_INT(-1, panelVcomFromOtp(read_otp_byte)); }

void test_bank_without_a_programmed_vcom_has_none(void) {
  program_bank(0xC00, 0xFF);
  TEST_ASSERT_EQUAL_INT(-1, panelVcomFromOtp(read_otp_byte));
}

void test_reads_no_further_than_the_second_bank_vcom(void) {
  panelVcomFromOtp(read_otp_byte);
  TEST_ASSERT_EQUAL_size_t(0xC4F, otp_read_count);
}

int main(void) {
  UNITY_BEGIN();
  RUN_TEST(test_reads_vcom_from_the_second_bank);
  RUN_TEST(test_prefers_the_first_bank);
  RUN_TEST(test_unprogrammed_otp_has_no_vcom);
  RUN_TEST(test_bank_without_a_programmed_vcom_has_none);
  RUN_TEST(test_reads_no_further_than_the_second_bank_vcom);
  return UNITY_END();
}
