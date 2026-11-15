#include <panel_vcom.h>

namespace {
  const size_t BANK_ADDRESSES[] = {0x000, 0xC00}; // UC8179 datasheet, OTP address map
  const uint8_t BANK_CHECK_CODE = 0xA5;
  const size_t BOUNDARY_COUNT = 11; // TB0-TB10, from bank offset 1
  const size_t RANGE_COUNT = 12;
  const size_t FIRST_RANGE_VCOM_OFFSET = 0x4E; // 0b, VCOM_DC[6:0]
  const size_t RANGE_SIZE = 0xF7;
  const size_t VERSIONS_OFFSET = 0xBDD; // production version[23:0], then LUT version[23:0]
  const size_t VERSION_SIZE = 6;
  const size_t FIRST_VCOM_INDEX = 1 + BOUNDARY_COUNT;
  const size_t FIRST_VERSION_INDEX = FIRST_VCOM_INDEX + RANGE_COUNT;

  // Where a byte at this bank offset goes in the summary; -1 when the summary leaves it out
  int summaryIndex(size_t offset) {
    if (offset >= 1 && offset <= BOUNDARY_COUNT) return offset;
    if (offset >= FIRST_RANGE_VCOM_OFFSET && offset < FIRST_RANGE_VCOM_OFFSET + RANGE_COUNT * RANGE_SIZE &&
        (offset - FIRST_RANGE_VCOM_OFFSET) % RANGE_SIZE == 0)
      return FIRST_VCOM_INDEX + (offset - FIRST_RANGE_VCOM_OFFSET) / RANGE_SIZE;
    if (offset >= VERSIONS_OFFSET && offset < VERSIONS_OFFSET + VERSION_SIZE)
      return FIRST_VERSION_INDEX + offset - VERSIONS_OFFSET;
    return -1;
  }
} // namespace

bool panelOtpSummaryFromOtp(uint8_t (*readOtpByte)(void), uint8_t summary[PANEL_OTP_SUMMARY_SIZE]) {
  size_t address = 0;
  for (size_t bankNumber = 0; bankNumber < 2; bankNumber++) {
    const size_t bank = BANK_ADDRESSES[bankNumber];
    for (; address < bank; address++)
      readOtpByte();
    const bool checked = readOtpByte() == BANK_CHECK_CODE;
    for (address++; address < bank + VERSIONS_OFFSET + VERSION_SIZE; address++) {
      const uint8_t value = readOtpByte();
      const int index = summaryIndex(address - bank);
      if (index >= 0) summary[index] = value;
    }
    summary[0] = bankNumber;
    if (checked) return true;
  }
  return false;
}

int panelVcomFromOtpSummary(const uint8_t summary[PANEL_OTP_SUMMARY_SIZE]) {
  const uint8_t vcom = summary[FIRST_VCOM_INDEX];
  return (vcom & 0x80) ? -1 : vcom; // unprogrammed bits read 1
}
