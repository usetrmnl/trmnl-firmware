#include <panel_vcom.h>
#include <stddef.h>

namespace {
  const size_t BANK_COUNT = 2;
  const size_t BANK_ADDRESSES[BANK_COUNT] = {0x000, 0xC00}; // UC8179 datasheet, OTP address map
  const uint8_t BANK_CHECK_CODE = 0xA5;
  const size_t FIRST_RANGE_VCOM_OFFSET = 0x4E; // 0b, VCOM_DC[6:0]
  const size_t OTP_BYTES_TO_READ = BANK_ADDRESSES[BANK_COUNT - 1] + FIRST_RANGE_VCOM_OFFSET + 1;
} // namespace

int panelVcomFromOtp(uint8_t (*readOtpByte)(void)) {
  uint8_t checkCodes[BANK_COUNT] = {0xFF, 0xFF};
  uint8_t vcoms[BANK_COUNT] = {0xFF, 0xFF};
  for (size_t address = 0; address < OTP_BYTES_TO_READ; address++) {
    const uint8_t value = readOtpByte();
    for (size_t bank = 0; bank < BANK_COUNT; bank++) {
      if (address == BANK_ADDRESSES[bank]) checkCodes[bank] = value;
      if (address == BANK_ADDRESSES[bank] + FIRST_RANGE_VCOM_OFFSET) vcoms[bank] = value;
    }
  }
  for (size_t bank = 0; bank < BANK_COUNT; bank++) {
    if (checkCodes[bank] != BANK_CHECK_CODE) continue;
    return (vcoms[bank] & 0x80) ? -1 : vcoms[bank]; // unprogrammed bits read 1
  }
  return -1;
}
