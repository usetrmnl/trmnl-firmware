#include <board_config.h>
#include <epd_driver.h>
#include <epd_panel_rev.h>
#include <globals.h>
#include <inttypes.h>
#include <panel_vcom.h>
#include <string.h>

static String panel_vcom_string; // empty unless read_panel_vcom() read an OG panel
static String panel_otp_string;  // empty unless read_panel_vcom() found a programmed OTP bank

#ifdef BB_EPAPER
static bool panel_is_ultrachip = false; // set by get_panel_rev()

static void WriteSPIByte(uint8_t data) {
  for (int i = 0; i < 8; i++) {
    digitalWrite(pDevice->epd_mosi_pin, (data & 0x80) ? 1 : 0);
    digitalWrite(pDevice->epd_sck_pin, 1);
    digitalWrite(pDevice->epd_sck_pin, 0);
    data <<= 1;
  }
}

static uint8_t ReadSPIByte(void) {
  uint8_t u8 = 0;
  for (int i = 0; i < 8; i++) {
    digitalWrite(pDevice->epd_sck_pin, 1);
    digitalWrite(pDevice->epd_sck_pin, 0);
    u8 <<= 1;
    u8 |= digitalRead(pDevice->epd_mosi_pin);
  }
  return u8;
}

// The UC8179 drives each bit while SCK is high; ReadSPIByte() samples after the fall and shifts every byte left by one
static uint8_t ReadSPIByteWhileClockHigh(void) {
  uint8_t u8 = 0;
  for (int i = 0; i < 8; i++) {
    digitalWrite(pDevice->epd_sck_pin, 1);
    u8 = (u8 << 1) | digitalRead(pDevice->epd_mosi_pin);
    digitalWrite(pDevice->epd_sck_pin, 0);
  }
  return u8;
}
/**
 * @brief Function to read the EPD revision to know what temperature profile to use
 * @param none
 * @return 32-bit value read from the panel's revision (REV) command
 */
uint32_t get_panel_rev(void) {
  uint32_t u32 = 0;
  uint8_t u8;
  bool bSolomon = false; // assume the panel has an UltraChip controller

  if (pDevice->epd_mosi_pin == 0 && pDevice->epd_sck_pin == 0) { // pre-defined PCB+display in bb_epaper; pins unknown
    return 0;
  }
    // Initialize the SPI bus in 'bit-bang' mode before running the normal init sequence
    // This will allow us to use MOSI as a bidirectional line for reading data from the panel
  pinMode(pDevice->epd_sck_pin, OUTPUT);
  pinMode(pDevice->epd_cs_pin, OUTPUT);
  digitalWrite(pDevice->epd_cs_pin, 1);
  pinMode(pDevice->epd_rst_pin, OUTPUT);
  pinMode(pDevice->epd_dc_pin, OUTPUT);
  pinMode(pDevice->epd_busy_pin, INPUT);

    // Reset the panel
  digitalWrite(pDevice->epd_rst_pin, 0);
  delay(20);
  digitalWrite(pDevice->epd_rst_pin, 1);
  delay(20);
  if (digitalRead(pDevice->epd_busy_pin) == 0) { // it's a SSD16xx; different command and output
    bSolomon = true;
  }
  panel_is_ultrachip = !bSolomon;
  digitalWrite(pDevice->epd_dc_pin, 0); // command mode
  digitalWrite(pDevice->epd_cs_pin, 0);
  pinMode(pDevice->epd_mosi_pin, OUTPUT);
  if (bSolomon) {
    WriteSPIByte(0x2e); // Read the "Display Option"
  } else {
    WriteSPIByte(0x70); // Revision (REV) command
  }
  digitalWrite(pDevice->epd_dc_pin, 1); // data mode
  pinMode(pDevice->epd_mosi_pin, INPUT);
  for (int i = 0; i < 7; i++) {
    // SSD16xx panels keep the old timing: the 4.26" panel check in display_init() matches a value read with it
    u8 = bSolomon ? ReadSPIByte() : ReadSPIByteWhileClockHigh();
    if (i >= 3) { // first 3 bytes are 0xff
      u32 <<= 8;
      u32 |= u8;
    }
  }
  digitalWrite(pDevice->epd_cs_pin, 1);
  return u32;
} /* get_panel_rev() */

static bool read_otp_summary(uint8_t summary[PANEL_OTP_SUMMARY_SIZE]) {
  digitalWrite(pDevice->epd_dc_pin, 0); // command mode
  digitalWrite(pDevice->epd_cs_pin, 0);
  pinMode(pDevice->epd_mosi_pin, OUTPUT);
  WriteSPIByte(0xA2); // Read OTP (ROTP) command, from address 0
  digitalWrite(pDevice->epd_dc_pin, 1); // data mode
  pinMode(pDevice->epd_mosi_pin, INPUT);
  const bool programmed = panelOtpSummaryFromOtp(ReadSPIByteWhileClockHigh, summary);
  digitalWrite(pDevice->epd_cs_pin, 1);
  return programmed;
}

void read_panel_vcom(uint32_t panel_id) {
  // ROTP 0xA2 is a UC8179 command; sending it to another controller is a risk the header is not worth
  if (strcmp(DEVICE_MODEL, "og") != 0 || !panel_is_ultrachip || panel_id == 0) return;
  int vcom;
  if (preferences.getUInt(PREFERENCES_PANEL_VCOM_ID, 0) == panel_id && preferences.isKey(PREFERENCES_PANEL_VCOM) &&
      preferences.isKey(PREFERENCES_PANEL_OTP)) {
    vcom = preferences.getInt(PREFERENCES_PANEL_VCOM, -1);
    panel_otp_string = preferences.getString(PREFERENCES_PANEL_OTP, "");
  } else {
    uint8_t summary[PANEL_OTP_SUMMARY_SIZE];
    const bool programmed = read_otp_summary(summary);
    vcom = programmed ? panelVcomFromOtpSummary(summary) : -1;
    panel_otp_string = "";
    for (size_t i = 0; programmed && i < PANEL_OTP_SUMMARY_SIZE; i++) {
      char hex[3];
      snprintf(hex, sizeof(hex), "%02x", summary[i]);
      panel_otp_string += hex;
    }
    preferences.putInt(PREFERENCES_PANEL_VCOM, vcom);
    preferences.putString(PREFERENCES_PANEL_OTP, panel_otp_string);
    preferences.putUInt(PREFERENCES_PANEL_VCOM_ID, panel_id);
  }
  panel_vcom_string = String(vcom);
  Log_info("Panel VCOM code = %d, OTP summary = %s", vcom, panel_otp_string.c_str());
}

#endif // BB_EPAPER

String display_panel_rev_string(void) {
  if (panel_rev == 0) {
    return String();
  }
  char sz[9];
  snprintf(sz, sizeof(sz), "%08" PRIx32, panel_rev);
  return String(sz);
} /* display_panel_rev_string() */

String display_panel_vcom_string(void) { return panel_vcom_string; }

String display_panel_otp_string(void) { return panel_otp_string; }
