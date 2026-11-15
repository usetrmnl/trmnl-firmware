#pragma once

#include <stdint.h>

// Reads a UC8179's OTP from address 0 through bank 1's first temperature range, one byte per call, and
// returns the VCOM_DC code (VDCS 0x82: -0.10 V - 0.05 V per step) the factory programmed; -1 when none
int panelVcomFromOtp(uint8_t (*readOtpByte)(void));
