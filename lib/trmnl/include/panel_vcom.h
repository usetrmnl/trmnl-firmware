#pragma once

#include <stddef.h>
#include <stdint.h>

const size_t PANEL_OTP_SUMMARY_SIZE = 30; // bank number, 11 temperature boundaries, 12 VCOMs, 6 version bytes

// Reads a UC8179's OTP from address 0, one byte per call, through the first bank with a check code, and copies that
// bank's number, temperature boundaries, each temperature range's VCOM_DC code and versions; false when no bank has one
bool panelOtpSummaryFromOtp(uint8_t (*readOtpByte)(void), uint8_t summary[PANEL_OTP_SUMMARY_SIZE]);

// The first temperature range's VCOM_DC code (VDCS 0x82: -0.10 V - 0.05 V per step); -1 when not programmed
int panelVcomFromOtpSummary(const uint8_t summary[PANEL_OTP_SUMMARY_SIZE]);
