#pragma once

#include <Arduino.h>
#include <vector>

struct ParsedModemNetwork {
  String ssid;
  int32_t rssi;
  bool open;        // ecn == 0 (unencrypted)
  bool is5GHz;      // channel >= 36
  bool enterprise;  // ecn == 5 (WPA2-Enterprise)
};

// Parse a raw `AT+CWLAP` response into deduplicated networks.
// Dedup keeps the highest RSSI per (SSID, band). Empty SSIDs and the device's
// own "TRMNL" AP are skipped; malformed entries (no closing ')' on the line) are skipped.
// SSIDs may contain any characters, including ( ) " and , (ESP-AT does not escape them).
std::vector<ParsedModemNetwork> parseCwlapResponse(const String &raw);
