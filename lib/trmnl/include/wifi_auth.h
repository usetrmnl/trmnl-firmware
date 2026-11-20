#pragma once

// ESP-AT CWLAP encryption codes match ESP-IDF wifi_auth_mode_t values.
// Use the numeric codes so this also builds with older SDKs that do not
// declare the WPA3 Enterprise and transition-mode enum names.
inline bool isEnterpriseWifiAuthMode(int authMode) {
  switch (authMode) {
  case 5:   // WPA2_ENTERPRISE
  case 10:  // WPA3_ENT_192
  case 14:  // WPA3_ENTERPRISE
  case 15:  // WPA2_WPA3_ENTERPRISE
    return true;
  default:
    return false;
  }
}
