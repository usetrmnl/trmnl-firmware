#include "modem_scan_parse.h"

std::vector<ParsedModemNetwork> parseCwlapResponse(const String &raw) {
  std::vector<ParsedModemNetwork> results;

  int pos = 0;
  while ((pos = raw.indexOf("+CWLAP:", pos)) >= 0) {
    int paren = raw.indexOf('(', pos);
    int close = raw.indexOf(')', paren);
    if (paren < 0 || close < 0) {
      pos++;
      continue;
    }

    String entry = raw.substring(paren + 1, close);

    // field 0: ecn
    int c1 = entry.indexOf(',');
    int ecn = entry.substring(0, c1).toInt();

    // field 1: quoted SSID
    int q1 = entry.indexOf('"', c1);
    int q2 = entry.indexOf('"', q1 + 1);
    String ssid = entry.substring(q1 + 1, q2);

    // field 2: rssi
    int c2 = entry.indexOf(',', q2 + 1);
    int c3 = entry.indexOf(',', c2 + 1);
    int32_t rssi = entry.substring(c2 + 1, c3).toInt();

    // skip quoted MAC, find channel
    int q3 = entry.indexOf('"', c3);
    int q4 = entry.indexOf('"', q3 + 1);
    int c4 = entry.indexOf(',', q4 + 1);
    int c5 = entry.indexOf(',', c4 + 1);
    int channel = entry.substring(c4 + 1, c5 >= 0 ? c5 : entry.length()).toInt();

    if (ssid.length() == 0 || ssid == "TRMNL") {
      pos = close;
      continue;
    }

    bool is5GHz = channel >= 36;

    // Dedup: keep highest RSSI per SSID within the same band.
    bool found = false;
    for (auto &net : results) {
      if (net.ssid == ssid && net.is5GHz == is5GHz) {
        if (rssi > net.rssi) net.rssi = rssi;
        found = true;
        break;
      }
    }
    if (!found) {
      results.push_back({ssid, rssi, ecn == 0, is5GHz, ecn == 5});
    }
    pos = close;
  }

  return results;
}
