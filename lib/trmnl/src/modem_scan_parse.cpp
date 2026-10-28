#include "modem_scan_parse.h"

// One entry per line: +CWLAP:(<ecn>,"<ssid>",<rssi>,"<mac>",<channel>[,<numeric fields>...])
// ESP-AT prints the SSID unescaped, so it may contain ( ) " and , itself. Only ecn is parsed from
// the left; everything after the SSID is found from the right, anchored on the quoted MAC (the
// last quoted field).
std::vector<ParsedModemNetwork> parseCwlapResponse(const String &raw) {
  std::vector<ParsedModemNetwork> results;

  int pos = 0;
  while ((pos = raw.indexOf("+CWLAP:", pos)) >= 0) {
    int eol = raw.indexOf('\n', pos);
    if (eol < 0) eol = raw.length();
    String line = raw.substring(pos, eol);
    pos = eol;

    int paren = line.indexOf('(');
    int close = line.lastIndexOf(')');
    if (paren < 0 || close < paren) continue;

    String entry = line.substring(paren + 1, close);

    // field 0: ecn
    int c1 = entry.indexOf(',');
    if (c1 < 0) continue;
    int ecn = entry.substring(0, c1).toInt();

    // field 3: quoted MAC
    int q4 = entry.lastIndexOf('"');
    if (q4 <= 0) continue;
    int q3 = entry.lastIndexOf('"', q4 - 1);

    // field 2: rssi, between the SSID's closing quote and the MAC: ...",<rssi>,"<mac>"
    int c3 = q3 - 1;
    if (c3 <= 0 || entry.charAt(c3) != ',') continue;
    int c2 = entry.lastIndexOf(',', c3 - 1);
    int32_t rssi = entry.substring(c2 + 1, c3).toInt();

    // field 1: quoted SSID
    int q1 = entry.indexOf('"', c1);
    int q2 = c2 - 1;
    if (q1 < 0 || q2 <= q1 || entry.charAt(q2) != '"') continue;
    String ssid = entry.substring(q1 + 1, q2);

    // field 4: channel
    int c4 = q4 + 1;
    int c5 = entry.indexOf(',', c4 + 1);
    int channel = entry.substring(c4 + 1, c5 >= 0 ? c5 : entry.length()).toInt();

    if (ssid.length() == 0 || ssid == "TRMNL") continue;

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
  }

  return results;
}
