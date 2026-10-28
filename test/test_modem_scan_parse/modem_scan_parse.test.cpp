#include <modem_scan_parse.h>
#include <unity.h>

// One +CWLAP entry: (ecn,"ssid",rssi,"mac",channel)
static const char *SAMPLE = "AT+CWLAP\r\n"
                            "+CWLAP:(5,\"CorpNet\",-55,\"aa:aa:aa:aa:aa:aa\",6)\r\n"     // WPA2-Enterprise, 2.4GHz
                            "+CWLAP:(0,\"CafeGuest\",-70,\"bb:bb:bb:bb:bb:bb\",11)\r\n"  // open, 2.4GHz
                            "+CWLAP:(3,\"HomeNet\",-40,\"cc:cc:cc:cc:cc:cc\",1)\r\n"     // WPA2-PSK, 2.4GHz
                            "+CWLAP:(5,\"CorpNet5\",-60,\"dd:dd:dd:dd:dd:dd\",40)\r\n"   // WPA2-Enterprise, 5GHz
                            "OK\r\n";

static ParsedModemNetwork *find(std::vector<ParsedModemNetwork> &v, const char *ssid, bool is5GHz) {
  for (auto &n : v)
    if (n.ssid == ssid && n.is5GHz == is5GHz) return &n;
  return nullptr;
}

void test_detects_wpa2_enterprise(void) {
  auto nets = parseCwlapResponse(SAMPLE);
  ParsedModemNetwork *n = find(nets, "CorpNet", false);
  TEST_ASSERT_NOT_NULL(n);
  TEST_ASSERT_TRUE(n->enterprise);
  TEST_ASSERT_FALSE(n->open);
  TEST_ASSERT_EQUAL_INT32(-55, n->rssi);
}

void test_open_and_personal_not_enterprise(void) {
  auto nets = parseCwlapResponse(SAMPLE);
  ParsedModemNetwork *open = find(nets, "CafeGuest", false);
  ParsedModemNetwork *psk = find(nets, "HomeNet", false);
  TEST_ASSERT_NOT_NULL(open);
  TEST_ASSERT_TRUE(open->open);
  TEST_ASSERT_FALSE(open->enterprise);
  TEST_ASSERT_NOT_NULL(psk);
  TEST_ASSERT_FALSE(psk->open);
  TEST_ASSERT_FALSE(psk->enterprise);
}

void test_band_from_channel(void) {
  auto nets = parseCwlapResponse(SAMPLE);
  TEST_ASSERT_NOT_NULL(find(nets, "CorpNet5", true));  // channel 40 -> 5GHz
  TEST_ASSERT_NULL(find(nets, "CorpNet5", false));
}

void test_dedup_keeps_highest_rssi_same_band(void) {
  String raw = "+CWLAP:(3,\"Dup\",-80,\"aa:aa:aa:aa:aa:aa\",6)\r\n"
               "+CWLAP:(3,\"Dup\",-50,\"bb:bb:bb:bb:bb:bb\",6)\r\nOK\r\n";
  auto nets = parseCwlapResponse(raw);
  int count = 0;
  for (auto &n : nets)
    if (n.ssid == "Dup") count++;
  TEST_ASSERT_EQUAL_INT(1, count);
  TEST_ASSERT_EQUAL_INT32(-50, find(nets, "Dup", false)->rssi);
}

void test_skips_trmnl_and_malformed(void) {
  String raw = "+CWLAP:(0,\"TRMNL\",-30,\"aa:aa:aa:aa:aa:aa\",6)\r\n"  // self AP, skipped
               "+CWLAP:(3,\"Broken\",-40,\"bb:bb\r\n"                    // no ')', skipped
               "OK\r\n";
  auto nets = parseCwlapResponse(raw);
  TEST_ASSERT_EQUAL_INT(0, (int)nets.size());
}

void test_ssid_with_parentheses(void) {
  String raw = "+CWLAP:(3,\"Foo (Bar)\",-50,\"aa:aa:aa:aa:aa:aa\",36)\r\n"
               "+CWLAP:(3,\"(x)\",-60,\"bb:bb:bb:bb:bb:bb\",6)\r\nOK\r\n";
  auto nets = parseCwlapResponse(raw);
  TEST_ASSERT_EQUAL_INT(2, (int)nets.size());
  ParsedModemNetwork *n = find(nets, "Foo (Bar)", true);
  TEST_ASSERT_NOT_NULL(n);
  TEST_ASSERT_EQUAL_INT32(-50, n->rssi);
  TEST_ASSERT_FALSE(n->open);
  n = find(nets, "(x)", false);
  TEST_ASSERT_NOT_NULL(n);
  TEST_ASSERT_EQUAL_INT32(-60, n->rssi);
}

void test_ssid_with_quotes_and_commas(void) {
  String raw = "+CWLAP:(0,\"my \"net\",-5,\"x\",-55,\"aa:aa:aa:aa:aa:aa\",40)\r\nOK\r\n";
  auto nets = parseCwlapResponse(raw);
  ParsedModemNetwork *n = find(nets, "my \"net\",-5,\"x", true);
  TEST_ASSERT_NOT_NULL(n);
  TEST_ASSERT_EQUAL_INT32(-55, n->rssi);
  TEST_ASSERT_TRUE(n->open);
}

void test_full_cwlapopt_fields(void) {
  // ESP-AT's default AT+CWLAPOPT mask adds freq_offset, freqcal_val, ciphers, bgn and wps
  String raw = "+CWLAP:(3,\"Home (5G)\",-48,\"aa:aa:aa:aa:aa:aa\",149,-1,-1,4,4,7,0)\r\nOK\r\n";
  auto nets = parseCwlapResponse(raw);
  ParsedModemNetwork *n = find(nets, "Home (5G)", true);
  TEST_ASSERT_NOT_NULL(n);
  TEST_ASSERT_EQUAL_INT32(-48, n->rssi);
}

void test_skips_empty_ssid(void) {
  String raw = "+CWLAP:(3,\"\",-40,\"aa:aa:aa:aa:aa:aa\",6)\r\nOK\r\n";
  TEST_ASSERT_EQUAL_INT(0, (int)parseCwlapResponse(raw).size());
}

void setUp(void) {}
void tearDown(void) {}

void process() {
  UNITY_BEGIN();
  RUN_TEST(test_detects_wpa2_enterprise);
  RUN_TEST(test_open_and_personal_not_enterprise);
  RUN_TEST(test_band_from_channel);
  RUN_TEST(test_dedup_keeps_highest_rssi_same_band);
  RUN_TEST(test_skips_trmnl_and_malformed);
  RUN_TEST(test_ssid_with_parentheses);
  RUN_TEST(test_ssid_with_quotes_and_commas);
  RUN_TEST(test_full_cwlapopt_fields);
  RUN_TEST(test_skips_empty_ssid);
  UNITY_END();
}

int main(int argc, char **argv) {
  process();
  return 0;
}
