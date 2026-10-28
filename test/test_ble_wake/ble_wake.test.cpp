#include <ble_wake_window.h>
#include <pairing.h>
#include <phone_beacon.h>
#include <unity.h>

void setUp() {}
void tearDown() {}

void test_pairing_identity_needs_no_model_discriminator() {
  PairingIdentity identity;
  TEST_ASSERT_TRUE(decodePairingIdentity(
    "{\"v\":1,\"sid\":\"c4b031f0-d5ea-4bda-bb15-95a4fa98f915\",\"pop\":\"0123456789ABCDEF\",\"mac\":\"8B9B4C\"}",
    identity));
  TEST_ASSERT_EQUAL_STRING("BLE:1:C4B031F0D5EA4BDABB1595A4FA98F915:0123456789ABCDEF:8B9B4C",
                           pairingHelpQrFragment(identity).c_str());
  TEST_ASSERT_EQUAL_STRING("TR64P0RZ1NFA9FDBP5CNMKX9HY8N", pairingAdvertisement(identity).c_str());
  PairingIdentity legacy;
  TEST_ASSERT_TRUE(decodePairingIdentity("{\"v\":1,\"sid\":\"c4b031f0-d5ea-4bda-bb15-95a4fa98f915\",\"model\":\"og\","
                                         "\"pop\":\"0123456789ABCDEF\",\"mac\":\"8B9B4C\"}",
                                         legacy));
  TEST_ASSERT_EQUAL_STRING(identity.sessionId.c_str(), legacy.sessionId.c_str());
  TEST_ASSERT_EQUAL_STRING(identity.proof.c_str(), legacy.proof.c_str());
}

void test_no_request_adds_no_sleep_delay() {
  BleWakeWindow window;
  TEST_ASSERT_EQUAL_UINT32(0, window.remaining(100));
}
void test_hold_expires_thirty_seconds_after_last_keepalive() {
  BleWakeWindow window;
  window.hold(500);
  TEST_ASSERT_EQUAL_UINT32(30000, window.remaining(500));
  window.hold(29000);
  TEST_ASSERT_EQUAL_UINT32(30000, window.remaining(29000));
  TEST_ASSERT_EQUAL_UINT32(1, window.remaining(58999));
  TEST_ASSERT_EQUAL_UINT32(0, window.remaining(59000));
  window.hold(59000);
  TEST_ASSERT_EQUAL_UINT32(0, window.remaining(59000));
}
void test_release_prevents_reacquisition_during_sleep_teardown() {
  BleWakeWindow window;
  window.hold(500);
  window.release();
  window.hold(600);
  TEST_ASSERT_EQUAL_UINT32(0, window.remaining(600));
  BleWakeWindow idle;
  idle.release();
  idle.hold(600);
  TEST_ASSERT_EQUAL_UINT32(0, idle.remaining(600));
}

void test_exit_button_requires_release_and_debounced_fresh_press() {
  BleWakeExitButton button;
  TEST_ASSERT_FALSE(button.pressed(0, true));
  TEST_ASSERT_FALSE(button.pressed(100, true));
  TEST_ASSERT_FALSE(button.pressed(110, false));
  TEST_ASSERT_FALSE(button.pressed(120, true));
  TEST_ASSERT_FALSE(button.pressed(140, false)); // bounce resets debounce
  TEST_ASSERT_FALSE(button.pressed(150, true));
  TEST_ASSERT_FALSE(button.pressed(179, true));
  TEST_ASSERT_TRUE(button.pressed(180, true));
}

void test_wake_commands_do_not_require_a_provisioning_attempt() {
  PairingIdentity identity;
  identity.sessionId = "12345678-1234-1234-1234-123456789abc";
  for (const char *op : {"wake_hold", "wake_release"}) {
    PairingRequest request;
    String error;
    String json = String("{\"v\":1,\"sid\":\"") + identity.sessionId + "\",\"attempt\":0,\"op\":\"" + op + "\"}";
    TEST_ASSERT_TRUE(parsePairingRequest(json, identity, request, error));
    TEST_ASSERT_EQUAL_STRING(op, request.op.c_str());
  }
}

void test_phone_beacon_matches_only_target_service_and_rejects_truncation() {
  uint8_t uuid[16];
  TEST_ASSERT_TRUE(phoneBeaconUuid("12345678-1234-1234-1234-123456789abc", uuid));
  const uint8_t packet[] = {17,   7,    0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12, 0x34,
                            0x12, 0x34, 0x12, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12};
  TEST_ASSERT_TRUE(matchesPhoneBeacon(packet, sizeof(packet), uuid));
  TEST_ASSERT_FALSE(matchesPhoneBeacon(packet, sizeof(packet) - 1, uuid));
  uuid[0] ^= 1;
  TEST_ASSERT_FALSE(matchesPhoneBeacon(packet, sizeof(packet), uuid));
  TEST_ASSERT_FALSE(phoneBeaconUuid("invalid", uuid));
}

bool parseWifi(const char *fields, PairingRequest &r, String &error) {
  PairingIdentity id;
  id.sessionId = "demo";
  return parsePairingRequest(String("{\"v\":1,\"sid\":\"demo\",") + fields + "}", id, r, error);
}
void test_wifi_management_parser_accepts_only_bounded_typed_requests() {
  PairingRequest r;
  String error;
  TEST_ASSERT_TRUE(parseWifi("\"op\":\"wifi_list\",\"attempt\":0", r, error));
  TEST_ASSERT_TRUE(parseWifi("\"op\":\"wifi_add\",\"attempt\":0,\"ssid\":\"Cafe\",\"password\":\"\"", r, error));
  TEST_ASSERT_TRUE(parseWifi("\"op\":\"wifi_remove\",\"attempt\":0,\"ssid\":\"Cafe\",\"slot\":4", r, error));
  for (const char *fields :
       {"\"op\":\"wifi_list\",\"attempt\":1",
        "\"op\":\"wifi_add\",\"attempt\":0,\"ssid\":\"Cafe\",\"password\":\"password\",\"enterprise\":\"false\"",
        "\"op\":\"wifi_add\",\"attempt\":0,\"ssid\":\"Cafe\",\"password\":\"password\",\"security\":\"open\"",
        "\"op\":\"wifi_add\",\"attempt\":0,\"ssid\":\"Cafe\",\"password\":42",
        "\"op\":\"wifi_add\",\"attempt\":0,\"ssid\":\"Cafe\",\"password\":\"short\"",
        "\"op\":\"wifi_add\",\"attempt\":0,\"ssid\":\"Cafe\",\"password\":\"password\",\"band\":\"5GHz\"",
        "\"op\":\"wifi_remove\",\"attempt\":0,\"ssid\":\"Cafe\",\"slot\":5",
        "\"op\":\"wifi_remove\",\"attempt\":0,\"ssid\":\"Cafe\",\"slot\":-1",
        "\"op\":\"wifi_remove\",\"attempt\":0,\"ssid\":\"Cafe\",\"slot\":true",
        "\"op\":\"wifi_remove\",\"attempt\":0,\"ssid\":\"Cafe\",\"slot\":1.5",
        "\"op\":\"wifi_remove\",\"attempt\":0,\"slot\":1"})
    TEST_ASSERT_FALSE(parseWifi(fields, r, error));
}

void test_json_strings_with_embedded_nul_are_rejected_before_conversion() {
  PairingRequest r;
  String error;
  for (const char *fields :
       {"\"op\":\"configure\",\"attempt\":1,\"ssid\":\"Cafe\\u0000Other\",\"password\":\"password\"",
        "\"op\":\"configure\",\"attempt\":1,\"ssid\":\"Cafe\",\"password\":\"password\\u0000suffix\"",
        "\"op\":\"wifi_add\",\"attempt\":0,\"ssid\":\"Cafe\\u0000Other\",\"password\":\"password\"",
        "\"op\":\"wifi_add\",\"attempt\":0,\"ssid\":\"Cafe\",\"password\":\"password\\u0000suffix\"",
        "\"op\":\"wifi_remove\",\"attempt\":0,\"ssid\":\"Cafe\\u0000Other\",\"slot\":0",
        "\"op\":\"status\\u0000Other\",\"attempt\":0",
        "\"op\":\"wifi_add\",\"attempt\":0,\"ssid\":\"Cafe\",\"password\":\"password\",\"band\":\"2.4GHz\\u0000Other\"",
        "\"op\":\"wifi_add\",\"attempt\":0,\"ssid\":\"Cafe\",\"password\":\"password\",\"security\":"
        "\"personal\\u0000Other\""})
    TEST_ASSERT_FALSE(parseWifi(fields, r, error));
  PairingIdentity id;
  id.sessionId = "demo";
  TEST_ASSERT_FALSE(
    parsePairingRequest("{\"v\":1,\"sid\":\"demo\\u0000Other\",\"op\":\"status\",\"attempt\":0}", id, r, error));
  TEST_ASSERT_TRUE(
    parseWifi("\"op\":\"configure\",\"attempt\":1,\"ssid\":\"Caf\\u00e9\",\"password\":\"password\"", r, error));
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_json_strings_with_embedded_nul_are_rejected_before_conversion);
  RUN_TEST(test_exit_button_requires_release_and_debounced_fresh_press);
  RUN_TEST(test_wifi_management_parser_accepts_only_bounded_typed_requests);
  RUN_TEST(test_phone_beacon_matches_only_target_service_and_rejects_truncation);
  RUN_TEST(test_wake_commands_do_not_require_a_provisioning_attempt);
  RUN_TEST(test_pairing_identity_needs_no_model_discriminator);
  RUN_TEST(test_no_request_adds_no_sleep_delay);
  RUN_TEST(test_hold_expires_thirty_seconds_after_last_keepalive);
  RUN_TEST(test_release_prevents_reacquisition_during_sleep_teardown);
  return UNITY_END();
}
