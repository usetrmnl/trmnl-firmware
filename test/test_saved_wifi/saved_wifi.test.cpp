#include <memory_persistence.h>
#include <saved_wifi.h>
#include <unity.h>
void setUp() {}
void tearDown() {}
void test_add_is_persistent_and_preserves_enterprise_and_static_slots() {
  MemoryPersistence p;
  p.writeString("wifi_2_ssid", "Office");
  p.writeString("wifi_2_pswd", "secret-office");
  p.writeBool("wifi_2_ent", true);
  p.writeBool("wifi_2_5g", true);
  p.writeString("wifi_2_identity", "employee");
  p.writeString("wifi_2_sip", "192.168.1.42");
  SavedWifiStore store(p);
  TEST_ASSERT_EQUAL_STRING("", store.add("Cafe", "").c_str());
  SavedWifiStore reloaded(p);
  auto networks = reloaded.list();
  TEST_ASSERT_EQUAL(2, networks.size());
  TEST_ASSERT_EQUAL(0, networks[0].slot);
  TEST_ASSERT_TRUE(networks[0].open);
  TEST_ASSERT_TRUE(networks[1].enterprise);
  TEST_ASSERT_TRUE(networks[1].is5GHz);
  TEST_ASSERT_EQUAL_STRING("employee", p.readString("wifi_2_identity", "").c_str());
  TEST_ASSERT_EQUAL_STRING("192.168.1.42", p.readString("wifi_2_sip", "").c_str());
  TEST_ASSERT_EQUAL_STRING("secret-office", p.readString("wifi_2_pswd", "").c_str());
}
void test_duplicates_full_storage_and_invalid_credentials_do_not_replace_networks() {
  MemoryPersistence p;
  SavedWifiStore store(p);
  TEST_ASSERT_EQUAL_STRING("invalid_credentials", store.add("", "password").c_str());
  TEST_ASSERT_EQUAL_STRING("", store.add("Home", "password").c_str());
  TEST_ASSERT_EQUAL_STRING("network_exists", store.add("Home", "different").c_str());
  TEST_ASSERT_EQUAL_STRING("password", p.readString("wifi_0_pswd", "").c_str());
  for (int i = 1; i < 5; ++i)
    TEST_ASSERT_EQUAL_STRING("", store.add(String("Network") + i, "").c_str());
  TEST_ASSERT_EQUAL_STRING("storage_full", store.add("Sixth", "").c_str());
  TEST_ASSERT_EQUAL(5, store.list().size());
}
void test_remove_guards_stale_slot_and_preserves_sparse_networks() {
  MemoryPersistence p;
  SavedWifiStore store(p);
  store.add("First", "password");
  store.add("Second", "password2");
  TEST_ASSERT_EQUAL_STRING("stale_network", store.remove(0, "Second").c_str());
  TEST_ASSERT_EQUAL_STRING("", store.remove(0, "First").c_str());
  TEST_ASSERT_EQUAL_STRING("stale_network", store.remove(0, "First").c_str());
  TEST_ASSERT_EQUAL(1, store.list().size());
  TEST_ASSERT_EQUAL(1, store.list()[0].slot);
  TEST_ASSERT_EQUAL_STRING("Second", store.list()[0].ssid.c_str());
  TEST_ASSERT_FALSE(p.recordExists("wifi_0_pswd"));
  TEST_ASSERT_EQUAL_STRING("", store.add("Replacement", "").c_str());
  TEST_ASSERT_EQUAL_STRING("stale_network", store.remove(0, "First").c_str());
  TEST_ASSERT_EQUAL(2, store.list().size());
}
class FailedStorage : public MemoryPersistence {
public:
  size_t writeString(const char *, const char *value) override { return strlen(value); }
};
void test_add_requires_readback_before_success() {
  FailedStorage p;
  SavedWifiStore store(p);
  TEST_ASSERT_EQUAL_STRING("storage_failed", store.add("Home", "password").c_str());
  TEST_ASSERT_EQUAL(0, store.list().size());
}
class FailedRemovalStorage : public MemoryPersistence {
public:
  String failedKey;
  bool remove(const char *key) override { return failedKey == key ? false : MemoryPersistence::remove(key); }
};
void test_remove_cleanup_failure_hides_deleted_network_and_preserves_other_slot() {
  FailedRemovalStorage p;
  SavedWifiStore store(p);
  TEST_ASSERT_EQUAL_STRING("", store.add("First", "password").c_str());
  TEST_ASSERT_EQUAL_STRING("", store.add("Second", "password2").c_str());
  p.writeBool("wifi_1_ent", true);
  p.writeString("wifi_1_identity", "employee");
  p.writeString("wifi_1_sip", "192.168.1.42");
  p.failedKey = "wifi_0_pswd";
  TEST_ASSERT_EQUAL_STRING("storage_failed", store.remove(0, "First").c_str());
  SavedWifiStore reloaded(p);
  auto networks = reloaded.list();
  TEST_ASSERT_EQUAL(1, networks.size());
  TEST_ASSERT_EQUAL(1, networks[0].slot);
  TEST_ASSERT_EQUAL_STRING("Second", networks[0].ssid.c_str());
  TEST_ASSERT_TRUE(networks[0].enterprise);
  TEST_ASSERT_FALSE(networks[0].open);
  TEST_ASSERT_EQUAL_STRING("password2", p.readString("wifi_1_pswd", "").c_str());
  TEST_ASSERT_EQUAL_STRING("employee", p.readString("wifi_1_identity", "").c_str());
  TEST_ASSERT_EQUAL_STRING("192.168.1.42", p.readString("wifi_1_sip", "").c_str());
  TEST_ASSERT_FALSE(p.recordExists("wifi_0_ssid"));
  TEST_ASSERT_EQUAL_STRING("password", p.readString("wifi_0_pswd", "").c_str());
}
void test_remove_marker_failure_keeps_network_visible_and_credentials_intact() {
  FailedRemovalStorage p;
  SavedWifiStore store(p);
  TEST_ASSERT_EQUAL_STRING("", store.add("First", "password").c_str());
  TEST_ASSERT_EQUAL_STRING("", store.add("Second", "password2").c_str());
  p.failedKey = "wifi_0_ssid";
  TEST_ASSERT_EQUAL_STRING("storage_failed", store.remove(0, "First").c_str());
  SavedWifiStore reloaded(p);
  auto networks = reloaded.list();
  TEST_ASSERT_EQUAL(2, networks.size());
  TEST_ASSERT_EQUAL(0, networks[0].slot);
  TEST_ASSERT_EQUAL_STRING("First", networks[0].ssid.c_str());
  TEST_ASSERT_FALSE(networks[0].open);
  TEST_ASSERT_EQUAL_STRING("password", p.readString("wifi_0_pswd", "").c_str());
  TEST_ASSERT_EQUAL(1, networks[1].slot);
  TEST_ASSERT_EQUAL_STRING("Second", networks[1].ssid.c_str());
  TEST_ASSERT_EQUAL_STRING("password2", p.readString("wifi_1_pswd", "").c_str());
}
int main() {
  UNITY_BEGIN();
  RUN_TEST(test_remove_cleanup_failure_hides_deleted_network_and_preserves_other_slot);
  RUN_TEST(test_remove_marker_failure_keeps_network_visible_and_credentials_intact);
  RUN_TEST(test_add_is_persistent_and_preserves_enterprise_and_static_slots);
  RUN_TEST(test_duplicates_full_storage_and_invalid_credentials_do_not_replace_networks);
  RUN_TEST(test_remove_guards_stale_slot_and_preserves_sparse_networks);
  RUN_TEST(test_add_requires_readback_before_success);
  return UNITY_END();
}
