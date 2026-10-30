#include <pairing.h>
#include <saved_wifi.h>

namespace {
  String key(int slot, const char *suffix) { return String("wifi_") + slot + suffix; }
// SSID is the slot's visibility marker. Remove it first; publish it last.
  const char *suffixes[] = {"_ssid", "_pswd", "_5g",  "_ent", "_username", "_identity",
                            "_usip", "_sip",  "_sgw", "_ssn", "_dns1",     "_dns2"};
  bool clearSlot(Persistence &p, int slot) {
    for (const char *suffix : suffixes) {
      const String k = key(slot, suffix);
      if (p.recordExists(k.c_str()) && (!p.remove(k.c_str()) || p.recordExists(k.c_str()))) return false;
    }
    return true;
  }
  bool writeChecked(Persistence &p, const String &k, const String &value) {
    const size_t written = p.writeString(k.c_str(), value.c_str());
    return (value.length() == 0 || written != 0) && p.recordExists(k.c_str()) && p.readString(k.c_str(), "") == value;
  }
} // namespace
std::vector<SavedWifiNetwork> SavedWifiStore::list() {
  std::vector<SavedWifiNetwork> result;
  for (int i = 0; i < 5; ++i) {
    String ssid = storage.readString(key(i, "_ssid").c_str(), "");
    if (ssid.length() == 0) continue;
    result.push_back(
      {i, ssid, storage.readBool(key(i, "_5g").c_str(), false), storage.readBool(key(i, "_ent").c_str(), false),
       storage.readString(key(i, "_pswd").c_str(), "").length() == 0});
  }
  return result;
}
String SavedWifiStore::add(const String &ssid, const String &password) {
  if (!validPairingCredentials(ssid, password)) return "invalid_credentials";
  bool occupied[5] = {};
  for (const auto &network : list()) {
    if (network.ssid == ssid) return "network_exists";
    occupied[network.slot] = true;
  }
  int slot = 0;
  while (slot < 5 && occupied[slot])
    ++slot;
  if (slot == 5) return "storage_full";
  // Clear only this unused slot, including any settings left by an interrupted write.
  if (!clearSlot(storage, slot) || !writeChecked(storage, key(slot, "_pswd"), password) ||
      !writeChecked(storage, key(slot, "_ssid"), ssid))
    return "storage_failed";
  return "";
}
String SavedWifiStore::remove(int slot, const String &expectedSsid) {
  if (slot < 0 || slot >= 5 || expectedSsid.length() == 0 ||
      storage.readString(key(slot, "_ssid").c_str(), "") != expectedSsid)
    return "stale_network";
  return clearSlot(storage, slot) ? "" : "storage_failed";
}
