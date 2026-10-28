#pragma once
#include <persistence_interface.h>
#include <vector>

struct SavedWifiNetwork {
  int slot;
  String ssid;
  bool is5GHz, enterprise, open;
};
class SavedWifiStore {
public:
  explicit SavedWifiStore(Persistence &storage) : storage(storage) {}
  std::vector<SavedWifiNetwork> list();
  String add(const String &ssid, const String &password);
  String remove(int slot, const String &expectedSsid);

private:
  Persistence &storage;
};
