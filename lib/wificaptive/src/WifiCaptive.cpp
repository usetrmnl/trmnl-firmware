#include "WifiCaptive.h"

#include <WiFi.h>
#include <trmnl_log.h>
#include <utility>
#define _NO_DEV_CONFIG_
#define UWORD uint16_t

#include "../../../include/display.h"
#include "../../../include/pairing_runtime.h"
#include "../../../include/power.h"
#include "WebServer.h"
#include "connect.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "wifi-helpers.h"
void goToSleep(void);
void saveShipmentStarted(void);
void showMessageWithLogo(MSG message_type);
// 15 minute timeout to prevent dead batteries
#define PORTAL_TIMEOUT (15 * 60 * 1000)

void WifiCaptive::setUpDNSServer(DNSServer &dnsServer, const IPAddress &localIP) {
  dnsServer.setTTL(3600);
  dnsServer.start(53, "*", localIP);
}

String WifiCaptive::getAPSSID() {
  uint64_t mac = ESP.getEfuseMac();
  char macSuffix[7];
  snprintf(macSuffix, sizeof(macSuffix), "%02X%02X%02X", (uint8_t)(mac >> 24), (uint8_t)(mac >> 32),
           (uint8_t)(mac >> 40));
  return String(WIFI_SSID) + "-" + String(macSuffix);
}

bool WifiCaptive::startPortal() {
  _pendingSubmission = false;
  _acceptSubmissions = true;
  _resetRequested = false;
  _acceptResets = true;
  _dnsServer = new DNSServer();
  _server = new AsyncWebServer(80);
  _connectionState = WifiConnectionState::Idle;

    // Set the WiFi mode to access point and station
  WiFi.mode(WIFI_MODE_AP);

    // Define the subnet mask for the WiFi network
  const IPAddress subnetMask(255, 255, 255, 0);
  const IPAddress localIP(4, 3, 2, 1);
  const IPAddress gatewayIP(4, 3, 2, 1);

  WiFi.disconnect();
  delay(50);

    // Configure the soft access point with a specific IP and subnet mask
  WiFi.softAPConfig(localIP, gatewayIP, subnetMask);
  delay(50);

  String SSID = getAPSSID();

    // Start the soft access point with the given ssid, password, channel, max number of clients
  WiFi.softAP(SSID.c_str(), WIFI_PASSWORD, WIFI_CHANNEL, 0, MAX_CLIENTS);
  delay(50);

    // Disable AMPDU RX on the ESP32 WiFi to fix a bug on Android
  esp_wifi_stop();
  esp_wifi_deinit();
  wifi_init_config_t my_config = WIFI_INIT_CONFIG_DEFAULT();
  my_config.ampdu_rx_enable = false;
  esp_wifi_init(&my_config);
  esp_wifi_start();
  vTaskDelay(100 / portTICK_PERIOD_MS); // Add a small delay
  pairingBegin();
  readWifiCredentials();
  _useExternalScan = !_networks.empty();
#ifdef BOARD_TRMNL_X
  _useExternalScan = _useExternalScan || (bool)_modemScanCallback;
#endif
  _nativeScanRunning = false;
  _scanCache.begin(_useExternalScan ? externalScanSnapshot() : ScanNetworks{}, _useExternalScan);
  serviceNetworkScan();

    // configure DSN and WEB server
  setUpDNSServer(*_dnsServer, localIP);

  WifiOperationCallbacks callbacks = {
      .resetSettings =
        [this]() {
          std::lock_guard<std::mutex> lock(_submissionMutex);
          const bool accepted = _acceptResets;
          if (accepted) _resetRequested = true;
          return accepted;
        },
      .setConnectionCredentials =
        [this](const WifiCredentials credentials, const String api_server, const String band) {
          return enqueueCredentials(credentials, api_server, band);
        },
      .getConnectionState = [this]() { return _connectionState.load(); },
      .getNetworks = [this](bool force) { return _scanCache.read(force); }};

#ifdef BOARD_TRMNL_X
  setUpWebserver(*_server, localIP, callbacks, _modemMac);
#else
  setUpWebserver(*_server, localIP, callbacks);
#endif

    // begin serving
  _server->begin();

  bool succesfullyConnected = false;
  bool connected_via_modem = false;
  long lTime = millis();
    // wait until SSID is provided
  while ((millis() - lTime) < PORTAL_TIMEOUT) // keep CaptivePortal active for a maximum of 15 minutes
  {
    _dnsServer->processNextRequest();

    bool resetRequested;
    {
      std::lock_guard<std::mutex> lock(_submissionMutex);
      resetRequested = _resetRequested;
      _resetRequested = false;
    }
    if (resetRequested) {
      resetSettings();
      if (_resetcallback != NULL) _resetcallback();
    }

    if (_tickCallback) _tickCallback();
    serviceNetworkScan();
    pairingTick();

    Submission submission;
    bool pending;
    {
      std::lock_guard<std::mutex> lock(_submissionMutex);
      // Leave portal requests queued until their HTTP response can reach the browser.
      pending =
        _pendingSubmission && (pairingUsesBle() || (uint32_t)(millis() - _connectionRequestedAt.load()) >= 1000);
      if (pending) {
        submission = _submission;
        _submission = Submission{};
        _pendingSubmission = false;
      }
    }
    if (!pending) {
      delay(DNS_INTERVAL);
    } else {
      _enterprise_credentials = submission.credentials;
      bool res = connectAndSave(submission.credentials, submission.server, submission.band, connected_via_modem);
      pairingWifiResult(res);

      if (res) {
        {
          std::lock_guard<std::mutex> lock(_submissionMutex);
          _acceptSubmissions = false;
        }
        succesfullyConnected = true;
        break;
      } else {
        WiFi.disconnect();
        WiFi.enableSTA(false);
        continue;
      }
    }
  }
  bool finalResetRequested;
  {
    std::lock_guard<std::mutex> lock(_submissionMutex);
    _acceptSubmissions = false;
    // Drain a reset queued while connectAndSave was blocking, even if it succeeded.
    // Later web requests are rejected instead of acknowledging work with no owner.
    _acceptResets = false;
    finalResetRequested = _resetRequested;
    _resetRequested = false;
  }
  if (finalResetRequested) {
    resetSettings();
    if (_resetcallback != NULL) _resetcallback();
  }

    // SSID provided, stop server
  _nativeScanRunning = false;
  WiFi.scanDelete();
  WiFi.softAPdisconnect(true);
  delay(1000);

  if (!connected_via_modem && (millis() - lTime) < PORTAL_TIMEOUT) {
    auto status = WiFi.status();
    if (status != WL_CONNECTED) {
      Log_info("Not connected after AP disconnect");
      WiFi.mode(WIFI_STA);
      // Always start with _enterprise_credentials to preserve static IP settings
      WifiCredentials credentials = _enterprise_credentials;
      Log_info("Reconnecting with static IP: %s, IP: %s", credentials.useStaticIP ? "yes" : "no",
               credentials.staticIP.c_str());
      auto result = initiateConnectionAndWaitForOutcome(credentials);
      status = result.status;
    }
  }

  _ssid = "";
  _password = "";
  _band = "";
  _enterprise_credentials = WifiCredentials{};

  // stop dsn
  _dnsServer->stop();
  delete _dnsServer;
  _dnsServer = nullptr;

  // stop server
  _server->end();
  delete _server;
  _server = nullptr;
  // Take different action for timeout (go to sleep)
  if ((millis() - lTime) > PORTAL_TIMEOUT) {
    pairingBeforeSleep();
#ifdef BOARD_TRMNL_X
    if (power().usbStatus() == UsbStatus::CONNECTED) {
      showMessageWithLogo(READY_TO_SHIP);
      while (power().usbStatus() == UsbStatus::CONNECTED) {
        Serial.println("USB power still detected, waiting...");
        delay(2000);
      }
    }
    showMessageWithLogo(SHIPPING_MODE);
    saveShipmentStarted();
    enter_shipment_sleep();
#else
    showMessageWithLogo(CAPTIVE_WIFI_TIMEOUT);
    goToSleep();
#endif
  }
  return succesfullyConnected;
}

WifiConnectionState WifiCaptive::classifyConnectionFailure(const WifiConnectionResult &result) {
  switch (result.eventData.disconnectReason) {
  case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
  case WIFI_REASON_802_1X_AUTH_FAILED:
  case WIFI_REASON_AUTH_FAIL:
  case WIFI_REASON_HANDSHAKE_TIMEOUT:
    return WifiConnectionState::AuthenticationFailed;
  case WIFI_REASON_NO_AP_FOUND:
    return WifiConnectionState::NetworkNotFound;
  default:
    return WifiConnectionState::Failed;
  }
}

bool WifiCaptive::enqueueCredentials(const WifiCredentials &credentials, const String &server, const String &band,
                                     bool fromBle) {
  std::lock_guard<std::mutex> lock(_submissionMutex);
  if (!_acceptSubmissions || _pendingSubmission) return false;
  if (!fromBle && !pairingReservePortal(server)) return false;
  _submission.credentials = credentials;
  _submission.server = server;
  _submission.band = band;
  _connectionRequestedAt = millis();
  _connectionState = WifiConnectionState::Connecting;
  _pendingSubmission = true;
  return true;
}
bool WifiCaptive::connectAndSave(WifiCredentials credentials, const String &server, const String &band,
                                 bool &connectedViaModem) {
  Log_info("Connecting with static IP: %s, IP: %s", credentials.useStaticIP ? "yes" : "no",
           credentials.staticIP.c_str());
  if (band == "5GHz")
    credentials.is5GHz = true;
  else if (band == "2.4GHz")
    credentials.is5GHz = false;
  else {
    for (const auto &network : pairingNetworks()) {
      if (network.ssid == credentials.ssid) {
        credentials.is5GHz = network.is5GHz;
        break;
      }
    }
  }
  const bool useModem = credentials.is5GHz && !credentials.isEnterprise && _modemConnectCallback;
  bool connected = false;
  WifiConnectionState failureState = WifiConnectionState::Failed;
  if (useModem) {
    connected = _modemConnectCallback(credentials.ssid, credentials.pswd);
  } else {
    const auto result = initiateConnectionAndWaitForOutcome(credentials);
    connected = result.status == WL_CONNECTED;
    if (!connected) failureState = classifyConnectionFailure(result);
  }
  _connectionState = connected ? WifiConnectionState::Connected : failureState;
  if (!connected && !useModem && !pairingUsesBle()) {
    // Native portal joins drop the AP; restart to make setup reachable again.
    Log_info("Connection failed, restarting device to bring the portal back up");
    ESP.restart();
  }
  connectedViaModem = connected && useModem;
  if (connected) {
    saveWifiCredentials(credentials);
    saveApiServer(server);
  }
  return connected;
}
void WifiCaptive::ScanCache::begin(ScanNetworks networks, bool isExternal) {
  std::lock_guard<std::mutex> lock(mutex);
  value = std::move(networks);
  external = isExternal;
  ready = external;
  pending = !external;
  refreshing = false;
  ++generation;
}

// Async consumers copy snapshots and queue requests; they never touch the radio.
WifiNetworkList WifiCaptive::ScanCache::read(bool force) {
  std::lock_guard<std::mutex> lock(mutex);
  if (force) {
    ++generation;
    pending = true;
    ready = false;
  }
  WifiNetworkList reply{ready, {}};
  if (ready) reply.networks = value.annotated;
  if (ready && !external) {
    pending = true;
    ready = false;
  }
  return reply;
}

std::vector<ExternalNetwork> WifiCaptive::ScanCache::latest() {
  std::lock_guard<std::mutex> lock(mutex);
  return value.discovered;
}

WifiCaptive::ScanCache::Request WifiCaptive::ScanCache::takeRequest() {
  std::lock_guard<std::mutex> lock(mutex);
  Request request;
  if (pending && !refreshing) {
    request.requested = true;
    request.generation = generation;
    pending = false;
    refreshing = true;
  }
  return request;
}

void WifiCaptive::ScanCache::publish(Request request, ScanNetworks networks) {
  std::lock_guard<std::mutex> lock(mutex);
  refreshing = false;
  // Preserve a force request that arrived while the radio scan was running.
  if (request.generation != generation) return;
  value = std::move(networks);
  ready = true;
}

void WifiCaptive::ScanCache::retry() {
  std::lock_guard<std::mutex> lock(mutex);
  refreshing = false;
  pending = true;
}

std::vector<ExternalNetwork> WifiCaptive::pairingNetworks() { return _scanCache.latest(); }

WifiCaptive::ScanNetworks WifiCaptive::externalScanSnapshot() {
  ScanNetworks snapshot;
  snapshot.discovered = _networks;
  for (const auto &network : _networks) {
    bool saved = false;
    for (const auto &credentials : _savedWifis)
      if (credentials.ssid == network.ssid) {
        saved = true;
        break;
      }
    snapshot.annotated.push_back({network.ssid, network.rssi, network.open, saved, network.is5GHz, network.enterprise});
  }
  return snapshot;
}

void WifiCaptive::serviceNetworkScan() {
  // Only the portal's main task accesses native scan storage or invokes the
  // modem scanner. No cache lock is held during radio work or SDK buffer copies.
  if (_nativeScanRunning) {
    const int status = WiFi.scanComplete();
    if (status == WIFI_SCAN_RUNNING) return;
    _nativeScanRunning = false;
    if (status == WIFI_SCAN_FAILED) {
      _scanCache.retry();
      return;
    }
    auto networks = getScannedUniqueNetworks(false);
    ScanNetworks snapshot;
    for (const auto &network : networks)
      snapshot.discovered.push_back({network.ssid, network.rssi, network.open, network.is5GHz, network.enterprise});
    snapshot.annotated = combineNetworks(networks, _savedWifis);
    WiFi.scanDelete();
    _scanCache.publish(_nativeScanRequest, std::move(snapshot));
  }

  const auto request = _scanCache.takeRequest();
  if (!request.requested) return;
  if (_useExternalScan) {
#ifdef BOARD_TRMNL_X
    if (_modemScanCallback) _networks = _modemScanCallback();
#endif
    _scanCache.publish(request, externalScanSnapshot());
    return;
  }
  _nativeScanRequest = request;
  WiFi.scanDelete();
#ifdef CONFIG_IDF_TARGET_ESP32C5
  WiFi.setBandMode(WIFI_BAND_MODE_AUTO);
#endif
  WiFi.scanNetworks(true);
  _nativeScanRunning = true;
}

void WifiCaptive::resetSettings() {
  Log_info("Resetting WiFi settings");

  Preferences preferences;
  preferences.begin("wificaptive", false);
  preferences.remove(WIFI_LAST_INDEX);
  for (int i = 0; i < WIFI_MAX_SAVED_CREDS; i++) {
    preferences.remove(WIFI_SSID_KEY(i));
    preferences.remove(WIFI_PSWD_KEY(i));
    preferences.remove(WIFI_5GHZ_KEY(i));
    preferences.remove(WIFI_ENT_KEY(i));
    preferences.remove(WIFI_USERNAME_KEY(i));
    preferences.remove(WIFI_IDENTITY_KEY(i));
    // Remove static IP settings
    preferences.remove(WIFI_USE_STATIC_KEY(i));
    preferences.remove(WIFI_STATIC_IP_KEY(i));
    preferences.remove(WIFI_STATIC_GW_KEY(i));
    preferences.remove(WIFI_STATIC_SN_KEY(i));
    preferences.remove(WIFI_STATIC_DNS1_KEY(i));
    preferences.remove(WIFI_STATIC_DNS2_KEY(i));
  }
  preferences.end();

  for (int i = 0; i < WIFI_MAX_SAVED_CREDS; i++) {
    _savedWifis[i] = WifiCredentials{};
  }

  // Clean up any WPA2 Enterprise state
  disableWpa2Enterprise();

  WiFi.disconnect(true, true);
  WiFi.eraseAP();
}

wl_status_t WifiCaptive::connect(const WifiCredentials credentials) {
  wl_status_t connRes = WL_NO_SSID_AVAIL;

  if (credentials.ssid != "") {
    WiFi.enableSTA(true);

    auto result = initiateConnectionAndWaitForOutcome(credentials);
    connRes = result.status;
  }

  return connRes;
}

void WifiCaptive::setResetSettingsCallback(std::function<void()> func) { _resetcallback = func; }

void WifiCaptive::setPortalTickCallback(std::function<void()> func) { _tickCallback = func; }

void WifiCaptive::setHostname(const String &hostname) { _hostname = hostname; }

String WifiCaptive::getHostname() { return _hostname; }

bool WifiCaptive::isSaved() {
  readWifiCredentials();
  for (const auto &network : _savedWifis)
    if (!network.ssid.isEmpty()) return true;
  return false;
}

void WifiCaptive::readWifiCredentials() {
  Preferences preferences;
  preferences.begin("wificaptive", true);

  for (int i = 0; i < WIFI_MAX_SAVED_CREDS; i++) {
    _savedWifis[i].ssid = preferences.getString(WIFI_SSID_KEY(i), "");
    _savedWifis[i].pswd = preferences.getString(WIFI_PSWD_KEY(i), "");
    _savedWifis[i].is5GHz = preferences.getBool(WIFI_5GHZ_KEY(i), false);
    _savedWifis[i].isEnterprise = preferences.getBool(WIFI_ENT_KEY(i), false);
    _savedWifis[i].username = preferences.getString(WIFI_USERNAME_KEY(i), "");
    _savedWifis[i].identity = preferences.getString(WIFI_IDENTITY_KEY(i), "");
    // Load static IP settings
    _savedWifis[i].useStaticIP = preferences.getBool(WIFI_USE_STATIC_KEY(i), false);
    _savedWifis[i].staticIP = preferences.getString(WIFI_STATIC_IP_KEY(i), "");
    _savedWifis[i].gateway = preferences.getString(WIFI_STATIC_GW_KEY(i), "");
    _savedWifis[i].subnet = preferences.getString(WIFI_STATIC_SN_KEY(i), "");
    _savedWifis[i].dns1 = preferences.getString(WIFI_STATIC_DNS1_KEY(i), "");
    _savedWifis[i].dns2 = preferences.getString(WIFI_STATIC_DNS2_KEY(i), "");
  }

  int idx = preferences.getInt(WIFI_LAST_INDEX, 0);
  if (idx < 0 || idx >= WIFI_MAX_SAVED_CREDS || _savedWifis[idx].ssid.isEmpty()) {
    idx = 0;
    for (int i = 0; i < WIFI_MAX_SAVED_CREDS; ++i)
      if (!_savedWifis[i].ssid.isEmpty()) {
        idx = i;
        break;
      }
  }
  _lastIndex = idx;

  preferences.end();
}

void WifiCaptive::saveWifiCredentials(const WifiCredentials credentials) {
  Log_info("Saving wifi credentials: %s (Enterprise: %s)", credentials.ssid.c_str(),
           credentials.isEnterprise ? "yes" : "no");

  // Check if the credentials already exist
  for (u16_t i = 0; i < WIFI_MAX_SAVED_CREDS; i++) {
    // For regular networks, check SSID and password
    if (!credentials.isEnterprise && !_savedWifis[i].isEnterprise) {
      if (_savedWifis[i].ssid == credentials.ssid && _savedWifis[i].pswd == credentials.pswd) {
        Log_info("Duplicate regular network found, not saving");
        return; // Avoid saving duplicate networks
      }
    }
    // For enterprise networks, check SSID, username, identity, and password
    else if (credentials.isEnterprise && _savedWifis[i].isEnterprise) {
      if (_savedWifis[i].ssid == credentials.ssid && _savedWifis[i].username == credentials.username &&
          _savedWifis[i].identity == credentials.identity && _savedWifis[i].pswd == credentials.pswd) {
        Log_info("Duplicate enterprise network found, not saving");
        return; // Avoid saving duplicate networks
      }
    }
  }

  for (u16_t i = WIFI_MAX_SAVED_CREDS - 1; i > 0; i--) {
    _savedWifis[i] = _savedWifis[i - 1];
  }

  _savedWifis[0] = credentials;

  Preferences preferences;
  preferences.begin("wificaptive", false);
  for (int i = 0; i < WIFI_MAX_SAVED_CREDS; i++) {
    preferences.putString(WIFI_SSID_KEY(i), _savedWifis[i].ssid);
    preferences.putString(WIFI_PSWD_KEY(i), _savedWifis[i].pswd);
    preferences.putBool(WIFI_5GHZ_KEY(i), _savedWifis[i].is5GHz);
    preferences.putBool(WIFI_ENT_KEY(i), _savedWifis[i].isEnterprise);
    preferences.putString(WIFI_USERNAME_KEY(i), _savedWifis[i].username);
    preferences.putString(WIFI_IDENTITY_KEY(i), _savedWifis[i].identity);
    // Save static IP settings
    preferences.putBool(WIFI_USE_STATIC_KEY(i), _savedWifis[i].useStaticIP);
    preferences.putString(WIFI_STATIC_IP_KEY(i), _savedWifis[i].staticIP);
    preferences.putString(WIFI_STATIC_GW_KEY(i), _savedWifis[i].gateway);
    preferences.putString(WIFI_STATIC_SN_KEY(i), _savedWifis[i].subnet);
    preferences.putString(WIFI_STATIC_DNS1_KEY(i), _savedWifis[i].dns1);
    preferences.putString(WIFI_STATIC_DNS2_KEY(i), _savedWifis[i].dns2);
  }
  preferences.putInt(WIFI_LAST_INDEX, 0);
  preferences.end();
}

void WifiCaptive::saveLastUsedWifiIndex(int index) {
  Preferences preferences;
  preferences.begin("wificaptive", false);

  // if index is out of bounds, set to 0
  if (index < 0 || index >= WIFI_MAX_SAVED_CREDS) {
    index = 0;
  }

  // if index is greater than the total number of saved wifis, set to 0
  if (index > 0) {
    readWifiCredentials();
    if (_savedWifis[index].ssid == "") {
      index = 0;
    }
  }

  preferences.putInt(WIFI_LAST_INDEX, index);
  preferences.end();
}

int WifiCaptive::readLastUsedWifiIndex() {
  readWifiCredentials();
  return _lastIndex;
}

void WifiCaptive::saveApiServer(String url) {
  Preferences preferences;
  preferences.begin("data", false);

  String currentUrl = preferences.getString("api_url", "");
  if (currentUrl != url) {
    Log_info("API server changed, clearing cached credentials to force re-pairing");
    preferences.remove("api_key");
    preferences.remove("friendly_id");
  }

  if (url == "") {
    preferences.remove("api_url"); // falls back to API_BASE_URL
  } else {
    preferences.putString("api_url", url);
  }

  preferences.end();
}

int WifiCaptive::findSavedWifiIndex(const WifiCredentials credentials) {
  for (int i = 0; i < WIFI_MAX_SAVED_CREDS; i++) {
    if (_savedWifis[i].ssid != credentials.ssid || _savedWifis[i].pswd != credentials.pswd ||
        _savedWifis[i].is5GHz != credentials.is5GHz || _savedWifis[i].isEnterprise != credentials.isEnterprise) {
      continue;
    }

    if (credentials.isEnterprise &&
        (_savedWifis[i].username != credentials.username || _savedWifis[i].identity != credentials.identity)) {
      continue;
    }

    return i;
  }

  return -1;
}

std::vector<WifiNetwork> WifiCaptive::getScannedUniqueNetworks(bool runScan) {
  std::vector<WifiNetwork> uniqueWifiNetworks;
  int n = WiFi.scanComplete();
  if (runScan == true) {
#ifdef CONFIG_IDF_TARGET_ESP32C5
    // Enable 5GHz network scan
    Log_info("About to set 2.4+5GHz mode");
    esp_wifi_set_band_mode(WIFI_BAND_MODE_AUTO);
    Log_info("after set 5GHz mode");
#endif
    WiFi.scanNetworks(false);
    delay(100);
    int n = WiFi.scanComplete();
    int scanRetries = 0;
    const int maxScanRetries = 3;
    while (n == WIFI_SCAN_RUNNING || n == WIFI_SCAN_FAILED) {
      delay(100);
      if (n == WIFI_SCAN_RUNNING) {
        n = WiFi.scanComplete();
      } else if (n == WIFI_SCAN_FAILED) {
        if (scanRetries >= maxScanRetries) {
          Log_info("Scan failed after %d retries, giving up", maxScanRetries);
          break;
        }
        scanRetries++;

        // There is a race coniditon that can occur, particularly if you use the async flag of WiFi.scanNetworks(true),
        // where you can race before the data is parsed. scanComplete will be -2, we'll see that and fail out, but then
        // a few microseconds later it actually fills in. This fixes that, in case we ever move back to the async
        // version of scanNetworks, but as long as it's sync above it'll work first shot always.
        Log_verbose("Supposedly failed to finish scan, let's wait 10 seconds before checking again (retry %d/%d)",
                    scanRetries, maxScanRetries);
        delay(10000);
        n = WiFi.scanComplete();
        if (n > 0) {
          Log_verbose("Scan actually did complete, we have %d networks, breaking loop.", n);
          // it didn't actually fail, we just raced before the scan was done filling in data
          break;
        }
        WiFi.scanDelete(); // Clean up failed scan state
        delay(100);
        WiFi.scanNetworks(false);
        delay(500);
        n = WiFi.scanComplete();
      }
    }
  }

  n = WiFi.scanComplete();
  Log_verbose("Scanning networks, final scan result: %d", n);

  // Process each found network
  for (int i = 0; i < n; ++i) {
    if (!WiFi.SSID(i).equals("TRMNL")) {
      String ssid = WiFi.SSID(i);
      int32_t rssi = WiFi.RSSI(i);

      wifi_auth_mode_t encType = WiFi.encryptionType(i);
      bool bIs5GHz = (WiFi.channel(i) >= 36);
      bool open = WiFi.encryptionType(i) == WIFI_AUTH_OPEN;
      bool enterprise = (encType == WIFI_AUTH_WPA2_ENTERPRISE);

      bool found = false;
      for (auto &network : uniqueWifiNetworks) {
        if (network.ssid == ssid && network.is5GHz == bIs5GHz) {
          Serial.println("Equal SSID & WiFi Band");
          found = true;
          if (network.rssi < rssi) {
            network.rssi = rssi;
          }
          break;
        }
      }
      if (!found) {
        uniqueWifiNetworks.push_back({ssid, rssi, open, false, bIs5GHz, enterprise});
      }
    }
  }

  Log_info("Unique networks found: %d", uniqueWifiNetworks.size());
  for (auto &network : uniqueWifiNetworks) {
    Log_info("SSID: %s, RSSI: %d, Open: %d, Band: %s", network.ssid.c_str(), network.rssi, network.open,
             (network.is5GHz) ? "5GHz" : "2.4GHz");
  }

  return uniqueWifiNetworks;
}

std::vector<WifiCredentials> WifiCaptive::matchNetworks(std::vector<WifiNetwork> &scanResults,
                                                        WifiCredentials savedWifis[]) {
  // sort scan results by RSSI
  std::sort(scanResults.begin(), scanResults.end(),
            [](const WifiNetwork &a, const WifiNetwork &b) { return a.rssi > b.rssi; });

  std::vector<WifiCredentials> sortedWifis;
  for (auto &network : scanResults) {
    for (int i = 0; i < WIFI_MAX_SAVED_CREDS; i++) {
      if (network.ssid == savedWifis[i].ssid && network.is5GHz == savedWifis[i].is5GHz) {
        sortedWifis.push_back(savedWifis[i]);
      }
    }
  }

  return sortedWifis;
}

std::vector<WifiNetwork> WifiCaptive::combineNetworks(std::vector<WifiNetwork> &scanResults,
                                                      WifiCredentials savedWifis[]) {
  std::vector<WifiNetwork> combinedWifiNetworks;
  for (auto &network : scanResults) {
    bool found = false;
    for (int i = 0; i < WIFI_MAX_SAVED_CREDS; i++) {
      if (network.ssid == savedWifis[i].ssid) {
        combinedWifiNetworks.push_back(
          {network.ssid, network.rssi, network.open, true, network.is5GHz, network.enterprise});
        found = true;
        break;
      }
    }
    if (!found) {
      combinedWifiNetworks.push_back(
        {network.ssid, network.rssi, network.open, false, network.is5GHz, network.enterprise});
    }
  }
  // add saved wifis that are not combinedWifiNetworks
  for (int i = 0; i < WIFI_MAX_SAVED_CREDS; i++) {
    bool found = false;
    for (auto &network : combinedWifiNetworks) {
      if (network.ssid == savedWifis[i].ssid && network.is5GHz == savedWifis[i].is5GHz) {
        found = true;
        break;
      }
    }
    if (!found && savedWifis[i].ssid != "") {
      combinedWifiNetworks.push_back(
        {savedWifis[i].ssid, -200, false, true, savedWifis[i].is5GHz, savedWifis[i].isEnterprise});
    }
  }

  return combinedWifiNetworks;
}

bool WifiCaptive::autoConnect() {
  Log_info("Trying to autoconnect to wifi...");
  readWifiCredentials();

  int last_used_index = readLastUsedWifiIndex();

  if (_savedWifis[last_used_index].ssid != "") {
    Log_info("Trying to connect to last used %s...", _savedWifis[last_used_index].ssid.c_str());
#ifdef TRMNL_BLE_PAIRING
    // BLE recovery and wake advertising share the radio with Wi-Fi.
    // ESP32-C3 coexistence aborts if Wi-Fi modem sleep is disabled.
    WiFi.setSleep(true);
#else
    WiFi.setSleep(false);
#endif
    WiFi.setMinSecurity(WIFI_AUTH_OPEN);
    WiFi.mode(WIFI_STA);

    if (tryConnectWithRetries(_savedWifis[last_used_index], last_used_index)) {
      return true;
    }
  }

  Log_info("Last used network unavailable, scanning for known networks...");
  std::vector<WifiNetwork> scanResults = getScannedUniqueNetworks(true);
  std::vector<WifiCredentials> sortedNetworks = matchNetworks(scanResults, _savedWifis);
  if (sortedNetworks.size() == 0) {
    Log_info("No matched networks found in scan, trying all saved networks...");
    sortedNetworks = std::vector<WifiCredentials>(_savedWifis, _savedWifis + WIFI_MAX_SAVED_CREDS);
  }

  WiFi.mode(WIFI_STA);
  for (auto &network : sortedNetworks) {
    int found_index = findSavedWifiIndex(network);
    if (network.ssid == "" || found_index == last_used_index) {
      continue;
    }

    Log_info("Trying to connect to saved network %s...", network.ssid.c_str());
    if (tryConnectWithRetries(network, found_index)) {
      return true;
    }
  }

  Log_info("Failed to connect to any network");
  return false;
}

bool WifiCaptive::tryConnectWithRetries(const WifiCredentials creds, int last_used_index) {
  for (int attempt = 0; attempt < WIFI_CONNECTION_ATTEMPTS; attempt++) {
    Log_info("Attempt %d to connect to %s (Enterprise: %s, Static IP: %s, IP: %s)", attempt + 1, creds.ssid.c_str(),
             creds.isEnterprise ? "yes" : "no", creds.useStaticIP ? "yes" : "no", creds.staticIP.c_str());
    connect(creds);
    if (WiFi.status() == WL_CONNECTED) {
      Log_info("Connected to %s", creds.ssid.c_str());
      if (last_used_index >= 0) {
        saveLastUsedWifiIndex(last_used_index);
      }
      return true;
    }
    WiFi.disconnect();

    // If this was an enterprise connection, clean up WPA2 Enterprise state
    if (creds.isEnterprise) {
      Log_info("Cleaning up WPA2 Enterprise state after failed attempt");
      disableWpa2Enterprise();
    }

    if (attempt < WIFI_CONNECTION_ATTEMPTS - 1) {
      uint32_t backoff_delay = 2000 * (1 << attempt);
      Log_info("Connection failed, waiting %d ms before retry...", backoff_delay);
      delay(backoff_delay);
    }
  }
  return false;
}

bool checkForSavedCredentials() {
  Preferences preferences;
  preferences.begin("wificaptive", true);
  bool saved = false;
  for (int i = 0; i < WIFI_MAX_SAVED_CREDS; ++i)
    if (!preferences.getString(WIFI_SSID_KEY(i), "").isEmpty()) {
      saved = true;
      break;
    }
  preferences.end();
  return saved;
}

void WifiCaptive::setNetworks(const std::vector<ExternalNetwork> &nets) { _networks = nets; }

WifiCredentials WifiCaptive::getLastCredentials() {
  readWifiCredentials();
  return _savedWifis[_lastIndex];
}

void WifiCaptive::setModemConnectCallback(ModemConnectCallback cb) { _modemConnectCallback = cb; }

#ifdef BOARD_TRMNL_X
void WifiCaptive::setModemMac(const String &mac) { _modemMac = mac; }

void WifiCaptive::setModemScanCallback(ModemScanCallback cb) { _modemScanCallback = cb; }
#endif

bool findNetwork(const char *ssid, int32_t *rssi_out) {
  Log_info("Scanning for network: %s", ssid);

  WiFi.mode(WIFI_STA);
  int n = WiFi.scanNetworks(false);

  if (n == 0) {
    Log_info("No networks found");
    return false;
  }

  Log_info("Found %d networks, searching for %s", n, ssid);

  for (int i = 0; i < n; ++i) {
    String scannedSSID = WiFi.SSID(i);

    if (scannedSSID == ssid) {
      int32_t rssi = WiFi.RSSI(i);
      Log_info("Found %s! RSSI: %d dBm", ssid, rssi);

      if (rssi_out) {
        *rssi_out = rssi;
      }

      WiFi.scanDelete();
      return true;
    }
  }

  Log_info("Network '%s' not found", ssid);
  WiFi.scanDelete();
  return false;
}

WifiCaptive WifiCaptivePortal;
