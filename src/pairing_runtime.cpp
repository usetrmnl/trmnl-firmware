#include <pairing_runtime.h>
#ifdef TRMNL_BLE_PAIRING
#include <ArduinoJson.h>
#include <Preferences.h>
#include <WifiCaptive.h>
#include <atomic>
#include <ble_wake_window.h>
#include <config.h>
#include <esp_idf_version.h>
#include <esp_log.h>
#include <esp_random.h>
#include <esp_timer.h>
#include <globals.h>
#include <mutex>
#include <preferences_persistence.h>
#include <protocomm.h>
#include <protocomm_ble.h>
#include <protocomm_security1.h>
#include <saved_wifi.h>
#include <services/device_setup.h>

extern TRMNL_DEVICE *pDevice;
bool scanForPhoneBeacon(const char *sessionId);

namespace {
  PairingIdentity identity;
  PairingSession session;
  std::mutex guard;
  protocomm_t *pc = nullptr;
  protocomm_ble_config_t bleConfig = {};
  protocomm_ble_name_uuid_t endpoints[] = {{"proto-ver", 0x0001}, {"prov-session", 0x0002}, {"trmnl-control", 0x0003}};
  bool retry = false, updating = false, identityPrepared = false, identityPersisted = false;
  uint64_t authBlockedUntil = 0;
  unsigned authFailures = 0;
  bool wifiManagement = false;
  bool wakeAuthenticated = false;
  uint64_t wakeAuthDeadline = 0;
  uint64_t wakeStartedAt = 0;
  uint64_t wakeLastHoldAt = 0;
  bool wakeReleaseRequested = false;
  BleWakeWindow wakeWindow;
  std::atomic<bool> wakeDisconnected{false};
  esp_err_t closeSecuritySession(protocomm_security_handle_t handle, uint32_t sessionID) {
    // Available on both IDF 4 and 5. Preserve Security 1 cleanup and let the main
    // task stop the transport after the Bluetooth callback has returned.
    wakeDisconnected.store(true);
    return protocomm_security1.close_transport_session(handle, sessionID);
  }
  protocomm_security_t security;
  String lastError;
  std::vector<ExternalNetwork> networkSnapshot;
  const char *identityKey = "ble_identity";
  const char *pendingAttemptKey = "ble_pending";
  uint64_t nowMs() { return esp_timer_get_time() / 1000; }
#if ESP_IDF_VERSION_MAJOR >= 5
  using SecurityParams = const void *;
#else
  using SecurityParams = const protocomm_security_pop_t *;
#endif
  esp_err_t authenticate(protocomm_security_handle_t h, SecurityParams params, uint32_t sid, const uint8_t *in,
                         ssize_t len, uint8_t **out, ssize_t *outlen, void *context) {
    if (nowMs() < authBlockedUntil) return ESP_ERR_INVALID_STATE;
    esp_err_t result = protocomm_security1.security_req_handler(h, params, sid, in, len, out, outlen, context);
    if (result != ESP_OK) {
      if (authFailures < 5) authFailures++;
      authBlockedUntil = nowMs() + (1000U << authFailures);
    }
    return result;
  }
  void writeStatus(JsonDocument &d, const String &error = "") {
    d["v"] = 1;
    d["sid"] = identity.sessionId;
    d["attempt"] = session.attempt();
    d["state"] = pairingStateName(session.state());
    d["remaining_ms"] = session.remaining(nowMs());
    if (error.length())
      d["error"] = error;
    else if (lastError.length())
      d["error"] = lastError;
    if (session.allowsBle() &&
        (session.state() == PairingState::codeReady || session.state() == PairingState::handedOff))
      d["friendly_id"] = session.code();
    d["updating"] = updating;
    if (wifiManagement) {
      d["wifi_management"] = true;
      d["wake_experiment"] = true; // Compatibility with existing companion apps.
      d["wake_elapsed_ms"] = nowMs() - wakeStartedAt;
      d["wake_hold_remaining_ms"] = wakeWindow.remaining(nowMs());
    }
  }
  String status(const String &error = "") {
    JsonDocument d;
    writeStatus(d, error);
    String s;
    serializeJson(d, s);
    return s;
  }
  void writeNetworks(JsonDocument &d, uint32_t offset, uint32_t limit) {
    const String apSSID = WifiCaptivePortal.getAPSSID();
    JsonArray networks = d["networks"].to<JsonArray>();
    // Reserve the largest cursor before measuring each candidate entry.
    d["next_offset"] = UINT32_MAX;
    size_t i = offset;
    for (; i < networkSnapshot.size() && networks.size() < limit; ++i) {
      const auto &network = networkSnapshot[i];
      if (network.ssid == apSSID) continue;
      JsonObject entry = networks.add<JsonObject>();
      entry["ssid"] = network.ssid;
      entry["band"] = network.is5GHz ? "5GHz" : "2.4GHz";
      entry["security"] = network.enterprise ? "enterprise" : network.open ? "open" : "personal";
      entry["rssi"] = network.rssi;
      if (measureJson(d) > 2048) {
        networks.remove(networks.size() - 1);
        // Skip an oversized entry so the next page cannot repeat forever.
        if (networks.size() == 0) continue;
        break;
      }
    }
    if (i < networkSnapshot.size())
      d["next_offset"] = (uint32_t)i;
    else
      d["next_offset"] = nullptr;
  }
  String manageWifi(const PairingRequest &request) {
    if (!wakeAuthenticated || !wakeWindow.remaining(nowMs())) return status("hold_required");
    Preferences preferences;
    if (!preferences.begin("wificaptive", false)) return status("storage_failed");
    PreferencesPersistence persistence(preferences);
    SavedWifiStore store(persistence);
    String error;
    if (request.op == "wifi_add")
      error = store.add(request.ssid, request.password);
    else if (request.op == "wifi_remove")
      error = store.remove(request.slot, request.ssid);
    JsonDocument d;
    writeStatus(d, error);
    if (!error.length()) d.remove("error");
    JsonArray networks = d["saved_networks"].to<JsonArray>();
    for (const auto &network : store.list()) {
      JsonObject entry = networks.add<JsonObject>();
      entry["slot"] = network.slot;
      entry["ssid"] = network.ssid;
      entry["band"] = network.is5GHz ? "5GHz" : "2.4GHz";
      entry["security"] = network.enterprise ? "enterprise" : network.open ? "open" : "personal";
    }
    preferences.end();
    String response;
    serializeJson(d, response);
    return response;
  }
  bool isWakeLeaseRequest(const PairingRequest &request) {
    return request.op == "wake_hold" || request.op == "wake_release";
  }
  bool isWifiManagementRequest(const PairingRequest &request) {
    return request.op == "wifi_list" || request.op == "wifi_add" || request.op == "wifi_remove";
  }
  // Caller holds guard, as for Wi-Fi management commands.
  String handleWakeLease(const PairingRequest &request) {
    if (request.attempt != 0) return status("stale_attempt");
    if (request.op == "wake_release") {
      wakeReleaseRequested = true;
      wakeWindow.release();
      return status();
    }
    const uint64_t now = nowMs();
    if (!wakeAuthenticated && now >= wakeAuthDeadline) return status("authentication_timeout");
    if (!wakeAuthenticated) wakeDisconnected.store(false);
    wakeWindow.hold(now);
    if (wakeWindow.remaining(now)) {
      wakeAuthenticated = true;
      wakeLastHoldAt = now;
    }
    return status();
  }
  String configureWifi(const PairingRequest &r) {
    // The portal owns the credential queue. Release guard before enqueueing.
    WifiCredentials c;
    {
      std::lock_guard<std::mutex> lock(guard);
      session.tick(nowMs());
      if (!session.allowsBle()) return status("unsupported_server");
      bool enterprise = false, personal = false;
      for (const auto &n : networkSnapshot)
        if (n.ssid == r.ssid) {
          if (n.enterprise)
            enterprise = true;
          else
            personal = true;
        }
      if (enterprise && !personal) return status("invalid_credentials");
      if (r.attempt == session.attempt()) return status();
      if (r.attempt < session.attempt()) return status("stale_attempt");
      if (!session.submit(r.attempt, r.ssid, r.password)) return status("busy");
      c.ssid = r.ssid;
      c.pswd = r.password;
      lastError = "";
    }
    if (!WifiCaptivePortal.enqueueCredentials(c, TRMNL_BLE_SETUP_SERVER_URL, "", true)) {
      std::lock_guard<std::mutex> lock(guard);
      session.wifiResult(false);
      lastError = "busy";
    }
    std::lock_guard<std::mutex> lock(guard);
    return status();
  }
  String handleSetupRequest(const PairingRequest &r) {
    std::lock_guard<std::mutex> lock(guard);
    session.tick(nowMs());
    // Enforce on every operation: the portal can change the attempt's server
    // after a connected application has already checked capabilities.
    if (r.op != "capabilities" && !session.allowsBle()) return status("unsupported_server");
    if (r.attempt && r.attempt != session.attempt()) return status("stale_attempt");
    if (r.op == "retry_setup") {
      if (!session.retrySetup(r.attempt)) return status("busy");
      retry = true;
      lastError = "";
    }
    if (r.op == "ack_code") {
      if (!session.acknowledge(r.attempt, nowMs())) return status("busy");
      // Persist the ACK before replying; a reset may precede the sleep hook.
      if (preferencesPersistence.recordExists(pendingAttemptKey) && !preferencesPersistence.remove(pendingAttemptKey))
        return status("storage_failed");
    }
    JsonDocument d;
    writeStatus(d);
    if (r.op == "capabilities") {
      d["server"] = session.server();
      d["security"] = 1;
      JsonArray bands = d["bands"].to<JsonArray>();
      bands.add("2.4GHz");
#ifdef BOARD_TRMNL_X
      bands.add("5GHz");
#endif
    }
    if (r.op == "networks") {
      writeNetworks(d, r.offset, r.limit);
    }
    String s;
    serializeJson(d, s);
    return s;
  }
  String control(const String &input) {
    PairingRequest request;
    String error;
    if (!parsePairingRequest(input, identity, request, error)) {
      std::lock_guard<std::mutex> lock(guard);
      return status(error);
    }
    // Protocomm has already authenticated this endpoint with Security 1.
    if (isWakeLeaseRequest(request) || isWifiManagementRequest(request)) {
      std::lock_guard<std::mutex> lock(guard);
      if (!wifiManagement) return status("unsupported_operation");
      return isWakeLeaseRequest(request) ? handleWakeLease(request) : manageWifi(request);
    }
    if (wifiManagement && request.op != "status" && request.op != "capabilities" && request.op != "networks" &&
        request.op != "ack_code") {
      std::lock_guard<std::mutex> lock(guard);
      return status("unsupported_operation");
    }
    return request.op == "configure" ? configureWifi(request) : handleSetupRequest(request);
  }
  esp_err_t handler(uint32_t, const uint8_t *in, ssize_t len, uint8_t **out, ssize_t *outlen, void *) {
    if (len < 0 || len > 2048 || (!in && len)) return ESP_ERR_INVALID_ARG;
    // Arduino 2.x concat copies length + 1 bytes, but protocomm's decrypted
    // buffer has no trailing NUL. Supply one without reading beyond the packet.
    char *terminated = (char *)malloc(len + 1);
    if (!terminated) return ESP_ERR_NO_MEM;
    if (len) memcpy(terminated, in, len);
    terminated[len] = '\0';
    String input;
    const bool copied = input.concat(terminated, len);
    free(terminated);
    if (!copied) return ESP_ERR_NO_MEM;
    String response = control(input);
    if (response.length() > 2048) return ESP_ERR_INVALID_SIZE;
    *out = (uint8_t *)malloc(response.length());
    if (!*out) return ESP_ERR_NO_MEM;
    memcpy(*out, response.c_str(), response.length());
    *outlen = response.length();
    return ESP_OK;
  }
  void stop() {
    if (pc) {
      protocomm_ble_stop(pc);
      protocomm_delete(pc);
      pc = nullptr;
    }
    identityPrepared = false;
    wifiManagement = false;
  }
  void start() {
    pc = protocomm_new();
    if (!pc) return;
 // GATT stores a 128-bit UUID in little-endian order.
    const uint8_t uuid[] = {0xd4, 0x51, 0xec, 0x63, 0xbe, 0x20, 0x63, 0x9a,
                            0x82, 0x4f, 0x91, 0x4e, 0x00, 0x00, 0x3e, 0x7c};
    memcpy(bleConfig.service_uuid, uuid, 16);
    pairingAdvertisement(identity).toCharArray(bleConfig.device_name, sizeof(bleConfig.device_name));
    bleConfig.nu_lookup = endpoints;
    bleConfig.nu_lookup_count = 3;
    protocomm_security_pop_t pop = {(const uint8_t *)identity.proof.c_str(), (uint16_t)identity.proof.length()};
    security = protocomm_security1;
    security.security_req_handler = authenticate;
    security.close_transport_session = closeSecuritySession;
    esp_log_level_set("security1", ESP_LOG_NONE);
    if (protocomm_set_version(
          pc, "proto-ver",
          "{\"prov\":{\"ver\":\"v1.1\",\"sec_ver\":1,\"sec_patch_ver\":0,\"cap\":[]},\"trmnl\":{\"ver\":1}}") !=
          ESP_OK ||
        protocomm_set_security(pc, "prov-session", &security, &pop) != ESP_OK ||
        protocomm_ble_start(pc, &bleConfig) != ESP_OK ||
        protocomm_add_endpoint(pc, "trmnl-control", handler, nullptr) != ESP_OK) {
      stop();
    }
  }
} // namespace
const PairingIdentity &pairingIdentity() { return identity; }
bool pairingIdentityReady() { return identityPrepared; }
String pairingPersistentKey() { return identityPersisted ? identity.proof : String(); }
bool pairingUsesBle() {
  if (!pc) return false;
  std::lock_guard<std::mutex> lock(guard);
  return session.usesBle();
}
void pairingClear() {
  stop();
  preferencesPersistence.remove(identityKey);
  preferencesPersistence.remove(pendingAttemptKey);
  identity = PairingIdentity{};
  identityPersisted = false;
}
void pairingPrepare() {
  if (identityPrepared) return;
  identityPersisted = false;
  if (preferencesPersistence.recordExists(identityKey)) {
    if (!decodePairingIdentity(preferencesPersistence.readString(identityKey, ""), identity) ||
        pairingHelpQrFragment(identity).length() == 0)
      return;
  } else {
    uint8_t bytes[16];
    esp_fill_random(bytes, sizeof(bytes));
    bytes[6] = (bytes[6] & 15) | 64;
    bytes[8] = (bytes[8] & 63) | 128;
    char sid[37];
    snprintf(sid, sizeof(sid), "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x", bytes[0],
             bytes[1], bytes[2], bytes[3], bytes[4], bytes[5], bytes[6], bytes[7], bytes[8], bytes[9], bytes[10],
             bytes[11], bytes[12], bytes[13], bytes[14], bytes[15]);
    identity.sessionId = sid;
    identity.proof = "";
    const char *alphabet = "0123456789ABCDEFGHJKMNPQRSTVWXYZ";
    esp_fill_random(bytes, sizeof(bytes));
    for (auto b : bytes)
      identity.proof += alphabet[b & 31];
    String apSsid = WifiCaptivePortal.getAPSSID();
    identity.mac = apSsid.substring(apSsid.length() - 6);
    // Persist before displaying the QR or advertising this identity.
    if (!savePairingIdentity(preferencesPersistence, identityKey, identity)) return;
  }
  identityPersisted = true;
  String server = preferencesPersistence.readString(PREFERENCES_API_URL, API_BASE_URL);
  session.start(nowMs(), server);
  retry = false;
  updating = false;
  lastError = "";
  networkSnapshot.clear();
  authFailures = 0;
  authBlockedUntil = 0;
  identityPrepared = true;
}
void pairingBegin() {
  // The screen may already contain this identity. Start BLE only after the
  // portal has completed its Wi-Fi driver reinitialization.
  pairingPrepare();
  if (pc || !identityPrepared) return;
  start();
}
bool pairingWakeBegin() {
  // Use the boot wake reason: display updates may have entered light sleep since.
  // Only a physical button/touch wake should spend time scanning for a phone.
  if (wakeup_reason != ESP_SLEEP_WAKEUP_GPIO && wakeup_reason != ESP_SLEEP_WAKEUP_EXT0 &&
      wakeup_reason != ESP_SLEEP_WAKEUP_EXT1)
    return false;
  if (pc || !preferencesPersistence.recordExists(identityKey) ||
      !preferencesPersistence.recordExists(PREFERENCES_API_KEY))
    return false;
  pairingPrepare();
  if (!identityPrepared) return false;
  const uint64_t scanStarted = nowMs();
  if (!scanForPhoneBeacon(identity.sessionId.c_str())) {
    ESP_LOGI("ble_wake", "phone beacon absent; scan lifecycle %u ms", (unsigned)(nowMs() - scanStarted));
    return false;
  }
  const uint32_t pendingAttempt = preferencesPersistence.readUint(pendingAttemptKey, 0);
  if (pendingAttempt) {
    String code = preferencesPersistence.readString(PREFERENCES_FRIENDLY_ID, "");
    if (!validFriendlyCode(code)) return false;
    session.restore(nowMs(), pendingAttempt, code, session.server());
  }
  wakeDisconnected.store(false);
  wakeWindow = BleWakeWindow{};
  wakeAuthenticated = false;
  wakeReleaseRequested = false;
  wakeLastHoldAt = 0;
  wifiManagement = true;
  wakeStartedAt = nowMs();
  wakeAuthDeadline = wakeStartedAt + 5000;
  start();
  if (!pc) return false;
  while (nowMs() < wakeAuthDeadline) {
    {
      std::lock_guard<std::mutex> lock(guard);
      if (wakeAuthenticated) return true;
    }
    delay(5);
  }
  {
    std::lock_guard<std::mutex> lock(guard);
    if (wakeAuthenticated) return true;
    wakeWindow.release();
  }
  stop();
  ESP_LOGI("ble_wake", "phone beacon detected but authentication timed out; resuming refresh");
  return false;
}

bool pairingReservePortal(const String &server) {
  if (!pc) return true;
  std::lock_guard<std::mutex> lock(guard);
  session.tick(nowMs());
  return session.reservePortal(server);
}
void pairingWifiResult(bool ok) {
  if (!pc) return;
  std::lock_guard<std::mutex> lock(guard);
  session.tick(nowMs());
  session.wifiResult(ok);
  if (!ok) lastError = "wifi_failed";
}
void pairingCredentialsReady(const String &code) {
  if (!pc) return;
  std::lock_guard<std::mutex> lock(guard);
  const uint64_t now = nowMs();
  session.tick(now);
  const bool wasSettingUp = session.state() == PairingState::settingUp;
  session.setupResult(code, now);
  if (session.state() == PairingState::codeReady) {
    lastError = "";
    if (wasSettingUp && session.usesBle() &&
        preferencesPersistence.writeUint(pendingAttemptKey, session.attempt()) != sizeof(uint32_t)) {
      lastError = "storage_failed";
    }
  }
}
void pairingSetupFinished() {
  if (!pc) return;
  std::lock_guard<std::mutex> lock(guard);
  if (session.state() == PairingState::settingUp) {
    session.setupResult("", nowMs());
    lastError = "setup_failed";
  }
}
void pairingTick(DeviceSetupResult *setupResult) {
  if (!pc || wifiManagement) return;
  auto nets = WifiCaptivePortal.pairingNetworks();
  bool doRetry = false, done = false, handedOff = false;
  {
    std::lock_guard<std::mutex> lock(guard);
    networkSnapshot = std::move(nets);
    session.tick(nowMs());
    doRetry = retry;
    retry = false;
    handedOff = session.handoffComplete(nowMs());
    done = session.state() == PairingState::expired || handedOff;
  }
  if (done) {
    if (handedOff) preferencesPersistence.remove(pendingAttemptKey);
    stop();
    return;
  }
  if (doRetry) {
    auto result = deviceSetup().perform();
    if (setupResult) *setupResult = std::move(result);
    pairingSetupFinished();
  }
}
void pairingResume() {
  if (pc) return;
  const uint32_t attempt = preferencesPersistence.readUint(pendingAttemptKey, 0);
  if (!attempt) return;
  String code = preferencesPersistence.readString(PREFERENCES_FRIENDLY_ID, "");
  if (!validFriendlyCode(code) || !preferencesPersistence.recordExists(PREFERENCES_API_KEY) ||
      !preferencesPersistence.recordExists(identityKey)) {
    preferencesPersistence.remove(pendingAttemptKey);
    return;
  }
  pairingPrepare();
  if (!identityPrepared) return;
  String server = preferencesPersistence.readString(PREFERENCES_API_URL, API_BASE_URL);
  session.restore(nowMs(), attempt, code, server);
  start();
}
static void prepareHandoff(bool ota) {
  if (!pairingUsesBle()) return;
 // Give the phone time to acknowledge the durable code before stopping BLE.
  while (pc) {
    bool wait;
    {
      std::lock_guard<std::mutex> lock(guard);
      wait = session.waitingForHandoff(nowMs());
    }
    if (!wait) break;
    pairingTick();
    delay(25);
  }
  if (!pc) return;
  {
    std::lock_guard<std::mutex> lock(guard);
    session.tick(nowMs());
    if (session.state() == PairingState::codeReady) {
      updating = ota;
    } else if (session.state() == PairingState::handedOff)
      preferencesPersistence.remove(pendingAttemptKey);
  }
  delay(250);
  stop();
}
void pairingBeforeOta() {
  if (wifiManagement) {
    stop();
    return;
  }
  prepareHandoff(true);
}
void pairingBeforeSleep() {
  if (wifiManagement) {
    const uint64_t sleepAt = nowMs();
    uint32_t remaining;
    const char *exitReason = "timeout";
    uint64_t sinceLastHold = 0;
    pinMode(pDevice->interrupt_pin, INPUT);
    BleWakeExitButton exitButton;
    exitButton.pressed(nowMs(), digitalRead(pDevice->interrupt_pin) == LOW);
    do {
      const bool buttonExit = exitButton.pressed(nowMs(), digitalRead(pDevice->interrupt_pin) == LOW);
      {
        std::lock_guard<std::mutex> lock(guard);
        const bool disconnected = wakeDisconnected.load();
        if (buttonExit || disconnected) wakeWindow.release();
        if (wakeReleaseRequested)
          exitReason = "release";
        else if (buttonExit)
          exitReason = "button";
        else if (disconnected)
          exitReason = "disconnect";
        remaining = wakeWindow.remaining(nowMs());
        // Seal the lease before releasing the mutex and stopping the transport.
        if (!remaining) {
          wakeWindow.release();
          sinceLastHold = nowMs() - wakeLastHoldAt;
        }
      }
      if (remaining) delay(10);
    } while (remaining);
    const auto extension = nowMs() - sleepAt;
    ESP_LOGI("ble_wake", "management exit reason=%s; last keepalive %u ms ago", exitReason, (unsigned)sinceLastHold);
    stop();
    ESP_LOGI("ble_wake", "management ended: hold added %u ms; stop took %u ms; total BLE lifetime %u ms",
             (unsigned)extension, (unsigned)(nowMs() - sleepAt - extension), (unsigned)(nowMs() - wakeStartedAt));
    return;
  }
  prepareHandoff(false);
  stop();
}
#else
static PairingIdentity emptyIdentity;
const PairingIdentity &pairingIdentity() { return emptyIdentity; }
bool pairingIdentityReady() { return false; }
String pairingPersistentKey() { return String(); }
bool pairingUsesBle() { return false; }
void pairingBegin() {}
bool pairingWakeBegin() { return false; }
void pairingPrepare() {}
void pairingTick(DeviceSetupResult *) {}
void pairingWifiResult(bool) {}
bool pairingReservePortal(const String &) { return true; }
void pairingCredentialsReady(const String &) {}
void pairingSetupFinished() {}
void pairingBeforeOta() {}
void pairingBeforeSleep() {}
void pairingResume() {}
void pairingClear() {}
#endif
