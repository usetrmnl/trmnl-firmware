#pragma once
#include <Arduino.h>
#include <persistence_interface.h>
#include <stdint.h>

// Active setup window; the saved QR/BLE identity does not expire.
constexpr uint32_t PROVISIONING_TIMEOUT_MS = 15U * 60U * 1000U;

// The firmware API origin and account-facing origin refer to production.
constexpr const char *TRMNL_DEFAULT_SETUP_URL = "https://trmnl.app";
constexpr const char *TRMNL_BLE_SETUP_SERVER_URL = "https://trmnl.eliz.live";

struct PairingIdentity {
  String sessionId, proof, mac;
};
bool decodePairingIdentity(const String &, PairingIdentity &);
bool savePairingIdentity(Persistence &, const char *key, const PairingIdentity &);
String pairingHelpQrFragment(const PairingIdentity &);
String pairingAdvertisement(const PairingIdentity &);
bool validPairingCredentials(const String &, const String &);
bool validFriendlyCode(const String &);
struct PairingRequest {
  String op, ssid, password;
  uint32_t attempt = 0, offset = 0, limit = 8;
  int slot = -1;
};
bool parsePairingRequest(const String &, const PairingIdentity &, PairingRequest &, String &error);
enum class PairingState {
  idle,
  waiting,
  connecting,
  wifiFailed,
  settingUp,
  setupFailed,
  codeReady,
  handedOff,
  expired
};
const char *pairingStateName(PairingState);
class PairingSession {
public:
  // Callers serialize access; network work runs in the captive-portal queue.
  void start(uint64_t now, const String &server);
  bool submit(uint32_t attempt, const String &ssid, const String &password);
  bool reservePortal(const String &server);
  void wifiResult(bool ok);
  void setupResult(const String &code, uint64_t now);
  bool retrySetup(uint32_t attempt);
  bool acknowledge(uint32_t attempt, uint64_t now);
  bool handoffComplete(uint64_t now) const;
  bool waitingForHandoff(uint64_t now) const;
  void tick(uint64_t now);
  void restore(uint64_t now, uint32_t attempt, const String &code, const String &server);
  uint32_t remaining(uint64_t now) const;
  PairingState state() const { return _state; }
  uint32_t attempt() const { return _attempt; }
  const String &code() const { return _code; }
  const String &server() const { return _server; }
  bool usesBle() const { return _fromBle; }
  bool allowsBle() const { return _server == TRMNL_BLE_SETUP_SERVER_URL; }

private:
  bool submitAttempt(uint32_t attempt);
  PairingState _state = PairingState::idle;
  uint64_t _started = 0;
  uint32_t _attempt = 0;
  String _code, _server;
  bool _fromBle = false;
  uint64_t _readyAt = 0, _ackAt = 0;
};
