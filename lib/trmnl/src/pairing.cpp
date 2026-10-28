#include <ArduinoJson.h>
#include <ctype.h>
#include <pairing.h>
#include <string.h>

static const char alphabet[] = "0123456789ABCDEFGHJKMNPQRSTVWXYZ";
static bool uuid(String &s) {
  if (s.length() != 36) return false;
  for (unsigned i = 0; i < 36; i++) {
    if (i == 8 || i == 13 || i == 18 || i == 23) {
      if (s[i] != '-') return false;
    } else if (!isxdigit((unsigned char)s[i]))
      return false;
  }
  s.toLowerCase();
  return true;
}
static bool validMacSuffix(const String &s) {
  if (s.length() != 6) return false;
  for (unsigned i = 0; i < s.length(); ++i)
    if (!isdigit((unsigned char)s[i]) && !(s[i] >= 'A' && s[i] <= 'F')) return false;
  return true;
}
bool decodePairingIdentity(const String &blob, PairingIdentity &out) {
  if (blob.length() > 512) return false;
  JsonDocument d;
  if (deserializeJson(d, blob)) return false;
  if (!d["v"].is<unsigned>() || d["v"].as<unsigned>() != 1) return false;
  if (!d["sid"].is<const char *>() || !d["pop"].is<const char *>()) return false;
  PairingIdentity id;
  id.sessionId = d["sid"].as<String>();
  if (!uuid(id.sessionId)) return false;
  String proof = d["pop"].as<String>();
  for (unsigned i = 0; i < proof.length(); i++) {
    unsigned char c = proof[i];
    if (isspace(c) || c == '-') continue;
    c = toupper(c);
    if (!c || !strchr(alphabet, c)) return false;
    id.proof += (char)c;
  }
  if (id.proof.length() != 16) return false;
  if (!d["mac"].isNull()) {
    if (!d["mac"].is<const char *>()) return false;
    id.mac = d["mac"].as<String>();
    if (!validMacSuffix(id.mac)) return false;
  }
  out = id;
  return true;
}
String pairingHelpQrFragment(const PairingIdentity &id) {
  String sid = id.sessionId;
  if (!uuid(sid) || !validMacSuffix(id.mac) || id.proof.length() != 16) return "";
  for (unsigned i = 0; i < id.proof.length(); ++i)
    if (!id.proof[i] || !strchr(alphabet, id.proof[i])) return "";
  sid.replace("-", "");
  sid.toUpperCase();
  return String("BLE:1:") + sid + ":" + id.proof + ":" + id.mac;
}
String pairingAdvertisement(const PairingIdentity &id) {
  String hex = id.sessionId;
  hex.replace("-", "");
  if (hex.length() != 32) return "";
  uint8_t b[16];
  for (int i = 0; i < 16; i++)
    b[i] = strtoul(hex.substring(i * 2, i * 2 + 2).c_str(), nullptr, 16);
  String result = "TR";
  for (int i = 0; i < 26; i++) {
    unsigned n = 0;
    for (int j = 0; j < 5; j++) {
      int bit = i * 5 + j - 2;
      n = (n << 1) | (bit < 0 ? 0 : ((b[bit / 8] >> (7 - bit % 8)) & 1));
    }
    result += alphabet[n];
  }
  return result;
}
bool validPairingCredentials(const String &ssid, const String &password) {
  if (ssid.length() < 1 || ssid.length() > 32 || strlen(ssid.c_str()) != ssid.length() ||
      strlen(password.c_str()) != password.length())
    return false;
  if (password.length() == 0 || (password.length() >= 8 && password.length() <= 63)) return true;
  if (password.length() != 64) return false;
  for (unsigned i = 0; i < 64; i++)
    if (!isxdigit((unsigned char)password[i])) return false;
  return true;
}
bool validFriendlyCode(const String &s) {
  if (s.length() != 6) return false;
  for (unsigned i = 0; i < 6; i++)
    if (!((s[i] >= '0' && s[i] <= '9') || (s[i] >= 'A' && s[i] <= 'Z') || (s[i] >= 'a' && s[i] <= 'z'))) return false;
  return true;
}
// Arduino String conversion can discard bytes after a JSON \u0000. Validate the
// original JSON string before interpreting names, credentials, or protocol fields.
static bool jsonStringWithoutNul(JsonVariantConst value) {
  if (!value.is<const char *>()) return false;
  const JsonString text = value.as<JsonString>();
  return memchr(text.c_str(), '\0', text.size()) == nullptr;
}
bool parsePairingRequest(const String &text, const PairingIdentity &id, PairingRequest &out, String &error) {
  error = "invalid_request";
  if (text.length() > 2048) return false;
  JsonDocument d;
  if (deserializeJson(d, text)) return false;
  if (!d["v"].is<unsigned>()) return false;
  if (d["v"].as<unsigned>() != 1) {
    error = "unsupported_version";
    return false;
  }
  if (!jsonStringWithoutNul(d["sid"]) || d["sid"].as<String>() != id.sessionId) {
    error = "wrong_session";
    return false;
  }
  if (!d["attempt"].is<uint32_t>() || !jsonStringWithoutNul(d["op"])) return false;
  PairingRequest r;
  r.attempt = d["attempt"];
  r.op = d["op"].as<String>();
  const bool wifi = r.op == "wifi_list" || r.op == "wifi_add" || r.op == "wifi_remove";
  if (wifi && r.attempt != 0) return false;
  if (!wifi && r.op != "capabilities" && r.op != "networks" && r.op != "status" && r.op != "configure" &&
      r.op != "retry_setup" && r.op != "ack_code" && r.op != "wake_hold" && r.op != "wake_release")
    return false;
  if (!wifi && r.attempt == 0 && r.op != "capabilities" && r.op != "networks" && r.op != "status" &&
      r.op != "wake_hold" && r.op != "wake_release")
    return false;
  if (r.op == "configure" || r.op == "wifi_add") {
    error = "invalid_credentials";
    if (!jsonStringWithoutNul(d["ssid"]) || !jsonStringWithoutNul(d["password"]) || d["enterprise"].as<bool>())
      return false;
    if (wifi && !d["enterprise"].isNull() && (!d["enterprise"].is<bool>() || d["enterprise"].as<bool>())) return false;
    if (wifi && ((!d["band"].isNull() && (!jsonStringWithoutNul(d["band"]) || d["band"].as<String>() != "2.4GHz")) ||
                 (!d["security"].isNull() &&
                  (!jsonStringWithoutNul(d["security"]) ||
                   (d["security"].as<String>() != "open" && d["security"].as<String>() != "personal")))))
      return false;
    r.ssid = d["ssid"].as<String>();
    r.password = d["password"].as<String>();
    if (!validPairingCredentials(r.ssid, r.password)) return false;
    if (wifi && !d["security"].isNull() &&
        d["security"].as<String>() != (r.password.length() == 0 ? "open" : "personal"))
      return false;
  }
  if (r.op == "wifi_remove") {
    error = "invalid_credentials";
    if (!d["slot"].is<unsigned>() || d["slot"].as<unsigned>() >= 5 || !jsonStringWithoutNul(d["ssid"])) return false;
    r.slot = d["slot"].as<int>();
    r.ssid = d["ssid"].as<String>();
    if (!validPairingCredentials(r.ssid, "")) return false;
  }
  if (r.op == "networks") {
    if (!d["offset"].is<uint32_t>() || !d["limit"].is<unsigned>()) return false;
    r.offset = d["offset"];
    r.limit = d["limit"];
    if (r.limit < 1 || r.limit > 8) return false;
  }
  out = r;
  error = "";
  return true;
}
const char *pairingStateName(PairingState s) {
  static const char *names[] = {"waiting",      "waiting",    "connecting", "wifi_failed", "setting_up",
                                "setup_failed", "code_ready", "handed_off", "expired"};
  return names[(int)s];
}
static String canonicalServer(String server) {
  while (server.endsWith("/"))
    server.remove(server.length() - 1);
  if (server.length() == 0 || server == TRMNL_DEFAULT_SETUP_URL) return TRMNL_BLE_SETUP_SERVER_URL;
  return server;
}
void PairingSession::start(uint64_t now, const String &server) {
  _started = now;
  _attempt = 0;
  _code = "";
  _state = PairingState::waiting;
  _server = canonicalServer(server);
  _fromBle = false;
  _readyAt = _ackAt = 0;
}
uint32_t PairingSession::remaining(uint64_t now) const {
  if (_state == PairingState::idle || now < _started) return 0;
  const uint64_t elapsed = now - _started;
  return elapsed >= PROVISIONING_TIMEOUT_MS ? 0 : PROVISIONING_TIMEOUT_MS - elapsed;
}
void PairingSession::tick(uint64_t now) {
  if (_state != PairingState::idle && _state != PairingState::handedOff && !remaining(now)) {
    _state = PairingState::expired;
    _code = "";
  }
}
bool PairingSession::submitAttempt(uint32_t a) {
  if (!a || a <= _attempt || (_state != PairingState::waiting && _state != PairingState::wifiFailed)) return false;
  _attempt = a;
  _state = PairingState::connecting;
  return true;
}
bool PairingSession::submit(uint32_t attempt, const String &ssid, const String &password) {
  if (!allowsBle() || !validPairingCredentials(ssid, password) || !submitAttempt(attempt)) return false;
  _fromBle = true;
  return true;
}
bool PairingSession::reservePortal(const String &server) {
  if (!submitAttempt(_attempt + 1)) return false;
  _server = canonicalServer(server);
  _fromBle = false;
  return true;
}
void PairingSession::wifiResult(bool ok) {
  if (_state == PairingState::connecting) _state = ok ? PairingState::settingUp : PairingState::wifiFailed;
}
void PairingSession::setupResult(const String &code, uint64_t now) {
  if (_state == PairingState::settingUp) {
    _code = validFriendlyCode(code) ? code : String();
    _state = _code.length() ? PairingState::codeReady : PairingState::setupFailed;
    if (_state == PairingState::codeReady) _readyAt = now;
  }
}
bool PairingSession::retrySetup(uint32_t a) {
  if (a != _attempt || _state != PairingState::setupFailed) return false;
  _state = PairingState::settingUp;
  return true;
}
bool PairingSession::acknowledge(uint32_t a, uint64_t now) {
  if (a != _attempt || (_state != PairingState::codeReady && _state != PairingState::handedOff)) return false;
  if (_state != PairingState::handedOff) _ackAt = now;
  _state = PairingState::handedOff;
  return true;
}
bool PairingSession::handoffComplete(uint64_t now) const {
  return _state == PairingState::handedOff && now - _ackAt >= 500;
}
bool PairingSession::waitingForHandoff(uint64_t now) const {
  return _state == PairingState::codeReady && now - _readyAt < 10000 && remaining(now) > 0;
}
void PairingSession::restore(uint64_t now, uint32_t a, const String &code, const String &server) {
  start(now, server);
  submitAttempt(a);
  wifiResult(true);
  setupResult(code, now);
  _fromBle = true;
  _readyAt = now;
}
bool savePairingIdentity(Persistence &p, const char *key, const PairingIdentity &identity) {
  if (pairingHelpQrFragment(identity).length() == 0) return false;
  JsonDocument d;
  d["v"] = 1;
  d["sid"] = identity.sessionId;
  d["pop"] = identity.proof;
  d["mac"] = identity.mac;
  String blob;
  serializeJson(d, blob);
  if (p.writeString(key, blob.c_str()) != blob.length()) return false;
  return p.readString(key, "") == blob;
}
