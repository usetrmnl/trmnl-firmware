// T3 — tap-path /api/display fetch over raw esp_tls with TLS session
// resumption across deep sleep.
//
// Why this exists: the normal HTTPClient/WiFiClientSecure path does a full TLS
// handshake on every wake (cert chain + signature verify, ~0.9-1.5 s). The
// TRMNL API edge (Caddy, trmnl.app) issues resumable TLS 1.2 session tickets
// (verified: `openssl s_client -tls1_2 -sess_out/-sess_in` -> "Reused").
// Arduino's WiFiClientSecure exposes no session save/restore hook, so this
// path drops to esp_tls, which does. The mbedtls session is flattened with
// mbedtls_ssl_session_save() into RTC RAM (survives deep sleep) and reloaded on
// the next tap for a 1-RTT resumed handshake.
//
// Scope guardrail: tap path only, and it returns HTTPS_NO_ERR *only* on a clean
// 200. Any redirect, non-200, chunked body, or transport error returns an error
// so the caller falls back to fetchApiDisplay() (the proven HTTPClient path).
// Worst case is one wasted connect, then the normal path runs.

#include <api-client/display.h>
#include <api-client/request_headers.h>
#include <api_response_parsing.h>
#include <trmnl_log.h>
#include <Arduino.h>
#include <esp_attr.h>

extern void trace(const char *name); // timing checkpoints (bl.cpp)

#if defined(CONFIG_ESP_TLS_CLIENT_SESSION_TICKETS)

#include <esp_tls.h>
#include <mbedtls/ssl.h>
#include <Preferences.h>
#include <WiFi.h>

// Serialized mbedtls session. 2 KB comfortably holds a TLS 1.2 ECDSA session;
// MBEDTLS_SSL_KEEP_PEER_CERTIFICATE=y means the peer cert rides along (~1 KB for
// trmnl.app). If a session ever serializes larger than this, we simply skip
// saving it (next tap does a full handshake).
//
// Persisted two ways: RTC RAM (fast, survives deep sleep) as the primary, and
// NVS/flash as a backup that ALSO survives a full power-cycle — RTC RAM is
// cleared on reset, so NVS is what lets resumption work when testing over USB
// (which resets the chip on every serial (re)connect).
static constexpr size_t TLS_SESSION_MAX = 2048;
RTC_DATA_ATTR static uint8_t g_tls_session[TLS_SESSION_MAX];
RTC_DATA_ATTR static size_t g_tls_session_len = 0;

namespace {

constexpr const char *NVS_NS = "tls_sess";
constexpr const char *NVS_KEY = "s";

// On a power-cycle RTC RAM is zeroed; pull the last session back from NVS so a
// resumed handshake is still possible. No-op when RTC RAM already has one.
void loadSessionFromNvsIfNeeded()
{
  if (g_tls_session_len > 0)
    return;
  Preferences p;
  if (!p.begin(NVS_NS, /*readOnly=*/true))
    return;
  size_t n = p.getBytesLength(NVS_KEY);
  if (n > 0 && n <= TLS_SESSION_MAX)
    g_tls_session_len = p.getBytes(NVS_KEY, g_tls_session, n);
  p.end();
  if (g_tls_session_len > 0)
    Log_info("[resumable] loaded session from NVS: %d bytes", (int)g_tls_session_len);
}

// ponytail: writes ~1.1 KB to NVS on every successful fetch (tickets rotate, so
// it changes each time). Fine for bench testing; gate behind a dev flag or drop
// the NVS half before merge if flash wear matters at production wake rates.
void saveSessionToNvs()
{
  Preferences p;
  if (!p.begin(NVS_NS, /*readOnly=*/false))
    return;
  if (g_tls_session_len > 0)
    p.putBytes(NVS_KEY, g_tls_session, g_tls_session_len);
  else
    p.remove(NVS_KEY);
  p.end();
}

ApiDisplayResult errResult(https_request_err_e err, const String &detail)
{
  Log_info("[resumable] fallback: %s", detail.c_str());
  return ApiDisplayResult{.error = err, .response = {}, .error_detail = detail};
}

// Parse "https://host[:port][/...]" into host + port. Returns false for non-https.
bool parseUrl(const String &url, String &host, int &port)
{
  const int schemeLen = 8; // "https://"
  if (!url.startsWith("https://"))
    return false;
  int start = schemeLen;
  int end = url.indexOf('/', start);
  String authority = (end < 0) ? url.substring(start) : url.substring(start, end);
  int colon = authority.indexOf(':');
  if (colon < 0) {
    host = authority;
    port = 443;
  } else {
    host = authority.substring(0, colon);
    port = authority.substring(colon + 1).toInt();
    if (port <= 0) port = 443;
  }
  return host.length() > 0;
}

} // namespace

ApiDisplayResult fetchApiDisplayResumable(ApiDisplayInputs &apiDisplayInputs)
{
  String host;
  int port = 443;
  if (!parseUrl(apiDisplayInputs.baseUrl, host, port))
    return errResult(HTTPS_UNABLE_TO_CONNECT, "non-https base url");

  // Build the raw request. buildDisplayHeaders() carries no Host / body, so we
  // add Host + Connection: close ourselves. Sensor headers (SENSOR_SDA builds)
  // are intentionally not replicated here; they matter on timer wakes, which
  // use the full path.
  String req = "GET /api/display HTTP/1.1\r\nHost: " + host + "\r\n";
  HttpHeaderList headers = buildDisplayHeaders(apiDisplayInputs);
  for (const auto &h : headers)
    req += h.first + ": " + h.second + "\r\n";
  req += "Connection: close\r\n\r\n";

  esp_tls_cfg_t cfg = {};
  cfg.timeout_ms = 15000;
  // Do NOT set skip_common_name: esp_tls maps skip_common_name -> set_hostname(NULL),
  // which drops the SNI extension. Caddy (trmnl.app) routes by SNI and resets the
  // handshake without it. Leaving it false makes esp_tls send SNI = host; cert
  // verification is still skipped because no CA is configured and the build sets
  // CONFIG_ESP_TLS_SKIP_SERVER_CERT_VERIFY (authmode VERIFY_NONE) — matches setInsecure().

  // Restore a saved session, if any, to attempt a resumed handshake.
  loadSessionFromNvsIfNeeded(); // repopulate RTC RAM after a power-cycle
  esp_tls_client_session_t restored;
  bool haveRestored = false;
  if (g_tls_session_len > 0) {
    mbedtls_ssl_session_init(&restored.saved_session);
    if (mbedtls_ssl_session_load(&restored.saved_session, g_tls_session, g_tls_session_len) == 0) {
      cfg.client_session = &restored;
      haveRestored = true;
    } else {
      mbedtls_ssl_session_free(&restored.saved_session);
      g_tls_session_len = 0; // corrupt/incompatible -> drop it
    }
  }

  esp_tls_t *tls = esp_tls_init();
  if (!tls) {
    if (haveRestored) mbedtls_ssl_session_free(&restored.saved_session);
    return errResult(HTTPS_UNABLE_TO_CONNECT, "esp_tls_init failed");
  }

  trace("api tls+GET begin");
  Log_info("[resumable] connecting %s:%d (resumed=%d, heap=%d)", host.c_str(), port,
           (int)haveRestored, (int)ESP.getFreeHeap());
  // Resolve the host ourselves via lwip's plain A-record path (same one the
  // HTTPClient route uses, measured ~30 ms) instead of letting esp_tls run
  // getaddrinfo, and connect by IP. cfg.common_name keeps SNI = real hostname
  // (esp_tls passes it to mbedtls_ssl_set_hostname), which Caddy needs for
  // routing. Also gives us a dns/tcp+tls trace split.
  trace("resumable dns begin");
  IPAddress addr;
  bool dnsOk = WiFi.hostByName(host.c_str(), addr) == 1;
  trace("resumable dns end");
  String connectTo = dnsOk ? addr.toString() : host;
  if (dnsOk)
    cfg.common_name = host.c_str();
  int conn = esp_tls_conn_new_sync(connectTo.c_str(), connectTo.length(), port, &cfg, tls);
  trace(conn == 1 ? "tls connected" : "tls connect fail");
  if (haveRestored) mbedtls_ssl_session_free(&restored.saved_session); // esp_tls copied it into its own ssl
  if (conn != 1) {
    int tls_code = 0, tls_flags = 0;
    esp_tls_error_handle_t eh = nullptr;
    if (esp_tls_get_error_handle(tls, &eh) == ESP_OK && eh)
      esp_tls_get_and_clear_last_error(eh, &tls_code, &tls_flags);
    esp_tls_conn_destroy(tls);
    trace("api tls+GET end");
    return errResult(HTTPS_UNABLE_TO_CONNECT,
                     "tls connect failed (resumed=" + String(haveRestored) +
                     ", esp_tls_code=-0x" + String(-tls_code, HEX) +
                     ", flags=0x" + String(tls_flags, HEX) + ")");
  }

  // Send request.
  const char *out = req.c_str();
  size_t total = req.length(), sent = 0;
  while (sent < total) {
    ssize_t w = esp_tls_conn_write(tls, out + sent, total - sent);
    if (w > 0) { sent += w; continue; }
    if (w == ESP_TLS_ERR_SSL_WANT_READ || w == ESP_TLS_ERR_SSL_WANT_WRITE) { delay(1); continue; }
    esp_tls_conn_destroy(tls);
    trace("api tls+GET end");
    return errResult(HTTPS_UNABLE_TO_CONNECT, "tls write failed");
  }

  // Read the whole response (Connection: close -> server closes at EOF).
  String resp;
  resp.reserve(2048);
  uint8_t buf[512];
  uint32_t start = millis();
  bool overflow = false;
  while (millis() - start < 15000) {
    ssize_t rd = esp_tls_conn_read(tls, buf, sizeof(buf));
    if (rd > 0) {
      if (resp.length() == 0) trace("resumable first byte");
      resp.concat(reinterpret_cast<const char *>(buf), rd);
      if (resp.length() > 32768) { overflow = true; break; } // /api/display JSON is tiny; bail if abnormal
      continue;
    }
    if (rd == 0) break; // clean close = done
    if (rd == ESP_TLS_ERR_SSL_WANT_READ || rd == ESP_TLS_ERR_SSL_WANT_WRITE) { delay(1); continue; }
    break; // error/close
  }
  trace("api tls+GET end");

  // Grab and persist the session ticket for the next tap before tearing down.
  // Caddy sends a NewSessionTicket only on FULL handshakes; a resumed handshake
  // yields a session without one. TLS 1.2 tickets are multi-use against Caddy
  // (verified with openssl -sess_in twice), so on a resumed connect we keep the
  // existing blob instead of clobbering it with a ticket-less session. If the
  // ticket ever expires, the server falls back to a full handshake and issues a
  // fresh ticket, which lands in the "save" branch — no stale-ticket lock-in.
  esp_tls_client_session_t *sess = esp_tls_get_client_session(tls);
  if (sess) {
    // ticket/ticket_len are private in mbedtls 3.x; MBEDTLS_PRIVATE is the
    // sanctioned accessor macro (there is no public "was a ticket issued" API).
    bool hasTicket = sess->saved_session.MBEDTLS_PRIVATE(ticket) &&
                     sess->saved_session.MBEDTLS_PRIVATE(ticket_len) > 0;
    if (hasTicket) {
      size_t olen = 0;
      int sr = mbedtls_ssl_session_save(&sess->saved_session, g_tls_session, sizeof(g_tls_session), &olen);
      g_tls_session_len = (sr == 0) ? olen : 0; // too-big or unsupported -> no resume next time
      saveSessionToNvs(); // also persist to flash so it survives a power-cycle
      Log_info("[resumable] session saved: %d bytes (rc=%d)", (int)g_tls_session_len, sr);
    } else {
      Log_info("[resumable] resumed, no new ticket — keeping saved session");
    }
    esp_tls_free_client_session(sess);
  }
  esp_tls_conn_destroy(tls);

  if (overflow)
    return errResult(HTTPS_RESPONSE_CODE_INVALID, "response too large");

  // Split status line / headers / body.
  int headerEnd = resp.indexOf("\r\n\r\n");
  if (headerEnd < 0)
    return errResult(HTTPS_RESPONSE_CODE_INVALID, "no header terminator");
  String head = resp.substring(0, headerEnd);
  String body = resp.substring(headerEnd + 4);

  // Status code from "HTTP/1.1 200 OK".
  int sp = head.indexOf(' ');
  int status = (sp >= 0) ? head.substring(sp + 1, sp + 4).toInt() : 0;
  if (status != 200)
    return errResult(HTTPS_RESPONSE_CODE_INVALID, "status " + String(status));

  // Chunked bodies aren't de-chunked here; hand those to the full path.
  String headLower = head;
  headLower.toLowerCase();
  if (headLower.indexOf("transfer-encoding: chunked") >= 0)
    return errResult(HTTPS_RESPONSE_CODE_INVALID, "chunked body");

  trace("api body read end");
  Log_info("[resumable] 200 OK, body %d bytes", body.length());

  auto apiResponse = parseResponse_apiDisplay(body);
  if (apiResponse.outcome == ApiDisplayOutcome::DeserializationError)
    return ApiDisplayResult{.error = HTTPS_JSON_PARSING_ERR, .response = {},
                            .error_detail = "JSON parse failed: " + apiResponse.error_detail};

  return ApiDisplayResult{.error = HTTPS_NO_ERR, .response = apiResponse, .error_detail = ""};
}

#else // !CONFIG_ESP_TLS_CLIENT_SESSION_TICKETS

// Session tickets not enabled in this build's sdkconfig -> always signal
// fallback so the caller uses fetchApiDisplay(). (Set
// CONFIG_ESP_TLS_CLIENT_SESSION_TICKETS=y in the sdkconfig to enable T3.)
ApiDisplayResult fetchApiDisplayResumable(ApiDisplayInputs &apiDisplayInputs)
{
  return ApiDisplayResult{.error = HTTPS_UNABLE_TO_CONNECT, .response = {},
                          .error_detail = "resumable TLS disabled in build"};
}

#endif // CONFIG_ESP_TLS_CLIENT_SESSION_TICKETS
