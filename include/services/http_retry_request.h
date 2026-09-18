#pragma once

#include <Arduino.h>
#include <api-client/request_headers.h>
#include <types.h>

#define HTTP_DEFAULT_TIMEOUT_MS  15000 // Wi-Fi connect and read timeout
#define HTTP_MODEM_TIMEOUT_MS    60000 // TRMNL X 5 GHz path (AT+HTTPCLIENT)
#define HTTP_FAST_RETRY_ATTEMPTS 5     // whole request is retried on any error
#define HTTP_FAST_RETRY_DELAY_MS 2000

class HTTPClient;

// Inputs for one HTTP GET. Everything else (attempts, timeouts, redirect
// handling, accepted status codes, body limits, transport selection) is fixed
// policy inside HttpRetryRequest so the API fetch and the image fetch behave
// the same way.
struct HttpRetryRequestConfig {
  String url;                      // what to GET
  String apiBaseUrl;               // `headers` are sent only to hops whose URL starts with this
  HttpHeaderList headers;          // API auth/metadata headers (ID, Access-Token, ...)
  uint32_t readTimeoutSeconds = 0; // 0 = HTTP_DEFAULT_TIMEOUT_MS
};

// One HTTP GET with retries, over Wi-Fi (HTTPClient) or, on TRMNL X joined to
// a 5 GHz network, the modem's AT+HTTPCLIENT. The body is owned by this object
// and stays valid until it is destroyed.
class HttpRetryRequest {
public:
  explicit HttpRetryRequest(const HttpRetryRequestConfig &config);
  ~HttpRetryRequest();
  HttpRetryRequest(const HttpRetryRequest &) = delete; // body() aliases internal storage
  HttpRetryRequest &operator=(const HttpRetryRequest &) = delete;

  // Single entry point. HTTPS_NO_ERR means a non-empty body is available.
  https_request_err_e execute();

  int httpCode() const { return _httpCode; }
  const String &contentType() const { return _contentType; } // response header, or sniffed from the body, or empty
  const String &errorDetail() const { return _errorDetail; }
  uint8_t *body() const; // writable in place; valid until destruction
  uint32_t bodySize() const { return _bodySize; }
  String bodyAsString() const; // copy, for JSON callers

private:
  bool useModem() const;
  https_request_err_e attemptOnce();
  https_request_err_e attemptWiFi();
  https_request_err_e readWiFiBody(HTTPClient &https, int contentLength);
  void applyRequestSettings(HTTPClient &https, const String &hopUrl);
#ifdef BOARD_TRMNL_X
  https_request_err_e attemptModem();
#endif
  https_request_err_e finishBody();
  void resetAttemptState();
  void releaseBody();
  int32_t signalRssi() const;

  HttpRetryRequestConfig _config;
  uint32_t _readTimeoutMs;
  int _httpCode = 0;
  String _contentType;
  String _errorDetail;
  String _payload;             // owns the body on the Wi-Fi no-Content-Length path
  uint8_t *_bodyBuffer = nullptr; // owns the body on the Wi-Fi sized path and the modem path
  uint32_t _bodySize = 0;
};
