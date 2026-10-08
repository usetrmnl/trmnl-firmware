#include <HTTPClient.h>
#include <StreamString.h>
#include <WiFi.h>
#include <config.h>
#include <http_client.h>
#include <http_utils.h>
#include <inttypes.h>
#include <services/http_retry_request.h>
#include <stdlib.h>
#include <string.h>
#include <trmnl_log.h>
#include <utility>

#ifdef BOARD_TRMNL_X
#include <WifiCaptive.h>
#include <globals.h>
#include <modem.h>
#endif

namespace {

  // A write-only Stream into a fixed buffer, for HTTPClient::writeToStream(). Writing past its
  // capacity fails, which writeToStream() reports as HTTPC_ERROR_STREAM_WRITE.
  class BodyBufferStream : public Stream {
  public:
    BodyBufferStream(uint8_t *buffer, size_t capacity) : _buffer(buffer), _capacity(capacity) {}

    size_t write(const uint8_t *data, size_t len) override {
      if (len > _capacity - _size) {
        _overflowed = true;
        return 0;
      }
      memcpy(_buffer + _size, data, len);
      _size += len;
      return len;
    }
    size_t write(uint8_t data) override { return write(&data, 1); }
    int available() override { return 0; }
    int read() override { return -1; }
    int peek() override { return -1; }

    size_t size() const { return _size; }
    bool overflowed() const { return _overflowed; }

  private:
    uint8_t *_buffer;
    size_t _capacity;
    size_t _size = 0;
    bool _overflowed = false;
  };

} // namespace

HttpRetryRequest::HttpRetryRequest(const HttpRetryRequestConfig &config) : _config(config) {
  _readTimeoutMs =
    _config.readTimeoutSeconds > 0 ? _config.readTimeoutSeconds * MS_TO_S_FACTOR : HTTP_DEFAULT_TIMEOUT_MS;
}

HttpRetryRequest::~HttpRetryRequest() { releaseBody(); }

// ---- public ---------------------------------------------------------------

https_request_err_e HttpRetryRequest::execute() {
  https_request_err_e err = HTTPS_NO_ERR;
  for (uint8_t attempt = 1; attempt <= HTTP_FAST_RETRY_ATTEMPTS; ++attempt) {
    err = attemptOnce();

    if (err == HTTPS_NO_ERR) {
      return err; // success!
    }

    if (!shouldFastRetryCode(_httpCode)) {
      // Deterministic failure (e.g. 404): another identical request will not help.
      Log_error("HTTP GET failed (code %d, not retried): %s - %s, RSSI %" PRId32, _httpCode, https_request_err_str(err),
                _errorDetail.c_str(), signalRssi());
      return err;
    }

    Log_error_serial("Connection attempt %d/%d failed: %s", attempt, HTTP_FAST_RETRY_ATTEMPTS, _errorDetail.c_str());

    if (attempt < HTTP_FAST_RETRY_ATTEMPTS) {
      delay(HTTP_FAST_RETRY_DELAY_MS);
    }
  }
  Log_error("HTTP GET failed after %d attempts: %s - %s, RSSI %" PRId32, HTTP_FAST_RETRY_ATTEMPTS,
            https_request_err_str(err), _errorDetail.c_str(), signalRssi());
  return err;
}

uint8_t *HttpRetryRequest::body() const {
  if (_bodyBuffer) return _bodyBuffer;
  if (_payload.length() > 0) return (uint8_t *)_payload.c_str();
  return nullptr;
}

String HttpRetryRequest::bodyAsString() const {
  if (_bodyBuffer) {
    // The buffer is NUL-terminated past the body: Arduino 2's concat(buf, len) copies len + 1.
    String s;
    s.reserve(_bodySize);
    s.concat((const char *)_bodyBuffer, _bodySize);
    return s;
  }
  return _payload;
}

// ---- attempt plumbing -----------------------------------------------------

bool HttpRetryRequest::useModem() const {
#ifdef BOARD_TRMNL_X
  return g_modem && WifiCaptivePortal.getLastCredentials().is5GHz;
#else
  return false;
#endif
}

int32_t HttpRetryRequest::signalRssi() const {
#ifdef BOARD_TRMNL_X
  if (useModem()) return g_modem->getSignalRssi();
#endif
  return WiFi.RSSI();
}

void HttpRetryRequest::resetAttemptState() {
  releaseBody();
  _payload = String();
  _bodySize = 0;
  _httpCode = 0;
  _contentType = "";
  _errorDetail = "";
}

void HttpRetryRequest::releaseBody() {
  if (_bodyBuffer) {
    free(_bodyBuffer);
    _bodyBuffer = nullptr;
  }
}

https_request_err_e HttpRetryRequest::attemptOnce() {
  resetAttemptState();
  https_request_err_e err;
#ifdef BOARD_TRMNL_X
  if (useModem()) {
    Log_info("GET via modem (5 GHz path): %s", _config.url.c_str());
    err = attemptModem();
  } else
#endif
  {
    err = attemptWiFi();
  }
  if (err != HTTPS_NO_ERR) return err;
  return finishBody();
}

https_request_err_e HttpRetryRequest::finishBody() {
  if (_bodySize == 0) {
    _errorDetail = "No data received";
    Log_error_serial("Receiving failed. No data received");
    return HTTPS_WRONG_IMAGE_SIZE;
  }
  if (_contentType.length() == 0) {
    const char *sniffed = sniffImageContentType(body(), _bodySize);
    if (sniffed) _contentType = sniffed;
  }
  return HTTPS_NO_ERR;
}

// ---- Wi-Fi transport ------------------------------------------------------

void HttpRetryRequest::applyRequestSettings(HTTPClient &https, const String &hopUrl) {
  https.setTimeout(_readTimeoutMs);
  https.setConnectTimeout(HTTP_DEFAULT_TIMEOUT_MS);
  https.addHeader("Accept-Encoding", "identity"); // plain bytes for both JSON and image bodies
  if (isSameOrigin(hopUrl, _config.apiBaseUrl)) {
    applyHeaders(https, _config.headers);
    logHeaders(_config.headers);
  }
  const char *collect[] = {"Content-Type"};
  https.collectHeaders(collect, 1);
}

https_request_err_e HttpRetryRequest::attemptWiFi() {
  return withHttp(
    _config.url,
    [this](HTTPClient *https, HttpError error) -> https_request_err_e {
      if (error == HttpError::HTTPCLIENT_WIFICLIENT_ERROR || !https) {
        _errorDetail = "Unable to create WiFiClient";
        Log_error_serial("%s", _errorDetail.c_str());
        return HTTPS_UNABLE_TO_CONNECT;
      }
      if (error != HttpError::HTTPCLIENT_SUCCESS) {
        _errorDetail = "Unable to create HTTPClient";
        Log_error_serial("%s", _errorDetail.c_str());
        return HTTPS_UNABLE_TO_CONNECT;
      }

      applyRequestSettings(*https, _config.url);
      Log_info("GET %s", _config.url.c_str());
      int code = https->GET();

      // follow one redirect
      if (code == HTTP_CODE_PERMANENT_REDIRECT || code == HTTP_CODE_TEMPORARY_REDIRECT) {
        String location = https->getLocation();
        https->end();
        String redirectUrl = resolveRedirectLocation(location, httpOriginOf(_config.url));
        Log_info("Redirected to: %s", redirectUrl.c_str());
        https->begin(redirectUrl);
        https->setReuse(false);
        applyRequestSettings(*https, redirectUrl);
        code = https->GET();
      }

      _httpCode = code;
      if (code < 0 ||
          !(code == HTTP_CODE_OK || code == HTTP_CODE_MOVED_PERMANENTLY || code == HTTP_CODE_TOO_MANY_REQUESTS)) {
        _errorDetail = "HTTP Client failed with error: " + https->errorToString(code) + "(" + String(code) + ")";
        Log_error_serial("[HTTPS] GET... failed, error: %s", _errorDetail.c_str());
        return HTTPS_RESPONSE_CODE_INVALID;
      }
      Log_info("GET... code: %d, RSSI: %d", code, WiFi.RSSI());

      _contentType = https->header("Content-Type");
      return readWiFiBody(*https, https->getSize());
    },
    /*resumable=*/true);
}

https_request_err_e HttpRetryRequest::readWiFiBody(HTTPClient &https, int contentLength) {
  unsigned long start = millis();

  if (contentLength <= 0) return readBodyMissingContentLength(https, start);

  uint32_t expected = (uint32_t)contentLength;
  Log_info("Content size: %" PRIu32, expected);
  if (expected > MAX_IMAGE_SIZE) {
    _errorDetail = "file size too big: " + String(expected);
    Log_error_serial("Receiving failed; %s", _errorDetail.c_str());
    return HTTPS_IMAGE_FILE_TOO_BIG;
  }

  _bodyBuffer = (uint8_t *)malloc(expected + 1); // + a NUL, see bodyAsString()
  if (!_bodyBuffer) {
    _errorDetail = "Failed to allocate " + String(expected) + " bytes for body";
    Log_error_serial("%s", _errorDetail.c_str());
    return HTTPS_OUT_OF_MEMORY;
  }

  WiFiClient *stream = https.getStreamPtr();
  uint32_t received = 0;
  bool closedEarly = false;
  unsigned long lastActivity = millis();
  while (received < expected && millis() < lastActivity + IMAGE_STREAM_INACTIVITY_TIMEOUT_MS) {
    int available = stream->available();
    if (available > 0) {
      size_t want = expected - received;
      if ((size_t)available < want) want = available;
      int got = stream->read(_bodyBuffer + received, want);
      if (got > 0) {
        received += got;
        lastActivity = millis();
      }
    } else if (!stream->connected()) {
      // server closed the connection before sending the whole body; no more data can arrive
      closedEarly = true;
      break;
    } else {
      vTaskDelay(1); // yield to allow time for the data to arrive
    }
  }
  stream->stop(); // Important! If you don't do this, WiFi will have a memory exception later

  if (received < expected) {
    _errorDetail = String("incomplete download (") + (closedEarly ? "connection closed early" : "timed out") +
                   "): " + String(received) + "/" + String(expected) + " bytes";
    Log_error_serial("Receiving failed; %s, RSSI %d", _errorDetail.c_str(), WiFi.RSSI());
    releaseBody();
    return HTTPS_TIMED_OUT;
  }

  _bodyBuffer[received] = '\0';
  _bodySize = received;
  Log_info("%" PRIu32 " bytes received in %lu ms", _bodySize, millis() - start);
  return HTTPS_NO_ERR;
}

https_request_err_e HttpRetryRequest::readBodyMissingContentLength(HTTPClient &https, unsigned long start) {
  Log_info("Content-Length not provided, using writeToStream()");

  // Attempt to read into a single max-size buffer first, to save on allocations.
  _bodyBuffer = (uint8_t *)malloc(MAX_IMAGE_SIZE + 1); // + a NUL, see bodyAsString()
  if (_bodyBuffer) return readBodyIntoBuffer(https, start);

  // No room for the max-size buffer; a small body may still fit as it grows.
  return readBodyIntoStreamString(https, start);
}

// Fills _bodyBuffer, already allocated at MAX_IMAGE_SIZE + 1, then shrinks it to fit.
https_request_err_e HttpRetryRequest::readBodyIntoBuffer(HTTPClient &https, unsigned long start) {
  BodyBufferStream sink(_bodyBuffer, MAX_IMAGE_SIZE);
  int written = https.writeToStream(&sink);
  if (written < 0) {
    releaseBody();
    if (sink.overflowed()) {
      _errorDetail = "file size too big: over " + String(MAX_IMAGE_SIZE);
      Log_error_serial("Receiving failed; %s", _errorDetail.c_str());
      return HTTPS_IMAGE_FILE_TOO_BIG;
    }
    _errorDetail =
      "connection closed mid-download, error: " + String(written) + " (" + https.errorToString(written) + ")";
    Log_error_serial("Receiving failed; %s, RSSI %d", _errorDetail.c_str(), WiFi.RSSI());
    return HTTPS_TIMED_OUT;
  }
  _bodySize = sink.size();
  _bodyBuffer[_bodySize] = '\0';
  uint8_t *fitted = (uint8_t *)realloc(_bodyBuffer, _bodySize + 1); // give back the rest
  if (fitted) _bodyBuffer = fitted;
  Log_info("%" PRIu32 " bytes received in %lu ms", _bodySize, millis() - start);
  return HTTPS_NO_ERR;
}

https_request_err_e HttpRetryRequest::readBodyIntoStreamString(HTTPClient &https, unsigned long start) {
  StreamString sstream;
  int written = https.writeToStream(&sstream);
  if (written < 0) {
    _errorDetail =
      "connection closed mid-download, error: " + String(written) + " (" + https.errorToString(written) + ")";
    Log_error_serial("Receiving failed; %s, RSSI %d", _errorDetail.c_str(), WiFi.RSSI());
    return HTTPS_TIMED_OUT;
  }
  _payload = std::move(static_cast<String &>(sstream));
  _bodySize = _payload.length();
  Log_info("%" PRIu32 " bytes received in %lu ms", _bodySize, millis() - start);
  return HTTPS_NO_ERR;
}

// ---- modem transport (TRMNL X, 5 GHz) --------------------------------------

#ifdef BOARD_TRMNL_X
https_request_err_e HttpRetryRequest::attemptModem() {
  if (!g_modem) {
    _errorDetail = "modem not available";
    return HTTPS_UNABLE_TO_CONNECT;
  }

  String reqHeaders;
  if (isSameOrigin(_config.url, _config.apiBaseUrl)) reqHeaders = formatHeaders(_config.headers);

  uint32_t capacity = 0;
  bool tooBig = false;
  bool outOfMemory = false;
  unsigned long timeoutMs = _readTimeoutMs > HTTP_MODEM_TIMEOUT_MS ? _readTimeoutMs : HTTP_MODEM_TIMEOUT_MS;

  auto res = g_modem->httpGet(
    _config.url,
    [&](const uint8_t *data, size_t len) -> bool {
      if (_bodySize + len > MAX_IMAGE_SIZE) {
        tooBig = true;
        return false;
      }
      // one byte more than the body for its NUL, see bodyAsString()
      if (_bodySize + len + 1 > capacity) {
        uint32_t next = capacity ? capacity * 2 : 16384;
        while (next < _bodySize + len + 1)
          next *= 2;
        if (next > MAX_IMAGE_SIZE + 1) next = MAX_IMAGE_SIZE + 1;
        uint8_t *grown = (uint8_t *)realloc(_bodyBuffer, next);
        if (!grown) {
          outOfMemory = true;
          return false;
        }
        _bodyBuffer = grown;
        capacity = next;
      }
      memcpy(_bodyBuffer + _bodySize, data, len);
      _bodySize += len;
      _bodyBuffer[_bodySize] = '\0';
      return true;
    },
    0, reqHeaders, timeoutMs);

  _httpCode = res.statusCode;
  if (tooBig) {
    _errorDetail = "file size too big: more than " + String(MAX_IMAGE_SIZE) + " bytes";
    Log_error_serial("Receiving failed; %s", _errorDetail.c_str());
    return HTTPS_IMAGE_FILE_TOO_BIG;
  }
  if (outOfMemory) {
    _errorDetail = "Failed to grow body buffer past " + String(_bodySize) + " bytes";
    Log_error_serial("%s", _errorDetail.c_str());
    return HTTPS_OUT_OF_MEMORY;
  }
  if (!res.ok) {
    _errorDetail = "modem HTTP status " + String(res.statusCode) + ", " + String(res.bytesReceived) + " bytes received";
    Log_error_serial("Modem GET failed: %s", _errorDetail.c_str());
    return HTTPS_RESPONSE_CODE_INVALID;
  }
  Log_info("Modem GET ok: %" PRIu32 " bytes", _bodySize);
  return HTTPS_NO_ERR;
}
#endif // BOARD_TRMNL_X
