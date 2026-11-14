#pragma once

#include <Arduino.h>
#include <HTTPClient.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <trmnl_log.h>

#include "api-client/request_headers.h"
#include "resumable_wifi_client_secure.h"

// Error codes for the HTTP utilities - using distinct values to avoid overlap
enum HttpError {
  HTTPCLIENT_SUCCESS = 100,
  HTTPCLIENT_WIFICLIENT_ERROR = 101, // Failed to create client
  HTTPCLIENT_HTTPCLIENT_ERROR = 102  // Failed to connect
};

uint32_t downloadStream(WiFiClient *stream, int content_size, uint8_t *buffer);

// A plain or TLS client for url's scheme; resumable resumes the trmnl.app TLS session across deep sleep
inline WiFiClient *newWiFiClientFor(const String &url, bool resumable) {
  if (url.indexOf("https://") == -1) return new WiFiClient();
  WiFiClientSecure *secureClient =
    resumable && url.indexOf("https://trmnl.app") != -1 ? new ResumableWiFiClientSecure() : new WiFiClientSecure();
  secureClient->setInsecure();
  return secureClient;
}

/**
 * @brief Higher-order function that sets up WiFiClient and HTTPClient, then runs a callback
 * @param url The initial URL to connect to
 * @param callback Function to call with the HTTPClient pointer and error code
 * @return The value returned by the callback
 */
template <typename Callback, typename ReturnType = decltype(std::declval<Callback>()(nullptr, (HttpError)0))>
ReturnType withHttp(const String &url, Callback callback, bool resumable = false) {
  Log_info("==== withHttp() %s", url.c_str());

  WiFiClient *client = newWiFiClientFor(url, resumable);

  // Check if client creation succeeded
  if (!client) {
    return callback(nullptr, HTTPCLIENT_WIFICLIENT_ERROR);
  }

  ReturnType result;
  { // Add a scoping block for HTTPClient https to make sure it is destroyed before WiFiClientSecure *client is

    HTTPClient https;
    if (https.begin(*client, url)) {
      https.setReuse(false);  // Disable keep-alive to ensure connection closes properly
      result = callback(&https, HTTPCLIENT_SUCCESS);
      https.end();
    } else {
      result = callback(nullptr, HTTPCLIENT_HTTPCLIENT_ERROR);
    }
  }
  client->stop();  // Explicitly close TCP socket before deleting
  delete client;

  return result;
}

/**
 * @brief Apply a list of request headers to an HTTPClient (Wi-Fi path).
 *
 * The header list is built by the transport-agnostic builders in
 * request_headers.h; this is the HTTPClient adapter for it.
 */
void applyHeaders(HTTPClient &https, const HttpHeaderList &headers);