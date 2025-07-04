#ifndef HTTP_UTILS_H
#define HTTP_UTILS_H

#include <Arduino.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>

// Error codes for the HTTP utilities - using distinct values to avoid overlap
enum HttpError
{
  HTTPCLIENT_SUCCESS = 100,
  HTTPCLIENT_WIFICLIENT_ERROR = 101, // Failed to create client
  HTTPCLIENT_HTTPCLIENT_ERROR = 102  // Failed to connect
};

/**
 * @brief Higher-order function that sets up WiFiClient and HTTPClient, then runs a callback
 * @param url The initial URL to connect to
 * @param callback Function to call with the HTTPClient pointer and error code
 * @return The value returned by the callback
 */
template <typename Callback, typename ReturnType = decltype(std::declval<Callback>()(nullptr, (HttpError)0))>
ReturnType withHttp(const String &url, Callback callback)
{
  Log_info("==== withHttp() %s", url.c_str());

  bool isHttps = (url.indexOf("https://") != -1);

  // Conditionally allocate only the client we need
  WiFiClient *client = nullptr;

  if (isHttps)
  {
    WiFiClientSecure *secureClient = new WiFiClientSecure();
    secureClient->setInsecure();
    client = secureClient;
  }
  else
  {
    client = new WiFiClient();
  }

  // Check if client creation succeeded
  if (!client)
  {
    return callback(nullptr, HTTPCLIENT_WIFICLIENT_ERROR);
  }

  ReturnType result;
  { // Add a scoping block for HTTPClient https to make sure it is destroyed before WiFiClientSecure *client is

    HTTPClient https;
    if (https.begin(*client, url))
    {
      result = callback(&https, HTTPCLIENT_SUCCESS);
      https.end();
    }
    else
    {
      result = callback(nullptr, HTTPCLIENT_HTTPCLIENT_ERROR);
    }
  }
  delete client;

  return result;
}

#endif // HTTP_UTILS_H