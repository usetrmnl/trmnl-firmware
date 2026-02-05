#ifndef HTTP_UTILS_H
#define HTTP_UTILS_H

#include <Arduino.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <SPIFFS.h>

#define CLIENT_CERT_PATH "/client_cert.pem"
#define CLIENT_KEY_PATH "/client_key.pem"

// Error codes for the HTTP utilities - using distinct values to avoid overlap
enum HttpError
{
  HTTPCLIENT_SUCCESS = 100,
  HTTPCLIENT_WIFICLIENT_ERROR = 101, // Failed to create client
  HTTPCLIENT_HTTPCLIENT_ERROR = 102  // Failed to connect
};

/**
 * @brief Load a PEM file from SPIFFS into a heap-allocated buffer
 * @param path SPIFFS file path
 * @param out_len Output: length of the loaded data
 * @return Heap-allocated buffer (caller must free), or nullptr on failure
 */
inline char *loadPemFile(const char *path, size_t &out_len)
{
  if (!SPIFFS.exists(path))
    return nullptr;

  File f = SPIFFS.open(path, FILE_READ);
  if (!f)
    return nullptr;

  size_t len = f.size();
  if (len == 0)
  {
    f.close();
    return nullptr;
  }

  char *buf = (char *)malloc(len + 1);
  if (!buf)
  {
    f.close();
    return nullptr;
  }

  f.readBytes(buf, len);
  buf[len] = '\0';
  f.close();
  out_len = len;
  return buf;
}

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
  char *certBuf = nullptr;
  char *keyBuf = nullptr;

  if (isHttps)
  {
    WiFiClientSecure *secureClient = new WiFiClientSecure();
    secureClient->setInsecure();

    // Load client certificate for mTLS if available
    size_t certLen = 0, keyLen = 0;
    certBuf = loadPemFile(CLIENT_CERT_PATH, certLen);
    keyBuf = loadPemFile(CLIENT_KEY_PATH, keyLen);

    if (certBuf && keyBuf)
    {
      secureClient->setCertificate(certBuf);
      secureClient->setPrivateKey(keyBuf);
      Log_info("withHttp: Client certificate loaded for mTLS");
    }

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
      https.setReuse(false);  // Disable keep-alive to ensure connection closes properly
      result = callback(&https, HTTPCLIENT_SUCCESS);
      https.end();
    }
    else
    {
      result = callback(nullptr, HTTPCLIENT_HTTPCLIENT_ERROR);
    }
  }
  client->stop();  // Explicitly close TCP socket before deleting
  delete client;

  // Free PEM buffers
  free(certBuf);
  free(keyBuf);

  return result;
}

#endif // HTTP_UTILS_H