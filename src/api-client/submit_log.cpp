#include "api-client/submit_log.h"
#include <stdio.h>
#include "trmnl_log.h"
#include <memory>
#include "http_client.h"
#include <api_request_serialization.h>
#include <config.h>
#include <api-client/time.h>
#include <device_identity.h>
#include <auth_signature.h>

// Pre-computed Ed25519 auth headers (computed before withHttp to avoid nesting)
struct Ed25519LogHeaders {
  bool present;
  String publicKey;
  String signature;
  String timestamp;
};

static void addEd25519Headers(HTTPClient &https, const Ed25519LogHeaders &auth)
{
  if (auth.present)
  {
    https.addHeader("X-Public-Key", auth.publicKey);
    https.addHeader("X-Signature", auth.signature);
    https.addHeader("X-Timestamp", auth.timestamp);
    Log_info("Added Ed25519 auth headers for log");
  }
}

// Pre-compute Ed25519 auth headers BEFORE opening the main HTTP connection
static Ed25519LogHeaders computeEd25519LogHeaders(LogApiInput &input, const String &baseUrl)
{
  if (input.authMode != PREFERENCES_AUTH_MODE_ED25519 ||
      input.identity == nullptr ||
      !input.identity->initialized)
  {
    return {.present = false};
  }

  Log_info("Pre-computing Ed25519 authentication for log submission");
  auto timeResult = fetchServerTime(baseUrl);
  if (!timeResult.success)
  {
    Log_error("Failed to fetch server time for log: %s", timeResult.error.c_str());
    return {.present = false};
  }

  auto sig = computeAuthSignature(*input.identity, timeResult.timestamp_ms);
  if (!sig.valid)
  {
    Log_error("Failed to compute Ed25519 signature for log");
    return {.present = false};
  }

  return {
    .present = true,
    .publicKey = publicKeyToHex(*input.identity),
    .signature = signatureToHex(sig),
    .timestamp = timestampToString(sig.timestamp_ms),
  };
}

bool submitLogToApi(LogApiInput &input, const char *api_url)
{
  String payload = serializeApiLogRequest(input.log_buffer);
  Log_info("[HTTPS] begin /api/log ...");

  // Pre-compute auth headers BEFORE withHttp to avoid nested HTTP connections
  auto ed25519Auth = computeEd25519LogHeaders(input, String(api_url));

  char new_url[200];
  strcpy(new_url, api_url);
  strcat(new_url, "/api/log");

  return withHttp(new_url, [&](HTTPClient *httpsPointer, HttpError errorCode) -> bool
                  {
                    if (errorCode != HttpError::HTTPCLIENT_SUCCESS || !httpsPointer)
                    {
                      Log_error("[HTTPS] Unable to connect");
                      return false;
                    }

                    Log_info("[HTTPS] POST...");

                    HTTPClient &https = *httpsPointer;

                    https.addHeader("ID", WiFi.macAddress());
                    https.addHeader("Accept", "application/json, */*");
                    https.addHeader("Access-Token", input.api_key);
                    https.addHeader("Content-Type", "application/json");

                    addEd25519Headers(https, ed25519Auth);

                    https.setTimeout(15000);
                    https.setConnectTimeout(15000);

                    Log_info("Send log - |%s|", payload.c_str());

                    // start connection and send HTTP header
                    int httpCode = https.POST(payload);
                    if(httpCode == HTTP_CODE_PERMANENT_REDIRECT || httpCode == HTTP_CODE_TEMPORARY_REDIRECT){
                      https.end();
                      https.begin(String(api_url) + https.getLocation());
                      https.addHeader("ID", WiFi.macAddress());
                      https.addHeader("Accept", "application/json, */*");
                      https.addHeader("Access-Token", input.api_key);
                      https.addHeader("Content-Type", "application/json");
                      addEd25519Headers(https, ed25519Auth);

                      https.setTimeout(15000);
                      https.setConnectTimeout(15000);
                      httpCode = https.POST(payload);
                    }

                    // httpCode will be negative on error
                    if (httpCode < 0)
                    {
                      Log_error("[HTTPS] POST... failed, error: %d %s", httpCode, https.errorToString(httpCode).c_str());
                      return false;
                    }
                    else if (httpCode != HTTP_CODE_OK &&
                             httpCode != HTTP_CODE_MOVED_PERMANENTLY &&
                             httpCode != HTTP_CODE_NO_CONTENT)
                    {
                      Log_error("[HTTPS] POST... failed, returned HTTP code unknown: %d %s", httpCode, https.errorToString(httpCode).c_str());
                      return false;
                    }

                    // HTTP header has been send and Server response header has been handled
                    Log_info("[HTTPS] POST OK, code: %d", httpCode);

                    return true; });
}
