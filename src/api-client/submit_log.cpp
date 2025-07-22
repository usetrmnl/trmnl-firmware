#include "api-client/submit_log.h"
#include <stdio.h>
#include "trmnl_log.h"
#include <memory>
#include "http_client.h"
#include <api_request_serialization.h>

bool submitLogToApi(LogApiInput &input, const char *api_url)
{
  String payload = serializeApiLogRequest(input.log_buffer);
  Log_info("[HTTPS] begin /api/log ...");

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
                    https.addHeader("Accept", "application/json");
                    https.addHeader("Access-Token", input.api_key);
                    https.addHeader("Content-Type", "application/json");

                    https.setTimeout(15000);
                    https.setConnectTimeout(15000);

                    Log_info("Send log - |%s|", payload.c_str());

                    // start connection and send HTTP header
                    int httpCode = https.POST(payload);

                    // httpCode will be negative on error
                    if (httpCode < 0)
                    {
                      Log_error("[HTTPS] POST... failed, error: %d %s", httpCode, https.errorToString(httpCode).c_str());
                      return false;
                    }
                    else if (httpCode != HTTP_CODE_OK && httpCode != HTTP_CODE_MOVED_PERMANENTLY && httpCode != HTTP_CODE_NO_CONTENT)
                    {
                      Log_error("[HTTPS] POST... failed, returned HTTP code unknown: %d %s", httpCode, https.errorToString(httpCode).c_str());
                      return false;
                    }

                    // HTTP header has been send and Server response header has been handled
                    Log_info("[HTTPS] POST OK, code: %d", httpCode);

                    return true; });
}
