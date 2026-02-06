#include <api-client/time.h>
#include <HTTPClient.h>
#include <trmnl_log.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <http_client.h>

TimeApiResult fetchServerTime(const String &baseUrl)
{
  return withHttp(
      baseUrl + "/api/time",
      [](HTTPClient *https, HttpError error) -> TimeApiResult
      {
        if (error == HttpError::HTTPCLIENT_WIFICLIENT_ERROR ||
            error == HttpError::HTTPCLIENT_HTTPCLIENT_ERROR)
        {
          Log_error("Unable to connect for time fetch");
          return TimeApiResult{
              .success = false,
              .timestamp_ms = 0,
              .error = "Unable to connect",
          };
        }

        // Short timeout for time fetch - it should be fast
        https->setTimeout(5000);
        https->setConnectTimeout(5000);

        int httpCode = https->GET();

        if (httpCode != HTTP_CODE_OK)
        {
          Log_error("[Time API] GET failed, code: %d", httpCode);
          return TimeApiResult{
              .success = false,
              .timestamp_ms = 0,
              .error = "HTTP error: " + String(httpCode),
          };
        }

        String payload = https->getString();
        Log_info("[Time API] Response: %s", payload.c_str());

        JsonDocument doc;
        DeserializationError jsonError = deserializeJson(doc, payload);

        if (jsonError)
        {
          Log_error("[Time API] JSON parse error: %s", jsonError.c_str());
          return TimeApiResult{
              .success = false,
              .timestamp_ms = 0,
              .error = "JSON parse error",
          };
        }

        uint64_t timestamp = doc["timestamp_ms"].as<uint64_t>();

        if (timestamp == 0)
        {
          Log_error("[Time API] Invalid timestamp in response");
          return TimeApiResult{
              .success = false,
              .timestamp_ms = 0,
              .error = "Invalid timestamp",
          };
        }

        Log_info("[Time API] Server time: %llu", timestamp);
        return TimeApiResult{
            .success = true,
            .timestamp_ms = timestamp,
            .error = "",
        };
      });
}
