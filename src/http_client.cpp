#include <ArduinoLog.h>
#include <http_client.h>

// See lib/trmnl/include/api-client/request_headers.h for how headers are built
void applyHeaders(HTTPClient &https, const HttpHeaderList &headers) {
  for (const auto &header : headers)
    https.addHeader(header.first, header.second);
}

uint32_t downloadStream(WiFiClient *stream, int content_size, uint8_t *buffer) {
  int iteration_counter = 0;
  int counter2 = content_size;
  unsigned long download_start = millis();
  unsigned long last_data_time = millis();
  int counter = 0;

  while (counter < content_size && millis() - download_start < 30000) {
    if (stream->available()) {
      Log.info("%s [%d]: Downloading... Available bytes: %d\r\n", __FILE__, __LINE__, stream->available());
      int bytes_to_read = min(stream->available(), counter2 - counter);
      counter += stream->readBytes(buffer + counter, bytes_to_read);
      iteration_counter++;
      last_data_time = millis();
    } else if (!stream->connected() || millis() - last_data_time > 5000) {
      break;
    }
    delay(10);
  }

  Log_info("Download end: %d/%d bytes in %lu ms (%d iterations)", counter, content_size, millis() - download_start,
           iteration_counter);
  return counter;
}