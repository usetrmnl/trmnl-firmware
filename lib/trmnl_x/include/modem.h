#pragma once

#include <Arduino.h>
#include <functional>
#include <vector>

// UART pins (ESP32-S3 to ESP32-C5)
#define AT_UART_TX 43
#define AT_UART_RX 44
#define AT_UART_RTS 1
#define AT_UART_CTS 2

class Modem {
private:
  HardwareSerial ModemSerial;
  uint32_t baudRate;
  String waitForResponse(const String& expected, unsigned long timeoutMs);

public:
  Modem(uint32_t baudRate);
  ~Modem();
  bool sendCommand(const char* command);
  String readResponse(unsigned long timeout);
  bool flashFromFile(const char* filename);
  bool eraseFlash();
  void setSerialBaud(uint32_t baud);

  struct ModemNetwork {
    String  ssid;
    int32_t rssi;
    bool    open;    // true if no encryption (ecn == 0)
    bool    is5GHz;  // true if channel >= 36
  };
  std::vector<ModemNetwork> scanNetworks();

  bool connectToNetwork(const String& ssid, const String& password);
  bool disconnectFromNetwork();

  struct ModemHttpResult {
    bool   ok;
    int    statusCode;
    String body;          // populated in text mode (saveToFile empty)
    size_t bytesReceived; // total payload bytes in either mode
  };
  // saveToFile:    empty = text mode (body in String)
  //               non-empty = binary mode (stream to file, body stays empty)
  // contentLength: 0 = unknown (progress shows bytes + speed only)
  //               >0 = enables percentage in progress output
  // reqHeaders:   newline-separated "Key: Value" headers, e.g. "ID: mac\nKey: val"
  //               Set via AT+HTTPCHEAD before the request; cleared afterwards.
  ModemHttpResult httpGet(const String& url, const String& saveToFile = "", size_t contentLength = 0, const String& reqHeaders = "");
  // Chunk-callback overload: chunkCb is called for each received chunk; return false to abort
  ModemHttpResult httpGet(const String& url, std::function<bool(const uint8_t*, size_t)> chunkCb, size_t contentLength = 0, const String& reqHeaders = "");

  // Configures modem SNTP and returns Unix timestamp (UTC), or 0 on failure.
  time_t getSntpTime();
};
