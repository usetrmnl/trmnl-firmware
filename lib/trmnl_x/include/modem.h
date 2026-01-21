#pragma once

#include <Arduino.h>

// UART pins (ESP32-S3 to ESP32-C5)
#define AT_UART_TX 43
#define AT_UART_RX 44
#define AT_UART_RTS 1
#define AT_UART_CTS 2

class Modem {
private:
  HardwareSerial ModemSerial;
  uint32_t baudRate;
    
public:
  Modem(uint32_t baudRate);
  ~Modem();
  bool sendCommand(const char* command);
  String readResponse(unsigned long timeout);
  bool flashFromFile(const char* filename);
};
