#ifdef BOARD_TRMNL_X

#include <Arduino.h>
#include <functional>
#include <esp_task_wdt.h>
#ifndef SERIAL_FLASHER_INTERFACE_UART
#define SERIAL_FLASHER_INTERFACE_UART
#endif
#include "esp_loader.h"
#include "esp_loader_io.h"
#include "esp32_port.h"
#include "modem.h"

#if defined (BOARD_TRMNL_X) || defined (BOARD_TRLML_X_EPDIY)
#include <LittleFS.h>
#define FS LittleFS
#else
#include <SPIFFS.h>
#define FS SPIFFS
#endif

Modem::Modem(uint32_t baudRate) : ModemSerial(0) {
  ModemSerial.begin(baudRate, SERIAL_8N1, AT_UART_RX, AT_UART_TX);
  ModemSerial.setPins(AT_UART_RX, AT_UART_TX, AT_UART_RTS, AT_UART_CTS);
  this->baudRate = baudRate;
  Serial.printf("[MODEM] Modem initialization...\n", baudRate);
  delay(100);

  // Verify modem is alive at the initial baud rate
  while (ModemSerial.available()) ModemSerial.read();
  sendCommand("AT");
  if (waitForResponse("OK", 5000).isEmpty()) {
    Serial.println("[MODEM] no response to AT — skipping baud upgrade");
    return;
  }

  sendCommand("AT+CWMODE=1");
  if (waitForResponse("OK", 5000).isEmpty()) {
    Serial.println("[MODEM] AT+CWMODE failed.");
    return;
  } else {
    Serial.println("[MODEM] set to station mode");
  }

  // Switch modem to 5 Mbps with RTS/CTS (volatile; reverts on power cycle)
  sendCommand("AT+UART_CUR=5000000,8,1,0,3");
  if (waitForResponse("OK", 5000).isEmpty()) {
    Serial.println("[MODEM] AT+UART_CUR failed — keeping initial baud rate");
    return;
  }

  // Reinitialize UART at 5 Mbps with hardware flow control via Arduino API
  ModemSerial.end();
  ModemSerial.setRxBufferSize(8192);
  ModemSerial.begin(5000000, SERIAL_8N1, AT_UART_RX, AT_UART_TX);
  ModemSerial.setHwFlowCtrlMode(UART_HW_FLOWCTRL_CTS_RTS, 120);
  ModemSerial.setPins(-1, -1, AT_UART_RTS, AT_UART_CTS);
  this->baudRate = 5000000;
  delay(100);  // let UART driver and modem settle after baud switch
  while (ModemSerial.available()) ModemSerial.read();  // flush transition garbage

  // Confirm communication at new baud rate
  sendCommand("AT");
  if (waitForResponse("OK", 5000).isEmpty()) {
    Serial.println("[MODEM] no response at 5 Mbps — falling back to 2.4 GHz mode");
  } else {
    Serial.println("[MODEM] running at 5 Mbps with RTS/CTS");
    _initialized = true;
  }
}


Modem::~Modem() {
  ModemSerial.end();
  Serial.println("[MODEM] ModemSerial ended");
}

bool Modem::sendCommand(const char* command) {
  ModemSerial.println(command);
  return true;
}

String Modem::readResponse(unsigned long timeout) {
  unsigned long startTime = millis();
  
  String response = "";

  while (ModemSerial.available() && millis() - startTime < timeout) {
    char c = ModemSerial.read();
    response += c;
    // Reset timeout on each received character
    startTime = millis();
  }
  return response;
}

bool Modem::flashFromFile(const char* filename, String& errorOut) {
  Serial.printf("[MODEM] Starting modem flash from: %s\n", filename);

  // Step 1: Open and validate firmware file
  File firmwareFile = FS.open(filename, "r");
  if (!firmwareFile) {
    Serial.println("[MODEM] ERROR: File not found!");
    errorOut = "Firmware file not found";
    return false;
  }

  size_t firmwareSize = firmwareFile.size();
  Serial.printf("[MODEM] Firmware size: %u bytes (%.2f KB)\n", firmwareSize, firmwareSize / 1024.0);

  if (firmwareSize == 0) {
    Serial.println("[MODEM] ERROR: File is empty!");
    firmwareFile.close();
    errorOut = "Firmware file is empty";
    return false;
  }

  // Step 2: Prepare for flashing
  Serial.println("[MODEM] Preparing for flash operation...");
  while (ModemSerial.available()) ModemSerial.read();
  delay(100);

  // Step 3: Initialize serial flasher
  Serial.println("[MODEM] Initializing serial flasher...");
  ModemSerial.end();
  delay(100);

  const loader_esp32_config_t flasherConfig = {
    .baud_rate = 1000000,
    .uart_port = 0,
    .uart_rx_pin = AT_UART_RX,
    .uart_tx_pin = AT_UART_TX,
    .reset_trigger_pin = 0,
    .gpio0_trigger_pin = 0,
    .rx_buffer_size = 2048,
    .tx_buffer_size = 2048,
    .queue_size = 0,
    .uart_queue = NULL,
    .dont_initialize_peripheral = false
  };

  esp_loader_error_t err = loader_port_esp32_init(&flasherConfig);
  if (err != ESP_LOADER_SUCCESS) {
    Serial.printf("[MODEM] ERROR: Serial flasher init failed (code %d)\n", err);
    firmwareFile.close();
    ModemSerial.begin(baudRate, SERIAL_8N1, AT_UART_RX, AT_UART_TX);
    errorOut = "Flasher init failed (code " + String((int)err) + ")";
    return false;
  }

  // Step 4: Connect to target (this will automatically enter bootloader mode)
  Serial.println("[MODEM] Connecting to bootloader...");

  // Configure connection parameters with more retries and longer timeout
  esp_loader_connect_args_t connectConfig = {
    .sync_timeout = 500,  // 500ms timeout per sync attempt
    .trials = 20,         // 20 retry attempts
  };

  err = esp_loader_connect(&connectConfig);

  if (err != ESP_LOADER_SUCCESS) {
    Serial.printf("[MODEM] ERROR: Bootloader connection failed (code %d)\n", err);
    firmwareFile.close();
    loader_port_reset_target();
    loader_port_esp32_deinit();
    ModemSerial.begin(baudRate, SERIAL_8N1, AT_UART_RX, AT_UART_TX);
    errorOut = "Bootloader connection failed";
    return false;
  }

  Serial.println("[MODEM] Connected to bootloader!");

  // Step 5: Start flash operation
  const uint32_t FLASH_BLOCK_SIZE = 4096;  // 4KB blocks
  const uint32_t FLASH_START_ADDR = 0x0;
  uint32_t totalBlocks = (firmwareSize + FLASH_BLOCK_SIZE - 1) / FLASH_BLOCK_SIZE;

  Serial.printf("[MODEM] Starting flash operation:\n");
  Serial.printf("  - Address: 0x%X\n", FLASH_START_ADDR);
  Serial.printf("  - Size: %u bytes\n", firmwareSize);
  Serial.printf("  - Blocks: %u x %u bytes\n", totalBlocks, FLASH_BLOCK_SIZE);

  err = esp_loader_flash_start(FLASH_START_ADDR, firmwareSize, FLASH_BLOCK_SIZE);
  if (err != ESP_LOADER_SUCCESS) {
    Serial.printf("[MODEM] ERROR: Flash start failed (code %d)\n", err);
    firmwareFile.close();
    loader_port_reset_target();
    loader_port_esp32_deinit();
    ModemSerial.begin(baudRate, SERIAL_8N1, AT_UART_RX, AT_UART_TX);
    errorOut = "Flash start failed (code " + String((int)err) + ")";
    return false;
  }

  // Step 6: Write firmware data block by block
  Serial.println("[MODEM] Erasing and writing flash...");

  uint8_t* blockBuffer = (uint8_t*)malloc(FLASH_BLOCK_SIZE);
  if (!blockBuffer) {
    Serial.println("[MODEM] ERROR: Failed to allocate block buffer!");
    firmwareFile.close();
    loader_port_reset_target();
    loader_port_esp32_deinit();
    ModemSerial.begin(baudRate, SERIAL_8N1, AT_UART_RX, AT_UART_TX);
    errorOut = "Out of memory for block buffer";
    return false;
  }

  unsigned long flashStartTime = millis();
  uint32_t lastProgressBlock = 0;
  bool flashSuccess = true;

  for (uint32_t blockNum = 0; blockNum < totalBlocks; blockNum++) {
    // Read block from file
    size_t bytesRead = firmwareFile.read(blockBuffer, FLASH_BLOCK_SIZE);

    // Pad incomplete blocks with 0xFF
    if (bytesRead < FLASH_BLOCK_SIZE) {
      memset(blockBuffer + bytesRead, 0xFF, FLASH_BLOCK_SIZE - bytesRead);
    }

    // Write block to flash
    err = esp_loader_flash_write(blockBuffer, FLASH_BLOCK_SIZE);
    if (err != ESP_LOADER_SUCCESS) {
      Serial.printf("[MODEM] ERROR: Write failed at block %u (code %d)\n", blockNum, err);
      errorOut = "Write error at block " + String((unsigned)blockNum) + " (code " + String((int)err) + ")";
      flashSuccess = false;
      break;
    }

    // Progress update every 16 blocks (64KB)
    if (blockNum - lastProgressBlock >= 16 || blockNum == totalBlocks - 1) {
      float progressPercent = (float)(blockNum + 1) * 100.0 / totalBlocks;
      uint32_t bytesWritten = (blockNum + 1) * FLASH_BLOCK_SIZE;
      if (bytesWritten > firmwareSize) bytesWritten = firmwareSize;

      Serial.printf("[FLASH] Progress: %u/%u blocks (%.1f%%) - %u/%u bytes\n",
                    blockNum + 1, totalBlocks, progressPercent, bytesWritten, firmwareSize);
      lastProgressBlock = blockNum;
    }
  }

  free(blockBuffer);
  firmwareFile.close();

  if (!flashSuccess) {
    Serial.println("[MODEM] Flash operation FAILED!");
    loader_port_reset_target();
    loader_port_esp32_deinit();
    ModemSerial.begin(baudRate, SERIAL_8N1, AT_UART_RX, AT_UART_TX);
    return false;
  }

  // Step 7: Finish flash operation
  Serial.println("[MODEM] Finishing flash operation...");
  err = esp_loader_flash_finish(true);  // true = reboot after flash
  if (err != ESP_LOADER_SUCCESS) {
    Serial.printf("[MODEM] WARNING: Flash finish returned code %d\n", err);
    // Continue anyway - often not critical
  }

  unsigned long flashDuration = millis() - flashStartTime;
  float flashSpeedKBps = (firmwareSize / 1024.0) / (flashDuration / 1000.0);

  // Step 8: Reset target and cleanup
  Serial.println("[MODEM] Resetting target...");
  loader_port_reset_target();
  loader_port_esp32_deinit();

  // Reinitialize ModemSerial for normal AT command operation
  Serial.println("[MODEM] Reinitializing Modem UART...");
  ModemSerial.begin(baudRate, SERIAL_8N1, AT_UART_RX, AT_UART_TX);
  delay(100);

  // Success summary
  Serial.println("[MODEM] FLASH COMPLETE!");
  Serial.printf("  Firmware: %s\n", filename);
  Serial.printf("  Size: %u bytes (%.2f KB)\n", firmwareSize, firmwareSize / 1024.0);
  Serial.printf("  Time: %lu ms (%.1f seconds)\n", flashDuration, flashDuration / 1000.0);
  Serial.printf("  Speed: %.2f KB/s\n", flashSpeedKBps);

  return true;
}

// ---------------------------------------------------------------------------
// eraseFlash(): full chip erase of the ESP32-C5 via serial bootloader
// ---------------------------------------------------------------------------
bool Modem::eraseFlash() {
  while (ModemSerial.available()) ModemSerial.read();
  ModemSerial.end();
  delay(100);

  const loader_esp32_config_t flasherConfig = {
    .baud_rate = 1000000,
    .uart_port = 0,
    .uart_rx_pin = AT_UART_RX,
    .uart_tx_pin = AT_UART_TX,
    .reset_trigger_pin = 0,
    .gpio0_trigger_pin = 0,
    .rx_buffer_size = 2048,
    .tx_buffer_size = 2048,
    .queue_size = 0,
    .uart_queue = NULL,
    .dont_initialize_peripheral = false
  };

  esp_loader_error_t err = loader_port_esp32_init(&flasherConfig);
  if (err != ESP_LOADER_SUCCESS) {
    Serial.printf("[MODEM] eraseFlash: init failed (code %d)\n", err);
    ModemSerial.begin(baudRate, SERIAL_8N1, AT_UART_RX, AT_UART_TX);
    return false;
  }

  esp_loader_connect_args_t connectConfig = { .sync_timeout = 500, .trials = 20 };
  err = esp_loader_connect(&connectConfig);
  if (err != ESP_LOADER_SUCCESS) {
    Serial.printf("[MODEM] eraseFlash: bootloader connect failed (code %d)\n", err);
    loader_port_reset_target();
    loader_port_esp32_deinit();
    ModemSerial.begin(baudRate, SERIAL_8N1, AT_UART_RX, AT_UART_TX);
    return false;
  }

  Serial.println("[MODEM] eraseFlash: connected, erasing (~10 s)...");
  err = esp_loader_flash_erase();

  loader_port_reset_target();
  loader_port_esp32_deinit();
  ModemSerial.begin(baudRate, SERIAL_8N1, AT_UART_RX, AT_UART_TX);

  if (err != ESP_LOADER_SUCCESS) {
    Serial.printf("[MODEM] eraseFlash: erase failed (code %d)\n", err);
    return false;
  }

  Serial.println("[MODEM] eraseFlash: done.");
  return true;
}

// ---------------------------------------------------------------------------
// setSerialBaud(): reinitialize ModemSerial at a different baud rate
// ---------------------------------------------------------------------------
void Modem::setSerialBaud(uint32_t baud) {
  ModemSerial.end();
  delay(50);
  ModemSerial.begin(baud, SERIAL_8N1, AT_UART_RX, AT_UART_TX);
  baudRate = baud;
  Serial.printf("[MODEM] Serial baud set to %u\n", baud);
}

// ---------------------------------------------------------------------------
// Private helper: accumulate RX bytes until expected marker or timeout
// ---------------------------------------------------------------------------
String Modem::waitForResponse(const String& expected, unsigned long timeoutMs) {
  unsigned long deadline = millis() + timeoutMs;
  String acc;
  while (millis() < deadline) {
    while (ModemSerial.available()) {
      acc += (char)ModemSerial.read();
    }
    if (acc.indexOf(expected) >= 0) return acc;
    delay(5);
  }
  return "";  // timeout — marker not found
}

// ---------------------------------------------------------------------------
// scanNetworks(): AT+CWLAP scan, returns deduplicated list
// ---------------------------------------------------------------------------
std::vector<Modem::ModemNetwork> Modem::scanNetworks() {
  while (ModemSerial.available()) ModemSerial.read();  // flush

  sendCommand("AT+CWLAP");

  // Dual-band scan can take up to ~10 s; allow 15 s
  unsigned long deadline = millis() + 15000;
  String acc;
  while (millis() < deadline) {
    while (ModemSerial.available()) acc += (char)ModemSerial.read();
    if (acc.indexOf("\nOK")  >= 0) break;
    if (acc.indexOf("ERROR") >= 0) break;
    delay(10);
  }

  std::vector<ModemNetwork> results;

  int pos = 0;
  while ((pos = acc.indexOf("+CWLAP:", pos)) >= 0) {
    int paren = acc.indexOf('(', pos);
    int close = acc.indexOf(')', paren);
    if (paren < 0 || close < 0) { pos++; continue; }

    String entry = acc.substring(paren + 1, close);

    // field 0: ecn
    int c1 = entry.indexOf(',');
    int ecn = entry.substring(0, c1).toInt();

    // field 1: quoted SSID
    int q1 = entry.indexOf('"', c1);
    int q2 = entry.indexOf('"', q1 + 1);
    String ssid = entry.substring(q1 + 1, q2);

    // field 2: rssi
    int c2 = entry.indexOf(',', q2 + 1);
    int c3 = entry.indexOf(',', c2 + 1);
    int32_t rssi = entry.substring(c2 + 1, c3).toInt();

    // skip quoted MAC, find channel
    int q3 = entry.indexOf('"', c3);
    int q4 = entry.indexOf('"', q3 + 1);
    int c4 = entry.indexOf(',', q4 + 1);
    int c5 = entry.indexOf(',', c4 + 1);
    int channel = entry.substring(c4 + 1, c5 >= 0 ? c5 : entry.length()).toInt();

    if (ssid.isEmpty() || ssid == "TRMNL") { pos = close; continue; }

    // Dedup: keep highest RSSI per SSID
    bool found = false;
    bool is5GHz = channel >= 36;
    for (auto& net : results) {
      if (net.ssid == ssid && net.is5GHz == is5GHz) { // only compare RSSI of the same band
        if (rssi > net.rssi) net.rssi = rssi;
        found = true;
        break;
      }
    }
    if (!found) {
      results.push_back({ssid, rssi, ecn == 0, is5GHz});
    }
    pos = close;
  }

  return results;
}

// ---------------------------------------------------------------------------
// connectToNetwork() / disconnectFromNetwork()
// ---------------------------------------------------------------------------
bool Modem::connectToNetwork(const String& ssid, const String& password) {
  // Escape backslash and double-quote in ssid/password
  String s = ssid;      s.replace("\\", "\\\\"); s.replace("\"", "\\\"");
  String p = password;  p.replace("\\", "\\\\"); p.replace("\"", "\\\"");

  while (ModemSerial.available()) ModemSerial.read();  // flush

  sendCommand("AT+CWMODE=1");
  if (waitForResponse("OK", 3000).isEmpty()) {
    Serial.println("[MODEM] connectToNetwork: AT+CWMODE=1 failed");
    return false;
  }

  String cmd = "AT+CWJAP=\"" + s + "\",\"" + p + "\"";
  sendCommand(cmd.c_str());

  // Success contains "WIFI GOT IP"; failure contains "FAIL" or "ERROR"
  String resp = waitForResponse("WIFI GOT IP", 20000);
  if (resp.isEmpty()) {
    Serial.println("[MODEM] connectToNetwork: failed to obtain IP");
    return false;
  }
  Serial.println("[MODEM] connectToNetwork: WIFI GOT IP");
  return true;
}

bool Modem::disconnectFromNetwork() {
  sendCommand("AT+CWQAP");
  return !waitForResponse("OK", 5000).isEmpty();
}

// ---------------------------------------------------------------------------
// httpGet(): AT+HTTPCLIENT GET — byte-by-byte state machine
// ---------------------------------------------------------------------------
Modem::ModemHttpResult Modem::httpGet(const String& url, const String& saveToFile, size_t contentLength, const String& reqHeaders) {
  while (ModemSerial.available()) ModemSerial.read();  // flush

  // Set custom request headers via AT+HTTPCHEAD (global; persists until cleared).
  // reqHeaders format: "Key: Value\nKey2: Value2" — newline-separated, no trailing CRLF.
  if (!reqHeaders.isEmpty()) {
    // Clear any previously set headers
    sendCommand("AT+HTTPCHEAD=0");
    if (waitForResponse("OK", 3000).isEmpty()) {
      Serial.println("[HTTP] HTTPCHEAD=0 failed");
      return {false, 0, "", 0};
    }

    // Set each header individually
    int pos = 0;
    while (pos <= (int)reqHeaders.length()) {
      int nl = reqHeaders.indexOf('\n', pos);
      String hdr = (nl < 0) ? reqHeaders.substring(pos) : reqHeaders.substring(pos, nl);
      hdr.trim();
      if (!hdr.isEmpty()) {
        Serial.printf("[HTTP] HTTPCHEAD(%u): %s\n", hdr.length(), hdr.c_str());
        sendCommand(("AT+HTTPCHEAD=" + String(hdr.length())).c_str());
        if (waitForResponse(">", 3000).isEmpty()) {
          Serial.println("[HTTP] HTTPCHEAD: no '>' prompt");
          return {false, 0, "", 0};
        }
        ModemSerial.write((const uint8_t*)hdr.c_str(), hdr.length());
        ModemSerial.flush();
        if (waitForResponse("OK", 3000).isEmpty()) {
          Serial.printf("[HTTP] HTTPCHEAD: no 'OK' for: %s\n", hdr.c_str());
          return {false, 0, "", 0};
        }
      }
      if (nl < 0) break;
      pos = nl + 1;
    }
  }

  // Use AT+HTTPURLCFG only when the URL itself is too long to fit inline.
  bool useUrlCfg = (url.length() + 24 > 256);

  if (useUrlCfg) {
    Serial.printf("[HTTP] URL %u bytes — using AT+HTTPURLCFG\n", url.length());
    Serial.flush();

    sendCommand(("AT+HTTPURLCFG=" + String(url.length())).c_str());
    String promptResp = waitForResponse(">", 5000);
    if (promptResp.isEmpty()) {
      Serial.println("[HTTP] URLCFG: no '>' prompt (timeout)");
      return {false, 0, "", 0};
    }
    Serial.printf("[HTTP] URLCFG prompt OK (%u bytes from modem)\n", promptResp.length());
    Serial.flush();

    ModemSerial.write((const uint8_t*)url.c_str(), url.length());
    ModemSerial.flush();

    String setOkResp = waitForResponse("SET OK", 5000);
    if (setOkResp.isEmpty()) {
      Serial.println("[HTTP] URLCFG: no 'SET OK' (timeout)");
      return {false, 0, "", 0};
    }
    Serial.printf("[HTTP] URLCFG SET OK (%u bytes from modem)\n", setOkResp.length());
    Serial.flush();

    while (ModemSerial.available()) ModemSerial.read();
    Serial.println("[HTTP] cmd: AT+HTTPCLIENT=2,0,\"\",,,2");
    Serial.flush();
    sendCommand("AT+HTTPCLIENT=2,0,\"\",,,2");
  } else {
    String httpCmd = "AT+HTTPCLIENT=2,0,\"" + url + "\",,,2";
    Serial.printf("[HTTP] cmd(%u): %s\n", httpCmd.length(), httpCmd.substring(0, 180).c_str());
    Serial.flush();
    sendCommand(httpCmd.c_str());
  }

  // --- open output file if requested ---
  bool toFile = saveToFile.length() > 0;
  File outFile;
  if (toFile) {
    outFile = FS.open(saveToFile, "w");
    if (!outFile) return {false, 0, "", 0};
  }

  // --- state machine ---
  enum class ParseState { SCAN, SIZE, DATA };
  ParseState state = ParseState::SCAN;

  // SCAN: byte-by-byte match against "+HTTPCLIENT:"
  const char MARKER[]  = "+HTTPCLIENT:";
  const int  MARKER_LEN = 12;
  int matchPos = 0;

  // SCAN: rolling tail to detect "\nOK\r" / "\nOK\n" and "ERROR"
  char tail[8] = {};
  int  tailLen  = 0;

  // SIZE: digit accumulator
  char     sizeBuf[12];
  int      sizeIdx = 0;
  uint32_t chunkRemaining = 0;

  // DATA: 512-byte write buffer
  uint8_t writeBuf[512];
  int     writeBufIdx = 0;

  String body;
  size_t totalBytes = 0;
  bool   done  = false;
  bool   error = false;

  unsigned long downloadStart  = 0;   // set on first DATA byte
  unsigned long lastProgressMs = 0;

  // Debug: capture first 256 raw bytes from modem
  char   rawDbg[257] = {};
  int    rawDbgLen   = 0;

  unsigned long deadline = millis() + 60000;

  while (!done && !error && millis() < deadline) {
    esp_task_wdt_reset();  // keep loopTask WDT subscription alive during long downloads

    // Progress reporting — runs once per batch iteration (~once per few hundred bytes)
    if (downloadStart > 0) {
      unsigned long now = millis();
      if (now - lastProgressMs >= 1000) {
        float elapsed = (now - downloadStart) / 1000.0f;
        float kbps    = elapsed > 0.0f ? (totalBytes / 1024.0f) / elapsed : 0.0f;
        if (contentLength > 0) {
          int pct = (int)(totalBytes * 100 / contentLength);
          Serial.printf("[HTTP] %u/%u bytes (%d%%) @ %.1f KB/s\n",
                        totalBytes, contentLength, pct, kbps);
        } else {
          Serial.printf("[HTTP] %u bytes @ %.1f KB/s\n", totalBytes, kbps);
        }
        lastProgressMs = now;
      }
    }

    int avail = ModemSerial.available();
    if (avail <= 0) { delay(1); continue; }

    uint8_t batch[256];
    int got = ModemSerial.readBytes(batch, min(avail, (int)sizeof(batch)));

    for (int i = 0; i < got && !done && !error; i++) {
      uint8_t b = batch[i];
      if (rawDbgLen < 256) rawDbg[rawDbgLen++] = (char)b;

      switch (state) {
      case ParseState::SCAN:
        // Feed rolling tail for terminator detection
        if (tailLen < (int)sizeof(tail)) {
          tail[tailLen++] = (char)b;
        } else {
          memmove(tail, tail + 1, sizeof(tail) - 1);
          tail[sizeof(tail) - 1] = (char)b;
        }

        if (tailLen >= 4 &&
            tail[tailLen-4] == '\n' && tail[tailLen-3] == 'O' &&
            tail[tailLen-2] == 'K' &&
            (tail[tailLen-1] == '\r' || tail[tailLen-1] == '\n')) {
          done = true; break;
        }
        if (tailLen >= 5 && memcmp(&tail[tailLen-5], "ERROR", 5) == 0) {
          error = true; break;
        }

        // Match +HTTPCLIENT: byte-by-byte
        if (b == (uint8_t)MARKER[matchPos]) {
          if (++matchPos == MARKER_LEN) {
            state    = ParseState::SIZE;
            sizeIdx  = 0;
            matchPos = 0;
            tailLen  = 0;
          }
        } else {
          matchPos = (b == (uint8_t)MARKER[0]) ? 1 : 0;
        }
        break;

      case ParseState::SIZE:
        if (b == ',') {
          sizeBuf[sizeIdx] = '\0';
          chunkRemaining   = (uint32_t)atoi(sizeBuf);
          state            = ParseState::DATA;
        } else if (b >= '0' && b <= '9') {
          if (sizeIdx < (int)sizeof(sizeBuf) - 1) sizeBuf[sizeIdx++] = (char)b;
        }
        break;

      case ParseState::DATA:
        if (toFile) {
          writeBuf[writeBufIdx++] = b;
          if (writeBufIdx >= (int)sizeof(writeBuf)) {
            if (outFile.write(writeBuf, writeBufIdx) != (size_t)writeBufIdx) {
              error = true; break;
            }
            writeBufIdx = 0;
          }
          // Capture first 512 bytes for error diagnostics (e.g. HTTP error body)
          if (body.length() < 512) body += (char)b;
        } else {
          body += (char)b;
        }
        totalBytes++;

        if (downloadStart == 0) downloadStart = lastProgressMs = millis();

        if (--chunkRemaining == 0) {
          if (toFile && writeBufIdx > 0) {
            if (outFile.write(writeBuf, writeBufIdx) != (size_t)writeBufIdx) {
              error = true; break;
            }
            writeBufIdx = 0;
          }
          state    = ParseState::SCAN;
          tailLen  = 0;
          matchPos = 0;
        }
        break;
      }
    }
  }

  if (toFile) {
    outFile.close();
    if (!done || error) FS.remove(saveToFile.c_str());
  }

  // Debug: print raw modem response and exit reason
  Serial.printf("[HTTP] raw(%u, %s): ", rawDbgLen,
    done ? "DONE" : (error ? "ERR" : "TOUT"));
  for (int _i = 0; _i < rawDbgLen; _i++) {
    char _c = rawDbg[_i];
    if (_c >= 32 && _c < 127) Serial.print(_c);
    else Serial.printf("<%02X>", (uint8_t)_c);
  }
  Serial.println();
  Serial.flush();

  if (downloadStart > 0) {
    float elapsed = (millis() - downloadStart) / 1000.0f;
    float kbps    = elapsed > 0.0f ? (totalBytes / 1024.0f) / elapsed : 0.0f;
    Serial.printf("[HTTP] Done: %u bytes in %.1f s (%.1f KB/s)\n", totalBytes, elapsed, kbps);
  }

  // Clear request headers (global on modem) so they don't bleed into later requests
  if (!reqHeaders.isEmpty()) {
    sendCommand("AT+HTTPCHEAD=0");
    waitForResponse("OK", 2000);
  }

  // statusCode: 200=success, 0=modem returned ERROR (HTTP 4xx/5xx), -1=timeout
  int statusCode = (done && totalBytes > 0) ? 200 : (error ? 0 : -1);
  return {done && totalBytes > 0, statusCode, body, totalBytes};
}

// ---------------------------------------------------------------------------
// httpGet() — chunk-callback overload: calls chunkCb(data, len) per chunk
// ---------------------------------------------------------------------------
Modem::ModemHttpResult Modem::httpGet(const String& url, std::function<bool(const uint8_t*, size_t)> chunkCb, size_t contentLength, const String& reqHeaders) {
  while (ModemSerial.available()) ModemSerial.read();  // flush

  if (!reqHeaders.isEmpty()) {
    sendCommand("AT+HTTPCHEAD=0");
    if (waitForResponse("OK", 3000).isEmpty()) {
      Serial.println("[HTTP] HTTPCHEAD=0 failed");
      return {false, 0, "", 0};
    }
    int pos = 0;
    while (pos <= (int)reqHeaders.length()) {
      int nl = reqHeaders.indexOf('\n', pos);
      String hdr = (nl < 0) ? reqHeaders.substring(pos) : reqHeaders.substring(pos, nl);
      hdr.trim();
      if (!hdr.isEmpty()) {
        sendCommand(("AT+HTTPCHEAD=" + String(hdr.length())).c_str());
        if (waitForResponse(">", 3000).isEmpty()) return {false, 0, "", 0};
        ModemSerial.write((const uint8_t*)hdr.c_str(), hdr.length());
        ModemSerial.flush();
        if (waitForResponse("OK", 3000).isEmpty()) return {false, 0, "", 0};
      }
      if (nl < 0) break;
      pos = nl + 1;
    }
  }

  bool useUrlCfg = (url.length() + 24 > 256);
  if (useUrlCfg) {
    sendCommand(("AT+HTTPURLCFG=" + String(url.length())).c_str());
    if (waitForResponse(">", 5000).isEmpty()) return {false, 0, "", 0};
    ModemSerial.write((const uint8_t*)url.c_str(), url.length());
    ModemSerial.flush();
    if (waitForResponse("SET OK", 5000).isEmpty()) return {false, 0, "", 0};
    while (ModemSerial.available()) ModemSerial.read();
    sendCommand("AT+HTTPCLIENT=2,0,\"\",,,2");
  } else {
    sendCommand(("AT+HTTPCLIENT=2,0,\"" + url + "\",,,2").c_str());
  }

  enum class ParseState { SCAN, SIZE, DATA };
  ParseState state = ParseState::SCAN;

  const char MARKER[]   = "+HTTPCLIENT:";
  const int  MARKER_LEN = 12;
  int matchPos = 0;

  char     tail[8] = {};
  int      tailLen  = 0;

  char     sizeBuf[12];
  int      sizeIdx = 0;
  uint32_t chunkRemaining = 0;

  uint8_t writeBuf[512];
  int     writeBufIdx = 0;

  size_t totalBytes = 0;
  bool   done  = false;
  bool   error = false;
  bool   cbAbort = false;

  unsigned long downloadStart  = 0;
  unsigned long lastProgressMs = 0;
  unsigned long deadline = millis() + 60000;

  while (!done && !error && !cbAbort && millis() < deadline) {
    esp_task_wdt_reset();

    if (downloadStart > 0) {
      unsigned long now = millis();
      if (now - lastProgressMs >= 1000) {
        float elapsed = (now - downloadStart) / 1000.0f;
        float kbps    = elapsed > 0.0f ? (totalBytes / 1024.0f) / elapsed : 0.0f;
        if (contentLength > 0) {
          int pct = (int)(totalBytes * 100 / contentLength);
          Serial.printf("[HTTP] %u/%u bytes (%d%%) @ %.1f KB/s\n", totalBytes, contentLength, pct, kbps);
        } else {
          Serial.printf("[HTTP] %u bytes @ %.1f KB/s\n", totalBytes, kbps);
        }
        lastProgressMs = now;
      }
    }

    int avail = ModemSerial.available();
    if (avail <= 0) { delay(1); continue; }

    uint8_t batch[256];
    int got = ModemSerial.readBytes(batch, min(avail, (int)sizeof(batch)));

    for (int i = 0; i < got && !done && !error && !cbAbort; i++) {
      uint8_t b = batch[i];

      switch (state) {
      case ParseState::SCAN:
        if (tailLen < (int)sizeof(tail)) {
          tail[tailLen++] = (char)b;
        } else {
          memmove(tail, tail + 1, sizeof(tail) - 1);
          tail[sizeof(tail) - 1] = (char)b;
        }
        if (tailLen >= 4 &&
            tail[tailLen-4] == '\n' && tail[tailLen-3] == 'O' &&
            tail[tailLen-2] == 'K' &&
            (tail[tailLen-1] == '\r' || tail[tailLen-1] == '\n')) {
          done = true; break;
        }
        if (tailLen >= 5 && memcmp(&tail[tailLen-5], "ERROR", 5) == 0) {
          error = true; break;
        }
        if (b == (uint8_t)MARKER[matchPos]) {
          if (++matchPos == MARKER_LEN) {
            state    = ParseState::SIZE;
            sizeIdx  = 0;
            matchPos = 0;
            tailLen  = 0;
          }
        } else {
          matchPos = (b == (uint8_t)MARKER[0]) ? 1 : 0;
        }
        break;

      case ParseState::SIZE:
        if (b == ',') {
          sizeBuf[sizeIdx] = '\0';
          chunkRemaining   = (uint32_t)atoi(sizeBuf);
          state            = ParseState::DATA;
        } else if (b >= '0' && b <= '9') {
          if (sizeIdx < (int)sizeof(sizeBuf) - 1) sizeBuf[sizeIdx++] = (char)b;
        }
        break;

      case ParseState::DATA:
        writeBuf[writeBufIdx++] = b;
        totalBytes++;
        if (downloadStart == 0) downloadStart = lastProgressMs = millis();
        --chunkRemaining;

        if (writeBufIdx >= (int)sizeof(writeBuf) || chunkRemaining == 0) {
          if (!chunkCb(writeBuf, writeBufIdx)) {
            cbAbort = true;
          }
          writeBufIdx = 0;
        }

        if (chunkRemaining == 0 && !cbAbort) {
          state    = ParseState::SCAN;
          tailLen  = 0;
          matchPos = 0;
        }
        break;
      }
    }
  }

  // Flush any remaining buffered data to callback (only on clean completion)
  if (writeBufIdx > 0 && done && !cbAbort) {
    if (!chunkCb(writeBuf, writeBufIdx)) cbAbort = true;
    writeBufIdx = 0;
  }

  if (downloadStart > 0) {
    float elapsed = (millis() - downloadStart) / 1000.0f;
    float kbps    = elapsed > 0.0f ? (totalBytes / 1024.0f) / elapsed : 0.0f;
    Serial.printf("[HTTP] Done: %u bytes in %.1f s (%.1f KB/s)\n", totalBytes, elapsed, kbps);
  }

  if (!reqHeaders.isEmpty()) {
    sendCommand("AT+HTTPCHEAD=0");
    waitForResponse("OK", 2000);
  }

  bool ok = done && totalBytes > 0 && !cbAbort;
  int statusCode = ok ? 200 : (error ? 0 : -1);
  return {ok, statusCode, "", totalBytes};
}

// ---------------------------------------------------------------------------
// getSntpTime(): configure modem SNTP and return Unix UTC timestamp
// ---------------------------------------------------------------------------
time_t Modem::getSntpTime() {
  while (ModemSerial.available()) ModemSerial.read();  // flush

  // Enable SNTP with UTC timezone and two well-known servers
  sendCommand("AT+CIPSNTPCFG=1,0,\"time.google.com\",\"time.cloudflare.com\"");
  if (waitForResponse("OK", 3000).isEmpty()) {
    Serial.println("[MODEM] CIPSNTPCFG failed");
    return 0;
  }

  // Wait for modem to confirm NTP sync via +TIME_UPDATED URC (up to 30 s)
  String upd = waitForResponse("+TIME_UPDATED", 30000);
  if (upd.isEmpty()) {
    Serial.println("[MODEM] SNTP +TIME_UPDATED timeout");
    return 0;
  }

  while (ModemSerial.available()) ModemSerial.read();  // flush after URC

  sendCommand("AT+CIPSNTPTIME?");
  String resp = waitForResponse("OK", 3000);

  // Response: +CIPSNTPTIME:Tue Oct 19 17:47:56 2021\r\nOK
  int idx = resp.indexOf("+CIPSNTPTIME:");
  if (idx < 0) { Serial.println("[MODEM] CIPSNTPTIME parse fail"); return 0; }

  String ascStr = resp.substring(idx + 13);
  ascStr.trim();
  // Take only the first line
  int nl = ascStr.indexOf('\n');
  if (nl > 0) ascStr = ascStr.substring(0, nl);
  ascStr.trim();

  struct tm t = {};
  if (!strptime(ascStr.c_str(), "%a %b %d %H:%M:%S %Y", &t)) {
    Serial.println("[MODEM] strptime failed");
    return 0;
  }
  t.tm_isdst = -1;
  time_t ts = mktime(&t);
  Serial.printf("[MODEM] SNTP UTC time: %lu\n", (unsigned long)ts);
  return ts;
}
#endif // BOARD_TRMNL_X
