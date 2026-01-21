
#include <Arduino.h>
#include "esp_loader.h"
#include "esp_loader_io.h"
#include "esp32_port.h"
#include "SPIFFS.h"
#include "modem.h"

Modem::Modem(uint32_t baudRate) : ModemSerial(0) {
  ModemSerial.begin(baudRate, SERIAL_8N1, AT_UART_RX, AT_UART_TX);
  this->baudRate = baudRate;
  Serial.printf("[MODEM] ModemSerial initialized at %u baud\n", baudRate);
  delay(100);
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

bool Modem::flashFromFile(const char* filename) {
  Serial.printf("[MODEM] Starting modem flash from: %s\n", filename);

  // Step 1: Open and validate firmware file
  File firmwareFile = SPIFFS.open(filename, "r");
  if (!firmwareFile) {
    Serial.println("[MODEM] ERROR: File not found!");
    return false;
  }

  size_t firmwareSize = firmwareFile.size();
  Serial.printf("[MODEM] Firmware size: %u bytes (%.2f KB)\n", firmwareSize, firmwareSize / 1024.0);

  if (firmwareSize == 0) {
    Serial.println("[MODEM] ERROR: File is empty!");
    firmwareFile.close();
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
    // Restart ModemSerial
    ModemSerial.begin(baudRate, SERIAL_8N1, AT_UART_RX, AT_UART_TX);
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
