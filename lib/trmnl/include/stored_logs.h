#pragma once

#include <stddef.h>
#include <Arduino.h>
#include <persistence_interface.h>

struct LogStoreResult {
  enum Status {
    SUCCESS,
    FAILURE
  } status;
  const char* message;
  uint8_t slot_used;
};

class StoredLogs {
private:
    uint8_t old_count;
    uint8_t new_count;
    const char* log_key;
    const char* head_key;
    Persistence& persistence;
    uint32_t overwrite_count;

public:
    StoredLogs(uint8_t old_count, uint8_t new_count, const char* log_key, const char* head_key, Persistence& persistence);
    
    LogStoreResult store_log(const String& log_buffer);
    String gather_stored_logs();
    void clear_stored_logs();
    uint32_t get_overwrite_count();
};
