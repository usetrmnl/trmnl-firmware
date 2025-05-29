#pragma once

#include <stddef.h>
#include <Arduino.h>
#include <persistence_interface.h>

class StoredLogs {
private:
    uint8_t max_notes;
    const char* log_key;
    const char* head_key;
    Persistence& persistence;

public:
    StoredLogs(uint8_t max_notes, const char* log_key, const char* head_key, Persistence& persistence);
    
    void store_log(const String& log_buffer);
    String gather_stored_logs();
    void clear_stored_logs();
};
