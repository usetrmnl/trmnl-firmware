#pragma once

#include <persistence_interface.h>
#include <stdint.h>

constexpr uint32_t OTA_RETRY_INTERVAL_SECONDS = 24UL * 60 * 60;
#define OTA_LAST_ATTEMPT_KEY "last_ota"

// True when interval seconds have passed since lastAttempt (0 = never); false while the clock is unsynced (now == 0)
bool retryDue(uint32_t now, uint32_t lastAttempt, uint32_t interval);
bool otaAttemptDue(uint32_t now, uint32_t lastOta);
uint32_t otaLastAttempt(Persistence &persistence);
void otaRecordAttempt(Persistence &persistence, uint32_t now);
