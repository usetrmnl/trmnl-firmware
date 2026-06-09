#pragma once

#include <stdint.h>
#include <persistence_interface.h>

constexpr uint32_t OTA_RETRY_INTERVAL_SECONDS = 24UL * 60 * 60;
#define OTA_LAST_ATTEMPT_KEY "last_ota"

bool otaAttemptDue(uint32_t now, uint32_t lastOta);
uint32_t otaLastAttempt(Persistence &persistence);
void otaRecordAttempt(Persistence &persistence, uint32_t now);
