#pragma once

#include <Arduino.h>
#include <display.h>
#include <misc/clock.h>
#include <persistence_interface.h>

struct FirmwareUpdateResult {
  bool updated = false;
  MSG failureMessage = NONE;
};

class FirmwareUpdateService {
public:
  FirmwareUpdateService(Persistence &persistence, Clock &clock, int32_t wifiConnectionRssiThreshold);

  bool isUpdateDue(bool update_firmware, const String &firmware_url);
  FirmwareUpdateResult performUpdate();

private:
  bool performFirmwareUpdate();

  Persistence &_persistence;
  Clock &_clock;
  int32_t _wifiConnectionRssiThreshold;
  MSG _failureMessage = NONE;
  char _firmwareUrl[1024];
};
