#include <battery.h>

#ifdef INCLUDE_BQ27427

#include <ArduinoLog.h>

#include "BQ27427.h"

float BQ27427Battery::readVoltage() {
  if (lipo._initialized) {
    float voltage = lipo.voltage() / 1000.0; // Convert mV to V
    Log.info("%s [%d]: Battery voltage reading from BQ27427: %.3f V\r\n", __FILE__, __LINE__, voltage);
    return voltage;
  } else {
    Log.error("%s [%d]: BQ27427 not initialized. Cannot read battery voltage.\r\n", __FILE__, __LINE__);
    return -1.0;
  }
}

#endif // INCLUDE_BQ27427
