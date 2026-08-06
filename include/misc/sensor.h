#pragma once

#include <DEV_Config.h>
#include <config.h>

#ifdef SENSOR_SDA
#define INCLUDE_ENV_SENSOR
#endif

#ifdef INCLUDE_ENV_SENSOR
#include <bb_scd41.h>
#include <bb_temperature.h>
#endif

/// @brief Environmental sample captured during this wake cycle. Temperatures
///        are in tenths of a degree Celsius.
struct SensorReadings {
  int co2 = 0;           // ppm; 0 = no SCD41 sample
  int scdTemperature = 0;
  int scdHumidity = 0;   // percent
  int sensorType = -1;   // bb_temperature device type; -1 = no sample
  int temperature = 0;
  int humidity = 0;      // percent
  int pressure = 0;      // hectopascal
  int sampledAt = 0;     // UTC epoch when the sample was captured
};

/// @brief Sensor interface. The base class silently does nothing and reports
///        empty readings, for boards with no environmental sensor bus.
class BaseSensor {
public:
  virtual ~BaseSensor() = default;

  virtual void init() {}
  virtual SensorReadings readings() { return SensorReadings(); }

  /// @brief Builds the SENSORS HTTP header value from the last sample.
  ///        Returns false when no sample is available; on success the caller
  ///        owns *szTemp and must free() it.
  virtual bool buildSensorsHeader(char **szTemp) {
    (void)szTemp;
    return false;
  }
};

#ifdef INCLUDE_ENV_SENSOR
/// @brief Samples an SCD41 CO2 sensor and/or a bb_temperature-supported
///        sensor over I2C once per wake cycle.
class EnvironmentSensor : public BaseSensor {
public:
  EnvironmentSensor(int sdaPin, int sclPin) : _sdaPin(sdaPin), _sclPin(sclPin) {}
  void init() override;
  SensorReadings readings() override { return _readings; }
  bool buildSensorsHeader(char **szTemp) override;

private:
  int _sdaPin;
  int _sclPin;
  SCD41 _scd41;
  BBTemp _bbt;
  SensorReadings _readings;
};
#endif // INCLUDE_ENV_SENSOR

/// @brief Sensor instance for the running board, selected at compile time.
BaseSensor &sensor();
