#pragma once

#include <stdint.h>

//
// derive device-specific battery flags
//
#if defined(BOARD_TRMNL_X) || defined(BOARD_FRAME)
#define INCLUDE_BQ27427
#endif

/// @brief Abstract battery voltage source.
class BaseBattery {
public:
  virtual ~BaseBattery() = default;

  /// @brief Read the battery voltage.
  /// @return voltage in Volts, or a negative value if the reading failed
  virtual float readVoltage() = 0;
};

/// @brief Reports a fixed 4.2 V on boards with no battery sense hardware
///        (FAKE_BATTERY_VOLTAGE).
class FakeBattery : public BaseBattery {
public:
  float readVoltage() override;
};

/// @brief Reads the battery through a 1:2 resistor divider on an ADC pin.
class ADCBattery : public BaseBattery {
public:
  explicit ADCBattery(uint8_t pin);
  float readVoltage() override;

private:
  uint8_t _pin;
};

/// @brief Seeed boards gate the battery divider behind a load switch that must
///        be enabled before sampling the ADC (and disabled after to save power).
class SeeedBattery : public ADCBattery {
public:
  SeeedBattery(uint8_t pin, uint8_t switchPin, uint8_t switchOnLevel);
  float readVoltage() override;

private:
  uint8_t _switchPin;
  uint8_t _switchOnLevel;
};

#ifdef INCLUDE_BQ27427
/// @brief Reads the battery voltage from the BQ27427 fuel gauge.
class BQ27427Battery : public BaseBattery {
public:
  float readVoltage() override;
};
#endif // INCLUDE_BQ27427

/// @brief Battery instance for the running board, selected at compile time.
BaseBattery &battery();
