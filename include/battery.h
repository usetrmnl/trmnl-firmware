#pragma once

#include <stdint.h>

//
// derive device-specific battery flags
//
#if defined(BOARD_TRMNL_X)
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
/// @brief Reads the battery voltage from the BQ27427 fuel gauge, and gathers
///        the full set of gas gauge readings (SOC, health, current, etc.).
class BQ27427Battery : public BaseBattery {
public:
  /// @brief Battery voltage from the last gaugeInit() snapshot.
  /// @return voltage in Volts, or a negative value if the last reading failed
  float readVoltage() override { return _voltage; } // V

  /// @brief Connects to the BQ27427 gas gauge, (re)configures it if needed,
  ///        and gathers a validated snapshot of its readings. Locks the
  ///        shared I2C bus for the duration.
  void gaugeInit();

  int readSoc() const { return _soc; }                     // %
  int readHealth() const { return _health; }                // %
  int readCurrent() const { return _current; }               // mA
  float readTemperature() const { return _temperature; }     // C
  int readCapacityRemain() const { return _capacityRemain; } // mAh
  int readCapacityFull() const { return _capacityFull; }     // mAh

private:
  /// @brief Performs a reset if the BQ27427 current-sensing resistor is
  ///        specified with the wrong polarity, so Impedance Track relearns
  ///        from scratch when the golden file is reloaded. Call BEFORE
  ///        connectAndConfigure().
  void resetIfPolarityInverted();

  /// @brief Applies the correct polarity to the BQ27427 current-sensing
  ///        resistor as-needed. Call AFTER connectAndConfigure(), since a
  ///        reset/golden-file reload restores the inverted ROM default.
  void correctCurrentPolarity();

  /// @brief Checksum-verified read of one 32-byte gauge data-memory block in
  ///        NORMAL mode, with the block-load settle delays the gauge needs.
  bool readGaugeBlockVerified(uint8_t classID, uint8_t blockNum, uint8_t data[32]);

  /// @brief The CC Cal byte holding the coulomb-counter sign (offset 5,
  ///        bit 7), or -1 if it could not be read consistently.
  int32_t readCCCalSignByte();

  /// @brief Write the CC Cal sign byte through a CONFIG UPDATE session
  ///        (TRM section 4.1 sequence) and verify it stuck.
  bool writeCCCalSignByte(uint8_t value);

  int _soc = -1;
  int _health = -1;
  int _current = -1;
  float _temperature = -1;
  int _capacityRemain = -1;
  int _capacityFull = -1;
  float _voltage = -1;
};
#endif // INCLUDE_BQ27427

/// @brief Battery instance for the running board, selected at compile time.
#ifdef INCLUDE_BQ27427
BQ27427Battery &battery();
#else
BaseBattery &battery();
#endif
