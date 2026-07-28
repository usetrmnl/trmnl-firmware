#pragma once

#include <hardware_types.h>
#include <stdint.h>

//
// derive device-specific power flags
//
#if defined(BOARD_TRMNL_X)
#define INCLUDE_TCA9535_POWER
#elif defined(BOARD_TRMNL_GEN2)
#define INCLUDE_GPIO_POWER
#elif defined(BOARD_ZECTRIX)
#define INCLUDE_ZECTRIX_POWER
#endif

/// @brief USB/charger status source. The base class reports UNKNOWN, for
///        boards whose charger hardware can't be read. Safe to call on any
///        board with no guards at the call site.
class BasePower {
public:
  virtual ~BasePower() = default;

  /// @brief Resolve USB/VBUS presence.
  virtual UsbStatus usbStatus() { return UsbStatus::UNKNOWN; }

  /// @brief Resolve the battery charge state reported by the charger IC.
  virtual ChargingStatus chargingStatus() { return ChargingStatus::UNKNOWN; }
};

/// @brief Reads the BQ25616 charger's PG/STAT lines on dedicated GPIOs
///        (open-drain, LOW = active).
class GPIOPower : public BasePower {
public:
  GPIOPower(uint8_t pgPin, uint8_t statPin);
  UsbStatus usbStatus() override;
  ChargingStatus chargingStatus() override;

private:
  uint8_t _pgPin;
  uint8_t _statPin;
};

#ifdef INCLUDE_ZECTRIX_POWER
/// @brief Reads the ZecTrix charger's active-low charging signal and
///        active-high charge-complete signal on dedicated GPIOs.
class ZectrixPower : public BasePower {
public:
  ZectrixPower(uint8_t chargingPin, uint8_t chargedPin);
  UsbStatus usbStatus() override;
  ChargingStatus chargingStatus() override;

private:
  uint8_t _chargingPin;
  uint8_t _chargedPin;
};
#endif // INCLUDE_ZECTRIX_POWER

#ifdef INCLUDE_TCA9535_POWER
class FASTEPD;

/// @brief Reads the BQ25616 charger's PG/STAT lines through the TCA9535 I/O
///        expander (reached via the FastEPD io* helpers).
class TCA9535Power : public BasePower {
public:
  TCA9535Power(FASTEPD &bbep, uint8_t pgPin, uint8_t statPin);
  UsbStatus usbStatus() override;
  ChargingStatus chargingStatus() override;

private:
  FASTEPD &_bbep;
  uint8_t _pgPin;
  uint8_t _statPin;
};
#endif // INCLUDE_TCA9535_POWER

/// @brief Power status source for the running board, selected at compile time.
BasePower &power();
