#pragma once

//
// Device-agnostic hardware types and enums
//

/// USB / VBUS presence. UNKNOWN on boards without USB-detection support.
enum class UsbStatus
{
    UNKNOWN,      // not implemented / not readable on this board
    CONNECTED,    // VBUS (USB) power present
    DISCONNECTED, // running on battery
};

/// Battery charge state reported by the charger IC (BQ25616 on TRMNL X / gen2).
enum class ChargingStatus
{
    UNKNOWN,      // not implemented / not readable on this board
    CHARGING,     // actively charging
    NOT_CHARGING, // charge complete, disabled, or no battery
    FAULT,        // charger fault — not yet implemented
};