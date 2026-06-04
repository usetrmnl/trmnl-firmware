#pragma once

// Override Unity's output hooks so that UNITY_OUTPUT_COMPLETE is a no-op,
// preventing Arduino-ESP32's default unity_flush()/cleanup from tearing
// down USB after UNITY_END().
//
// Activated by -D UNITY_INCLUDE_CONFIG_H in platformio.ini.
//
// Source: https://github.com/platformio/platformio-core/issues/5359#issuecomment-4240210598

#define UNITY_OUTPUT_COMPLETE()
