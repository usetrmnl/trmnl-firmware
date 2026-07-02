#pragma once

// Read the onboard SHT4x over I2C. Returns true and writes tempC (°C) and
// humidity (%RH) on success; false on bus error or CRC failure.
bool sht4x_read(float &tempC, float &humidity);
