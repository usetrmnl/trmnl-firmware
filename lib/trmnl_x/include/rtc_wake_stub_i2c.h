#pragma once

#include <stdint.h>
#include <stddef.h>

#define I2C_PORT I2C_NUM_0

void wake_stub_i2c_init(int sda, int scl);
void wake_stub_i2c_write(uint8_t addr7, const uint8_t *data, size_t len);
void wake_stub_i2c_read(uint8_t addr7, uint8_t reg, uint8_t *out, size_t len);