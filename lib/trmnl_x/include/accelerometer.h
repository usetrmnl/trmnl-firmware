#pragma once

#include <stdint.h>

#include "bma530_features.h"

#define BMA530_I2C_ADDRESS  0x18

// Orientation output definitions
#define FACE_UP            0x00
#define FACE_DOWN          0x01
#define PORTRAIT_UP_RIGHT  0x00
#define LANDSCAPE_LEFT     0x01
#define PORTRAIT_UP_DOWN   0x02
#define LANDSCAPE_RIGHT    0x03

void bma530_int1_handler();

int8_t bma5_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
int8_t bma5_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);
void bma5_delay_us(uint32_t period_us, void *intf_ptr);

int8_t bma530_init_device(struct bma5_dev *dev);

int8_t bma530_configure_low_power_mode(struct bma5_dev *dev);

int8_t bma530_configure_orientation(struct bma5_dev *dev);

int8_t bma530_configure_int1(struct bma5_dev *dev);

void bma530_process_orientation(struct bma5_dev *dev);