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

// Rotation, in degrees, of the BMA530 die relative to the device's physical
// landscape/portrait edges on the TRMNL X board.
#define BMA530_MOUNT_OFFSET_DEG 45.0f

// ORIENTATION_1.theta raw register value
#define BMA530_ORIENT_THETA_RAW 0x27

void bma530_int1_handler();

int8_t bma5_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
int8_t bma5_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);
void bma5_delay_us(uint32_t period_us, void *intf_ptr);

int8_t bma530_init_device(struct bma5_dev *dev);

int8_t bma530_configure_low_power_mode(struct bma5_dev *dev);

// Polls SENSOR_STATUS.acc_data_rdy until the accelerometer's first sample
// after being enabled becomes valid, or timeout_ms elapses.
int8_t bma530_wait_for_data_ready(struct bma5_dev *dev, uint32_t timeout_ms);

// Initializing BMA530
int8_t bma530_wake_and_init(struct bma5_dev *dev);

typedef enum {
    BMA530_ORIENT_READ_FAILED, // I2C/read error
    BMA530_ORIENT_FLAT,        // valid read, but too flat to trust the in-plane angle
    BMA530_ORIENT_VALID,       // valid, trustworthy - orientation_out was written
} bma530_orient_read_t;

// Locked orientation read: computes the current orientation via
// bma530_compute_orientation(), gated against the flat-state case.
// orientation_out is only written when the return value is BMA530_ORIENT_VALID.
bma530_orient_read_t bma530_read_orientation(struct bma5_dev *dev, uint8_t *orientation_out);

int8_t bma530_configure_orientation(struct bma5_dev *dev);

int8_t bma530_configure_int1(struct bma5_dev *dev);

void bma530_process_orientation(struct bma5_dev *dev);

// Computes device orientation directly from raw accelerometer data, correcting
// for the BMA530_MOUNT_OFFSET_DEG
int8_t bma530_compute_orientation(struct bma5_dev *dev, uint8_t *orientation_out, bool *is_flat_out,
                                   float *raw_phi_deg_out, float *corrected_phi_deg_out);

// Returns a lowercase, human-readable name for a PORTRAIT_UP_RIGHT / LANDSCAPE_LEFT /
// PORTRAIT_UP_DOWN / LANDSCAPE_RIGHT value, e.g. for logging or API requests.
const char *bma530_orientation_to_string(uint8_t orientation);