#ifdef BOARD_TRMNL_X

#include <Arduino.h>

#include "Wire.h"

#include "accelerometer.h"
#include <trmnl_log.h>

// ############################ ACCELEROMETER #############################

// Volatile flag for interrupt
volatile bool orientation_interrupt_occurred = false;

/**
 * INT1 interrupt handler
 */
void IRAM_ATTR bma530_int1_handler() {
    orientation_interrupt_occurred = true;
}

/**
 * I2C read function for BMA530
 */
int8_t bma5_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr) {
    if (reg_data == NULL) {
        return -1;
    }

    Wire.beginTransmission(BMA530_I2C_ADDRESS);
    Wire.write(reg_addr);

    if (Wire.endTransmission(false) != 0) {
        return -1;
    }

    Wire.requestFrom(BMA530_I2C_ADDRESS, (uint8_t)length);

    for (uint32_t i = 0; i < length; i++) {
        if (Wire.available()) {
            reg_data[i] = Wire.read();
        } else {
            return -1;
        }
    }

    return 0;
}

/**
 * I2C write function for BMA530
 */
int8_t bma5_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr) {
    if (reg_data == NULL) {
        return -1;
    }

    Wire.beginTransmission(BMA530_I2C_ADDRESS);
    Wire.write(reg_addr);

    for (uint32_t i = 0; i < length; i++) {
        Wire.write(reg_data[i]);
    }

    if (Wire.endTransmission() != 0) {
        return -1;
    }

    return 0;
}

/**
 * Delay function for BMA530
 */
void bma5_delay_us(uint32_t period_us, void *intf_ptr) {
    delayMicroseconds(period_us);
}

/**
 * Initialize the BMA530 device
 */
int8_t bma530_init_device(struct bma5_dev *dev) {
    int8_t rslt;

    // Configure device structure for I2C interface
    dev->intf = BMA5_I2C_INTF;
    dev->bus_read = bma5_i2c_read;
    dev->bus_write = bma5_i2c_write;
    dev->delay_us = bma5_delay_us;
    dev->intf_ptr = NULL;
    dev->context = BMA5_SMARTPHONE;  // Or BMA5_WEARABLE/BMA5_HEARABLE

    // Initialize the sensor
    rslt = bma530_init(dev);
    if (rslt != BMA5_OK) {
        Log_info_serial("BMA530 initialization failed: %d", rslt);
        return rslt;
    }

    Log_info_serial("BMA530 initialized successfully");
    Log_info_serial("Chip ID: 0x%02X", dev->chip_id);

    return BMA5_OK;
}

/**
 * Configure accelerometer for low power mode
 */
int8_t bma530_configure_low_power_mode(struct bma5_dev *dev) {
    int8_t rslt;
    struct bma5_acc_conf acc_cfg;
    uint8_t sensor_ctrl;

    // Get current accelerometer configuration
    rslt = bma5_get_acc_conf(&acc_cfg, dev);
    if (rslt != BMA5_OK) {
        Log_info_serial("Failed to get acc config: %d", rslt);
        return rslt;
    }

    // Configure for Low Power Mode
    acc_cfg.power_mode = BMA5_POWER_MODE_LPM;        // Low Power Mode (duty cycling)
    acc_cfg.acc_odr = BMA5_ACC_ODR_HZ_25;            // 25 Hz ODR (valid for LPM)
    acc_cfg.acc_bwp = BMA5_ACC_BWP_NORM_AVG4;        // Normal averaging
    acc_cfg.acc_range = BMA5_ACC_RANGE_MAX_4G;       // 4G range
    acc_cfg.noise_mode = BMA5_NOISE_MODE_LOWER_POWER;// Lower power noise mode
    acc_cfg.acc_drdy_int_auto_clear = BMA5_ACC_DRDY_INT_AUTO_CLEAR_ENABLED;

    // Apply configuration
    rslt = bma5_set_acc_conf(&acc_cfg, dev);
    if (rslt != BMA5_OK) {
        Log_info_serial("Failed to set acc config: %d", rslt);
        return rslt;
    }

    // Enable accelerometer
    sensor_ctrl = BMA5_SENSOR_CTRL_ENABLE;
    rslt = bma5_set_acc_conf_0(sensor_ctrl, dev);
    if (rslt != BMA5_OK) {
        Log_info_serial("Failed to enable accelerometer: %d", rslt);
        return rslt;
    }

    Log_info_serial("Low power mode configured:");
    Log_info_serial("  Power Mode: Low Power Mode (Duty Cycling)");
    Log_info_serial("  ODR: 25 Hz");
    Log_info_serial("  Range: 4G");
    Log_info_serial("  Noise Mode: Lower Power");

    return BMA5_OK;
}

/**
 * Configure orientation detection
 */
int8_t bma530_configure_orientation(struct bma5_dev *dev) {
    int8_t rslt;
    struct bma530_orient conf;
    struct bma530_feat_eng_gpr_0 gpr_0;
    uint8_t gpr_ctrl_host = BMA5_ENABLE;

    // Get current orientation configuration
    rslt = bma530_get_orient_config(&conf, dev);
    if (rslt != BMA5_OK) {
        Log_info_serial("Failed to get orientation config: %d", rslt);
        return rslt;
    }

    // Configure orientation parameters
    conf.ud_en = 0;             // Enable upside-down (face up/down) detection
    conf.mode = 0;              // Symmetric mode
    conf.blocking = 0;          // No blocking during movement
    conf.theta = 0x27;          // Tilt angle threshold (default: 0x27 = 39)
    conf.hold_time = 0x5;       // Hold time for confirmation (default: 0x5)
    conf.slope_thres = 0xCD;    // Slope threshold to prevent false detection (default: 0xCD = 205)
    conf.hysteresis = 0x20;     // Hysteresis value (default: 0x20 = 32)

    // Apply orientation configuration
    rslt = bma530_set_orient_config(&conf, dev);
    if (rslt != BMA5_OK) {
        Log_info_serial("Failed to set orientation config: %d", rslt);
        return rslt;
    }

    // Enable orientation feature in feature engine
    rslt = bma530_get_feat_eng_gpr_0(&gpr_0, dev);
    if (rslt != BMA5_OK) {
        Log_info_serial("Failed to get feature engine GPR: %d", rslt);
        return rslt;
    }

    // Disable ALL other features - ensure ONLY orientation triggers interrupts
    gpr_0.gen_int1_en = 0x00;
    gpr_0.gen_int2_en = 0x00;
    gpr_0.gen_int3_en = 0x00;
    gpr_0.step_en = 0x00;
    gpr_0.sig_mo_en = 0x00;
    gpr_0.tilt_en = 0x00;
    gpr_0.acc_foc_en = 0x00;

    // Enable ONLY orientation detection
    gpr_0.orient_en = 0x01;

    rslt = bma530_set_feat_eng_gpr_0(&gpr_0, dev);
    if (rslt != BMA5_OK) {
        Log_info_serial("Failed to enable orientation feature: %d", rslt);
        return rslt;
    }

    // Set feature engine control to host
    rslt = bma5_set_regs(BMA5_REG_FEAT_ENG_GPR_CTRL, &gpr_ctrl_host, 1, dev);
    if (rslt != BMA5_OK) {
        Log_info_serial("Failed to set feature engine control: %d", rslt);
        return rslt;
    }

    Log_info_serial("Orientation detection configured:");
    Log_info_serial("  Face up/down detection: Enabled");
    Log_info_serial("  Mode: Symmetric");
    Log_info_serial("  Theta: 0x%02X", conf.theta);
    Log_info_serial("  Hold time: 0x%02X", conf.hold_time);
    Log_info_serial("  Slope threshold: 0x%02X", conf.slope_thres);
    Log_info_serial("  Hysteresis: 0x%02X", conf.hysteresis);

    return BMA5_OK;
}

/**
 * Configure INT1 pin for orientation interrupts
 */
int8_t bma530_configure_int1(struct bma5_dev *dev) {
    int8_t rslt;
    struct bma530_int_map int_map;
    struct bma5_int_conf_types int_config;

    // Get current interrupt mapping
    rslt = bma530_get_int_map(&int_map, dev);
    if (rslt != BMA5_OK) {
        Log_info_serial("Failed to get interrupt mapping: %d", rslt);
        return rslt;
    }

    // Clear ALL interrupt mappings first to ensure only orientation triggers
    int_map.acc_drdy_int_map = BMA530_ACC_DRDY_INT_MAP_UNMAPPED;
    int_map.fifo_wm_int_map = BMA530_FIFO_WM_INT_MAP_UNMAPPED;
    int_map.fifo_full_int_map = BMA530_FIFO_FULL_INT_MAP_UNMAPPED;
    int_map.gen_int1_int_map = BMA530_GEN_INT1_INT_MAP_UNMAPPED;
    int_map.gen_int2_int_map = BMA530_GEN_INT2_INT_MAP_UNMAPPED;
    int_map.gen_int3_int_map = BMA530_GEN_INT3_INT_MAP_UNMAPPED;
    int_map.step_det_int_map = BMA530_STEP_DET_INT_MAP_UNMAPPED;
    int_map.step_cnt_int_map = BMA530_STEP_CNT_INT_MAP_UNMAPPED;
    int_map.sig_mo_int_map = BMA530_SIG_MO_INT_MAP_UNMAPPED;
    int_map.tilt_int_map = BMA530_TILT_INT_MAP_UNMAPPED;
    int_map.acc_foc_int_map = BMA530_ACC_FOC_INT_MAP_UNMAPPED;
    int_map.feat_eng_err_int_map = BMA530_FEAT_ENG_ERR_INT_MAP_UNMAPPED;

    // Map ONLY orientation interrupt to INT1 pin
    int_map.orient_int_map = BMA530_ORIENT_INT_MAP_INT1;

    rslt = bma530_set_int_map(&int_map, dev);
    if (rslt != BMA5_OK) {
        Log_info_serial("Failed to set interrupt mapping: %d", rslt);
        return rslt;
    }

    // Configure INT1 hardware pin
    int_config.int_src = BMA5_INT_1;  // Configure INT1

    rslt = bma5_get_int_conf(&int_config, 1, dev);
    if (rslt != BMA5_OK) {
        Log_info_serial("Failed to get INT1 config: %d", rslt);
        return rslt;
    }

    // INT1 pin configuration (external pull-up to 3.3V on hardware)
    int_config.int_conf.int_mode = BMA5_INT1_MODE_LATCHED;       // Latched mode (stays low until cleared)
    int_config.int_conf.int_od = BMA5_INT1_OD_OPEN_DRAIN;        // Open-drain (for external pull-up)
    int_config.int_conf.int_lvl = BMA5_INT1_LVL_ACTIVE_LOW;      // Active low (pulls to GND)

    rslt = bma5_set_int_conf(&int_config, 1, dev);
    if (rslt != BMA5_OK) {
        Log_info_serial("Failed to set INT1 config: %d", rslt);
        return rslt;
    }

    Log_info_serial("INT1 configured:");
    Log_info_serial("  Orientation interrupt mapped to INT1");
    Log_info_serial("  Mode: Latched");
    Log_info_serial("  Output: Open-Drain");
    Log_info_serial("  Level: Active Low (pulls to GND with external pull-up)");

    return BMA5_OK;
}

/**
 * Process orientation change
 */
void bma530_process_orientation(struct bma5_dev *dev) {
    int8_t rslt;
    struct bma530_int_status_types int_status;
    struct bma530_feat_eng_feat_out feat_out;

    // Set interrupt source to INT1
    int_status.int_src = BMA530_INT_STATUS_INT1;

    // Read interrupt status
    rslt = bma530_get_int_status(&int_status, 1, dev);
    if (rslt != BMA5_OK) {
        Log_info_serial("Failed to read interrupt status: %d", rslt);
        return;
    }

    // Check if orientation interrupt occurred
    if (int_status.int_status.orient_int_status & BMA5_ENABLE) {
        Log_info_serial("*** Orientation change detected! ***");

        // Read orientation output values
        rslt = bma530_get_feat_eng_feature_out(&feat_out, dev);
        if (rslt != BMA5_OK) {
            Log_info_serial("Failed to read orientation data: %d", rslt);
            return;
        }

        uint8_t portrait_landscape = feat_out.orientation_portrait_landscape;
        uint8_t face_up_down = feat_out.orientation_face_up_down;

        // Print orientation state
        const char *pl;
        switch (portrait_landscape) {
            case PORTRAIT_UP_RIGHT: pl = "Portrait Upright"; break;
            case LANDSCAPE_LEFT:    pl = "Landscape Left"; break;
            case PORTRAIT_UP_DOWN:  pl = "Portrait Upside Down"; break;
            case LANDSCAPE_RIGHT:   pl = "Landscape Right"; break;
            default:                pl = "Unknown";
        }
        const char *fud;
        switch (face_up_down) {
            case FACE_UP:   fud = "Face Up"; break;
            case FACE_DOWN: fud = "Face Down"; break;
            default:        fud = "Unknown";
        }
        Log_info_serial("Orientation: %s | %s", pl, fud);

        // Clear interrupt status
        rslt = bma530_set_int_status(&int_status, 1, dev);
        if (rslt != BMA5_OK) {
            Log_info_serial("Failed to clear interrupt status: %d", rslt);
        }
    }
}

// ############################ ACCELEROMETER #############################
#endif // BOARD_TRMNL_X
