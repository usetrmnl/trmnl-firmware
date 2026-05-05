/******************************************************************************
*                                                                             *
*                                 Copyright by                                *
*                                                                             *
*                               Azoteq (Pty) Ltd                              *
*                           Republic of South Africa                          *
*                                                                             *
*                            Tel: +27(0)21 863 0033                           *
*                                 www.azoteq.com                              *
*                                                                             *
*******************************************************************************
* Refer to IQS323 datasheet for more information, available here:             *
* - https://www.azoteq.com/design/datasheets/                                 *
*******************************************************************************
*                       IQS323 - Registers & Memory Map                       *
*******************************************************************************/

#ifndef __IQS323_ADDRESSES_H
#define __IQS323_ADDRESSES_H

/* Device Information - Read Only */

/* VERSION DETAILS: 0x00 - 0x09 */
#define IQS323_MM_PROD_NUM                      0x00
#define IQS323_MM_MAJOR_VERSION_NUM             0x01
#define IQS323_MM_MINOR_VERSION_NUM             0x02

/* SYSTEM INFORMATION: 0x10 - 0x18 */
#define IQS323_MM_SYSTEM_STATUS		              0x10
#define IQS323_MM_GESTURES                      0x11
#define IQS323_MM_SLIDER_COORDINATES            0x12
#define IQS323_MM_CH0_COUNTS                    0x13
#define IQS323_MM_CH0_LTA                       0x14
#define IQS323_MM_CH1_COUNTS                    0x15
#define IQS323_MM_CH1_LTA                       0x16
#define IQS323_MM_CH2_COUNTS                    0x17
#define IQS323_MM_CH2_LTA                       0x18

/* RELEASE UI / MOVEMENT UI: 0x20 - 0x25 */
#define IQS323_MM_CH0_ACT_MOVE_LTA              0x20
#define IQS323_MM_CH1_ACT_MOVE_LTA              0x21
#define IQS323_MM_CH2_ACT_MOVE_LTA              0x22
#define IQS323_MM_CH0_DELTA_SNAP                0x23
#define IQS323_MM_CH1_DELTA_SNAP                0x24
#define IQS323_MM_CH2_DELTA_SNAP                0x25

/* SENSOR 0 SETUP: 0x30 - 0x39 */
#define IQS323_MM_S0_SETUP_0                    0x30
#define IQS323_MM_S0_CONV_FREQ_SETUP            0x31
#define IQS323_MM_S0_PROX_CONTROL               0x32
#define IQS323_MM_S0_PROX_INPUT_CONTROL         0x33
#define IQS323_MM_S0_PATTERN_DEFINITIONS        0x34
#define IQS323_MM_S0_PATTERN_SELECT             0x35
#define IQS323_MM_S0_ATI_SETUP        	        0x36
#define IQS323_MM_S0_ATI_BASE         	        0x37
#define IQS323_MM_S0_ATI_MULTI_SELECTION        0x38
#define IQS323_MM_S0_COMPENSATION               0x39

/* SENSOR 1 SETUP: 0x40 - 0x49 */
#define IQS323_MM_S1_SETUP_0                    0x40
#define IQS323_MM_S1_CONV_FREQ_SETUP            0x41
#define IQS323_MM_S1_PROX_CONTROL               0x42
#define IQS323_MM_S1_PROX_INPUT_CONTROL         0x43
#define IQS323_MM_S1_PATTERN_DEFINITIONS        0x44
#define IQS323_MM_S1_PATTERN_SELECT             0x45
#define IQS323_MM_S1_ATI_SETUP        	        0x46
#define IQS323_MM_S1_ATI_BASE         	        0x47
#define IQS323_MM_S1_ATI_MULTI_SELECTION        0x48
#define IQS323_MM_S1_COMPENSATION               0x49

/* SENSOR 2 SETUP: 0x50 - 0x59 */
#define IQS323_MM_S2_SETUP_0                    0x50
#define IQS323_MM_S2_CONV_FREQ_SETUP            0x51
#define IQS323_MM_S2_PROX_CONTROL               0x52
#define IQS323_MM_S2_PROX_INPUT_CONTROL         0x53
#define IQS323_MM_S2_PATTERN_DEFINITIONS        0x54
#define IQS323_MM_S2_PATTERN_SELECT             0x55
#define IQS323_MM_S2_ATI_SETUP        	        0x56
#define IQS323_MM_S2_ATI_BASE         	        0x57
#define IQS323_MM_S2_ATI_MULTI_SELECTION        0x58
#define IQS323_MM_S2_COMPENSATION               0x59

/* CHANNEL 0 SETUP: 0x60 - 0x64 */
#define IQS323_MM_CH0_SETUP_0                   0x60
#define IQS323_MM_CH0_PROX_SETTINGS             0x61
#define IQS323_MM_CH0_TOUCH_SETTINGS            0x62
#define IQS323_MM_CH0_FOLLOWER_WEIGHT           0x63
#define IQS323_MM_CH0_MOVEMENT_UI               0x64

/* CHANNEL 1 SETUP: 0x70 - 0x74 */
#define IQS323_MM_CH1_SETUP_0                   0x70
#define IQS323_MM_CH1_PROX_SETTINGS             0x71
#define IQS323_MM_CH1_TOUCH_SETTINGS            0x72
#define IQS323_MM_CH1_FOLLOWER_WEIGHT           0x73
#define IQS323_MM_CH1_MOVEMENT_UI               0x74

/* CHANNEL 2 SETUP: 0x80 - 0x84 */
#define IQS323_MM_CH2_SETUP_0                   0x80
#define IQS323_MM_CH2_PROX_SETTINGS             0x81
#define IQS323_MM_CH2_TOUCH_SETTINGS            0x82
#define IQS323_MM_CH2_FOLLOWER_WEIGHT           0x83
#define IQS323_MM_CH2_MOVEMENT_UI               0x84

/* SLIDER CONFIG: 0x90 - 0x98 */
#define IQS323_MM_SLIDER_SETUP_CALIBRATION      0x90
#define IQS323_MM_SLIDER_CALIBRATION_BOT_SPEED  0x91
#define IQS323_MM_SLIDER_TOP_SPEED              0x92
#define IQS323_MM_SLIDER_RESOLUTION             0x93
#define IQS323_MM_SLIDER_EN_MASK                0x94
#define IQS323_MM_SLIDER_EN_STATUS_POINTER      0x95
#define IQS323_MM_SLIDER_DELTA_LINK_0        	  0x96
#define IQS323_MM_SLIDER_DELTA_LINK_1         	0x97
#define IQS323_MM_SLIDER_DELTA_LINK_2           0x98

/* GESTURE CONFIG: 0xA0 - 0xA6 */
#define IQS323_MM_GESTURE_ENABLE                0xA0
#define IQS323_MM_GESTURE_MINIMUM_TIME          0xA1
#define IQS323_MM_GESTURE_MAX_TAP_TIME          0xA2
#define IQS323_MM_GESTURE_MAX_SWIPE_TIME        0xA3
#define IQS323_MM_GESTURE_MIN_HOLD_TIME         0xA4
#define IQS323_MM_GESTURE_MAX_TAP_DISTANCE      0xA5
#define IQS323_MM_GESTURE_MIN_SWIPE_DISTANCE    0xA6

/* FILTER BETAS: 0xB0 - 0xB4 */
#define IQS323_MM_COUNTS_FILTER_BETA            0xB0
#define IQS323_MM_LTA_FILTER_BETA               0xB1
#define IQS323_MM_LTA_FAST_FILTER_BETA          0xB2
#define IQS323_MM_ACT_MOVE_LTA_FILTER_BETA      0xB3
#define IQS323_MM_FAST_FILTER_BAND              0xB4

/* SYSTEM CONTROL: 0xC0 - 0xC5 */
#define IQS323_MM_SYSTEM_CONTROL                0xC0
#define IQS323_MM_NORMAL_POWER_REPORT_RATE      0xC1
#define IQS323_MM_LOW_POWER_REPORT_RATE         0xC2
#define IQS323_MM_ULP_REPORT_RATE               0xC3
#define IQS323_MM_HALT_MODE_REPORT_RATE         0xC4
#define IQS323_MM_POWER_MODE_TIMEOUT            0xC5

/* GENERAL: 0xD0 - 0xD4 */
#define IQS323_MM_OUTA_MASK                     0xD0
#define IQS323_MM_I2C_TRANS_TIMEOUT             0xD1
#define IQS323_MM_EVENT_TIMEOUTS                0xD2
#define IQS323_MM_EVENT_EN_ACTIVATION_THRESHOLD 0xD3
#define IQS323_MM_RELEASE_UI_MOVE_TIMEOUT       0xD4

/* I2C SETTINGS: 0xE0 - 0xE1 */
#define IQS323_MM_I2C_SETUP                     0xE0
#define IQS323_MM_HARDWARE_ID                   0xE1

#endif /* __IQS323_ADDRESSES_H */