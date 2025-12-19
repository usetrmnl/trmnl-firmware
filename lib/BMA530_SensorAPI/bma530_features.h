/**
* Copyright (c) 2024 Bosch Sensortec GmbH. All rights reserved.
*
* BSD-3-Clause
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* @file       bma530_features.h
* @date       2024-07-29
* @version    v4.2.0
*
*/

#ifndef _BMA530_FEATURES_H
#define _BMA530_FEATURES_H

/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************/
/****************************** Header files **********************************/
/******************************************************************************/

#include "bma530.h"
#include "bma530_context.h"

/******************************************************************************/
/********************** Register macros for bit masking ***********************/
/******************************************************************************/

/*! Feature conf error macros */
#define BMA530_GEN_INT1_CONF_ERR_MSK             UINT8_C(0x01)
#define BMA530_GEN_INT1_CONF_ERR_POS             UINT8_C(0)

#define BMA530_GEN_INT2_CONF_ERR_MSK             UINT8_C(0x02)
#define BMA530_GEN_INT2_CONF_ERR_POS             UINT8_C(1)

#define BMA530_GEN_INT3_CONF_ERR_MSK             UINT8_C(0x04)
#define BMA530_GEN_INT3_CONF_ERR_POS             UINT8_C(2)

#define BMA530_STEP_CONF_ERR_MSK                 UINT8_C(0x08)
#define BMA530_STEP_CONF_ERR_POS                 UINT8_C(3)

#define BMA530_SIG_MO_CONF_ERR_MSK               UINT8_C(0x10)
#define BMA530_SIG_MO_CONF_ERR_POS               UINT8_C(4)

#define BMA530_TILT_CONF_ERR_MSK                 UINT8_C(0x20)
#define BMA530_TILT_CONF_ERR_POS                 UINT8_C(5)

#define BMA530_ORIENT_CONF_ERR_MSK               UINT8_C(0x40)
#define BMA530_ORIENT_CONF_ERR_POS               UINT8_C(6)

#define BMA530_ACC_FOC_CONF_ERR_MSK              UINT8_C(0x80)
#define BMA530_ACC_FOC_CONF_ERR_POS              UINT8_C(7)

/*! Step counter feature macros */

#define BMA530_SC_DEFAULT_WATERMARK_LEVEL        UINT16_C(0x0)
#define BMA530_SC_DEFAULT_RESET_COUNTER          UINT8_C(0x0)
#define BMA530_SC_DEFAULT_SD_EN                  UINT8_C(0x1)
#define BMA530_SC_DEFAULT_SC_EN                  UINT8_C(0x1)
#define BMA530_SC_DEFAULT_FILTER_COEFF_B_2       UINT16_C(0x55F)
#define BMA530_SC_DEFAULT_FILTER_COEFF_B_1       UINT16_C(0xABE)
#define BMA530_SC_DEFAULT_FILTER_COEFF_B_0       UINT16_C(0x55F)
#define BMA530_SC_DEFAULT_FILTER_COEFF_A_2       UINT16_C(0xE897)
#define BMA530_SC_DEFAULT_FILTER_COEFF_A_1       UINT16_C(0x41EF)
#define BMA530_SC_DEFAULT_FILTER_COEFF_SCALE_A   UINT8_C(0xE)
#define BMA530_SC_DEFAULT_FILTER_COEFF_SCALE_B   UINT8_C(0xE)

/*! An interrupt will be triggered every time the difference in number of
 * steps counted from last event is equal to (set value * 20). If 0, the interrupt is disabled */
#define BMA530_SC_WATERMARK_LVL_MSK              UINT16_C(0x03FF)
#define BMA530_SC_WATERMARK_LVL_POS              UINT8_C(0)

/*! Reset the accumulated step count value */
#define BMA530_SC_RESET_COUNTER_MSK              UINT16_C(0x0400)
#define BMA530_SC_RESET_COUNTER_POS              UINT8_C(10)

/*! Enable step detector */
#define BMA530_SC_SD_EN_MSK                      UINT16_C(0x0800)
#define BMA530_SC_SD_EN_POS                      UINT8_C(11)

/*! Enable step counter */
#define BMA530_SC_SC_EN_MSK                      UINT16_C(0x1000)
#define BMA530_SC_SC_EN_POS                      UINT8_C(12)

/*! Threshold for upper peak of acceleration magnitude for step detection */
#define BMA530_SC_ENVELOPE_UP_THRES_MSK          UINT16_C(0xFFFF)
#define BMA530_SC_ENVELOPE_UP_THRES_POS          UINT8_C(0)

/*! Adaptive upper peak threshold decay coefficient */
#define BMA530_SC_ENVELOPE_UP_DECAY_COEFF_MSK    UINT16_C(0xFFFF)
#define BMA530_SC_ENVELOPE_UP_DECAY_COEFF_POS    UINT8_C(0)

/*! Threshold for lower peak of acceleration magnitude for step detection */
#define BMA530_SC_ENVELOPE_DOWN_THRES_MSK        UINT16_C(0xFFFF)
#define BMA530_SC_ENVELOPE_DOWN_THRES_POS        UINT8_C(0)

/*! Adaptive lower peak threshold decay coefficient */
#define BMA530_SC_ENVELOPE_DOWN_DECAY_COEFF_MSK  UINT16_C(0xFFFF)
#define BMA530_SC_ENVELOPE_DOWN_DECAY_COEFF_POS  UINT8_C(0)

/*! Exponential smoothing filter coefficient for computing mean of acceleration magnitude */
#define BMA530_SC_ACC_MEAN_DECAY_COEFF_MSK       UINT16_C(0xFFFF)
#define BMA530_SC_ACC_MEAN_DECAY_COEFF_POS       UINT8_C(0)

/*! Exponential smoothing filter coefficient for computing mean duration between steps */
#define BMA530_SC_STEP_DUR_MEAN_DECAY_COEFF_MSK  UINT16_C(0xFFFF)
#define BMA530_SC_STEP_DUR_MEAN_DECAY_COEFF_POS  UINT8_C(0)

/*! Minimum number of consecutive steps to be detected for updating step count */
#define BMA530_SC_STEP_BUFFER_SIZE_MSK           UINT16_C(0x000F)
#define BMA530_SC_STEP_BUFFER_SIZE_POS           UINT8_C(0)

/*! Enable or disable cascading of filters */
#define BMA530_SC_FILTER_CASCADE_ENABLED_MSK     UINT16_C(0x0010)
#define BMA530_SC_FILTER_CASCADE_ENABLED_POS     UINT8_C(4)

/*! Scale factor for the step count to handle overcounting to undercounting */
#define BMA530_SC_STEP_COUNTER_INCREAMENT_MSK    UINT16_C(0x3FE0)
#define BMA530_SC_STEP_COUNTER_INCREAMENT_POS    UINT8_C(5)

/*! Enable or disable detection of half step */
#define BMA530_SC_EN_HALF_STEP_MSK               UINT16_C(0x4000)
#define BMA530_SC_EN_HALF_STEP_POS               UINT8_C(14)

/*! Minimum duration between two consecutive steps while walking */
#define BMA530_SC_PEAK_DUR_MIN_WALKING_MSK       UINT16_C(0x00FF)
#define BMA530_SC_PEAK_DUR_MIN_WALKING_POS       UINT8_C(0)

/*! Minimum duration between two consecutive steps while running */
#define BMA530_SC_PEAK_DUR_MIN_RUNNING_MSK       UINT16_C(0xFF00)
#define BMA530_SC_PEAK_DUR_MIN_RUNNING_POS       UINT8_C(8)

/* Ratio of acceleration magnitude variance during running to walking */
#define BMA530_SC_ACTIVITY_DET_FACT_MSK          UINT16_C(0x000F)
#define BMA530_SC_ACTIVITY_DET_FACT_POS          UINT8_C(0)

/*! Acceleration magnitude variance threshold for activity classification */
#define BMA530_SC_ACTIVITY_DET_THRES_MSK         UINT16_C(0xFFF0)
#define BMA530_SC_ACTIVITY_DET_THRES_POS         UINT8_C(4)

/*! Maximum duration between two consecutive step occurance */
#define BMA530_SC_STEP_DUR_MAX_MSK               UINT16_C(0x00FF)
#define BMA530_SC_STEP_DUR_MAX_POS               UINT8_C(0)

/*! Maximum duration since last step where next step shall be detected to add missed step, if any */
#define BMA530_SC_STEP_DUR_WINDOW_MSK            UINT16_C(0xFF00)
#define BMA530_SC_STEP_DUR_WINDOW_POS            UINT8_C(8)

/*! Enable or disable post-processing for duration between steps */
#define BMA530_SC_EN_STEP_DUR_PP_MSK             UINT16_C(0x0001)
#define BMA530_SC_EN_STEP_DUR_PP_POS             UINT8_C(0)

/*! Scale factor for mean step duration for step processing */
#define BMA530_SC_STEP_DUR_THRES_MSK             UINT16_C(0x000E)
#define BMA530_SC_STEP_DUR_THRES_POS             UINT8_C(1)

/*! Enable or disable post-processing of step based on mean crossing */
#define BMA530_SC_EN_MCR_PP_MSK                  UINT16_C(0x0010)
#define BMA530_SC_EN_MCR_PP_POS                  UINT8_C(4)

/*! Threshold for number of mean crossings between two consecutive steps */
#define BMA530_SC_MCR_THRES_MSK                  UINT16_C(0x03E0)
#define BMA530_SC_MCR_THRES_POS                  UINT8_C(5)

/*! Filter coefficient B2 of 2nd order IIR filter */
#define BMA530_SC_FILTER_COEFF_B_2_MSK           UINT16_C(0xFFFF)
#define BMA530_SC_FILTER_COEFF_B_2_POS           UINT8_C(0)

/*! Filter coefficient B1 of 2nd order IIR filter */
#define BMA530_SC_FILTER_COEFF_B_1_MSK           UINT16_C(0xFFFF)
#define BMA530_SC_FILTER_COEFF_B_1_POS           UINT8_C(0)

/*! Filter coefficient B0 of 2nd order IIR filter */
#define BMA530_SC_FILTER_COEFF_B_0_MSK           UINT16_C(0xFFFF)
#define BMA530_SC_FILTER_COEFF_B_0_POS           UINT8_C(0)

/*! Filter coefficient A2 of 2nd order IIR filter */
#define BMA530_SC_FILTER_COEFF_A_2_MSK           UINT16_C(0xFFFF)
#define BMA530_SC_FILTER_COEFF_A_2_POS           UINT8_C(0)

/*! Filter coefficient A1 of 2nd order IIR filter */
#define BMA530_SC_FILTER_COEFF_A_1_MSK           UINT16_C(0xFFFF)
#define BMA530_SC_FILTER_COEFF_A_1_POS           UINT8_C(0)

/*! Scaling of filter coefficients A of 2nd order IIR filter */
#define BMA530_SC_FILTER_COEFF_SCALE_A_MSK       UINT16_C(0x00FF)
#define BMA530_SC_FILTER_COEFF_SCALE_A_POS       UINT8_C(0)

/*! Scaling of filter coefficients B of 2nd order IIR filter */
#define BMA530_SC_FILTER_COEFF_SCALE_B_MSK       UINT16_C(0xFF00)
#define BMA530_SC_FILTER_COEFF_SCALE_B_POS       UINT8_C(8)

/*! Size of the segment for detection of significant motion of the device */
#define BMA530_SIG_MOT_BLOCK_SIZE_MSK            UINT16_C(0xFFFF)
#define BMA530_SIG_MOT_BLOCK_SIZE_POS            UINT8_C(0)

/*! Minimum value of the peak to peak acceleration magnitude */
#define BMA530_SIG_MOT_P2P_MIN_MSK               UINT16_C(0x03FF)
#define BMA530_SIG_MOT_P2P_MIN_POS               UINT8_C(0)

/*! Minimum number of mean crossing per second in acceleration magnitude */
#define BMA530_SIG_MOT_MCR_MIN_MSK               UINT16_C(0xFC00)
#define BMA530_SIG_MOT_MCR_MIN_POS               UINT8_C(10)

/*! Maximum value of the peak to peak acceleration magnitude */
#define BMA530_SIG_MOT_P2P_MAX_MSK               UINT16_C(0x03FF)
#define BMA530_SIG_MOT_P2P_MAX_POS               UINT8_C(0)

/*! Maximum number of mean crossing per second in acceleration magnitude */
#define BMA530_SIG_MOT_MCR_MAX_MSK               UINT16_C(0xFC00)
#define BMA530_SIG_MOT_MCR_MAX_POS               UINT8_C(10)

/*! Duration for which the acceleration vector is averaged to be reference vector */
#define BMA530_TILT_SEGMENT_SIZE_MSK             UINT16_C(0x00FF)
#define BMA530_TILT_SEGMENT_SIZE_POS             UINT8_C(0)

/*! Minimum angle by which the device shall be tilted for event detection */
#define BMA530_TILT_MIN_TILT_ANGLE_MSK           UINT16_C(0xFF00)
#define BMA530_TILT_MIN_TILT_ANGLE_POS           UINT8_C(8)

/*! Exponential smoothing coefficient for computing low-pass mean of acceleration vector */
#define BMA530_TILT_BETA_ACC_MEAN_MSK            UINT16_C(0xFFFF)
#define BMA530_TILT_BETA_ACC_MEAN_POS            UINT8_C(0)

/*! Selection of upside down orientation detection  */
#define BMA530_ORIENT_UD_EN_MSK                  UINT16_C(0x0001)
#define BMA530_ORIENT_UD_EN_POS                  UINT8_C(0)

/*! Selection of mode for orientation spread in the detection plane */
#define BMA530_ORIENT_MODE_MSK                   UINT16_C(0x0006)
#define BMA530_ORIENT_MODE_POS                   UINT8_C(1)

/*! Blocking allows to prevent change of orientation during large movement of device */
#define BMA530_ORIENT_BLOCKING_MSK               UINT16_C(0x0018)
#define BMA530_ORIENT_BLOCKING_POS               UINT8_C(3)

/*! Maximum allowed tilt angle for device to be in flat state */
#define BMA530_ORIENT_THETA_MSK                  UINT16_C(0x07E0)
#define BMA530_ORIENT_THETA_POS                  UINT8_C(5)

/*! Minimum duration the device shall be in new orientation for change detection */
#define BMA530_ORIENT_HOLD_TIME_MSK              UINT16_C(0xF800)
#define BMA530_ORIENT_HOLD_TIME_POS              UINT8_C(11)

/*! Minimum slope between consecutive acceleration samples to pervent the change of orientation during large movement */
#define BMA530_ORIENT_SLOPE_THRES_MSK            UINT16_C(0x00FF)
#define BMA530_ORIENT_SLOPE_THRES_POS            UINT8_C(0)

/*! Hysteresis of acceleration for orientation change detection */
#define BMA530_ORIENT_HYSTERESIS_MSK             UINT16_C(0xFF00)
#define BMA530_ORIENT_HYSTERESIS_POS             UINT8_C(8)

/*! Android compatibility mode macros */
#define BMA530_ANDROID_COMP_MSK                  UINT8_C(0x01)

/*! Feature axis macros */
#define BMA530_FEAT_AXIS_EX_MSK                  UINT8_C(0x0E)
#define BMA530_FEAT_AXIS_EX_POS                  UINT8_C(1)

#define BMA530_FEAT_X_INV_MSK                    UINT8_C(0x10)
#define BMA530_FEAT_X_INV_POS                    UINT8_C(4)

#define BMA530_FEAT_Y_INV_MSK                    UINT8_C(0x20)
#define BMA530_FEAT_Y_INV_POS                    UINT8_C(5)

#define BMA530_FEAT_Z_INV_MSK                    UINT8_C(0x40)
#define BMA530_FEAT_Z_INV_POS                    UINT8_C(6)

/*! Generic interrupt macors */
/*! Minimum/maximum slope of acceleration signal for interrupt detection based on selected motion criterion. */
#define BMA530_GEN_INT_SLOPE_THRES_MSK           UINT16_C(0x0FFF)
#define BMA530_GEN_INT_SLOPE_THRES_POS           UINT8_C(0)

/*! Logical evaluation condition between enabled axis status */
#define BMA530_GEN_INT_COMB_SEL_MSK              UINT16_C(0x1000)
#define BMA530_GEN_INT_COMB_SEL_POS              UINT8_C(12)

/*! Enabling of axis for generic interrupt detection */
#define BMA530_GEN_INT_AXIS_SEL_MSK              UINT16_C(0xE000)
#define BMA530_GEN_INT_AXIS_SEL_POS              UINT8_C(13)

/*! Hysteresis for the slope of the acceleration signal */
#define BMA530_GEN_INT_HYST_MSK                  UINT16_C(0x03FF)
#define BMA530_GEN_INT_HYST_POS                  UINT8_C(0)

/*! Logical evaluation condition between enabled axis status */
#define BMA530_GEN_INT_CRIT_SEL_MSK              UINT16_C(0x0400)
#define BMA530_GEN_INT_CRIT_SEL_POS              UINT8_C(10)

/*! Mode of the acceleration reference update */
#define BMA530_GEN_INT_ACC_REF_UP_MSK            UINT16_C(0x1800)
#define BMA530_GEN_INT_ACC_REF_UP_POS            UINT8_C(11)

/*! Minimum duration for which the selected criterion is true for interrupt detection. */
#define BMA530_GEN_INT_DURATION_MSK              UINT16_C(0x1FFF)
#define BMA530_GEN_INT_DURATION_POS              UINT8_C(0)

/*! Wait time for clearing the event after condition evaluates false */
#define BMA530_GEN_INT_WAIT_TIME_MSK             UINT16_C(0xE000)
#define BMA530_GEN_INT_WAIT_TIME_POS             UINT8_C(13)

/*! Quiet time after an interrupt where no additional interrupts are detected */
#define BMA530_GEN_INT_QUIET_TIME_MSK            UINT16_C(0x1FFF)
#define BMA530_GEN_INT_QUIET_TIME_POS            UINT8_C(0)

/*! Reference acceleration signal for x-axis */
#define BMA530_GEN_INT_REF_ACC_X_MSK             UINT16_C(0xFFFF)
#define BMA530_GEN_INT_REF_ACC_X_POS             UINT8_C(0)

/*! Reference acceleration signal for y-axis */
#define BMA530_GEN_INT_REF_ACC_Y_MSK             UINT16_C(0xFFFF)
#define BMA530_GEN_INT_REF_ACC_Y_POS             UINT8_C(0)

/*! Reference acceleration signal for z-axis */
#define BMA530_GEN_INT_REF_ACC_Z_MSK             UINT16_C(0xFFFF)
#define BMA530_GEN_INT_REF_ACC_Z_POS             UINT8_C(0)

/*! Accel foc configuration macros */
#define BMA530_ACC_FOC_OFF_X_MSK                 UINT16_C(0x01FF)
#define BMA530_ACC_FOC_OFF_X_POS                 UINT8_C(0)

#define BMA530_ACC_FOC_OFF_Y_MSK                 UINT16_C(0x01FF)
#define BMA530_ACC_FOC_OFF_Y_POS                 UINT8_C(0)

#define BMA530_ACC_FOC_OFF_Z_MSK                 UINT16_C(0x01FF)
#define BMA530_ACC_FOC_OFF_Z_POS                 UINT8_C(0)

#define BMA530_ACC_FOC_APPLY_CORR_MSK            UINT16_C(0x0001)
#define BMA530_ACC_FOC_APPLY_CORR_POS            UINT8_C(0)

#define BMA530_ACC_FOC_FILTER_COEFF_MSK          UINT16_C(0x000E)
#define BMA530_ACC_FOC_FILTER_COEFF_POS          UINT8_C(1)

#define BMA530_ACC_FOC_AXIS_1G_MSK               UINT16_C(0x0070)
#define BMA530_ACC_FOC_AXIS_1G_POS               UINT8_C(4)

/******************************************************************************/
/********************* Macros for supported field values **********************/
/******************************************************************************/

#define BMA530_ORIENT_MODE_SYMM                  UINT8_C(0x00)
#define BMA530_ORIENT_MODE_HIGH_ASYMM            UINT8_C(0x01)
#define BMA530_ORIENT_MODE_LOW_ASYMM             UINT8_C(0x02)

#define BMA530_ORIENT_BLOCKING_MODE_0            UINT8_C(0x00)
#define BMA530_ORIENT_BLOCKING_MODE_1            UINT8_C(0x01)
#define BMA530_ORIENT_BLOCKING_MODE_2            UINT8_C(0x02)
#define BMA530_ORIENT_BLOCKING_MODE_3            UINT8_C(0x03)

#define BMA530_GEN_INT_COMB_SEL_LOGICAL_OR       UINT8_C(0x00)
#define BMA530_GEN_INT_COMB_SEL_LOGICAL_AND      UINT8_C(0x01)

#define BMA530_GEN_INT_CRI_SEL_INACT             UINT8_C(0x00)
#define BMA530_GEN_INT_CRI_SEL_ACT               UINT8_C(0x01)

#define BMA530_GEN_INT_ACC_REF_UP_ON_EVENT       UINT8_C(0x00)
#define BMA530_GEN_INT_ACC_REF_UP_ON_ALWAYS      UINT8_C(0x01)
#define BMA530_GEN_INT_ACC_REF_UP_ON_MANUAL      UINT8_C(0x02)

#define BMA530_FEAT_AXIS_EX_DEFAULT_0            UINT8_C(0x00)
#define BMA530_FEAT_AXIS_EX_YXZ                  UINT8_C(0x01)
#define BMA530_FEAT_AXIS_EX_XZY                  UINT8_C(0x02)
#define BMA530_FEAT_AXIS_EX_ZXY                  UINT8_C(0x03)
#define BMA530_FEAT_AXIS_EX_YZX                  UINT8_C(0x04)
#define BMA530_FEAT_AXIS_EX_ZYX                  UINT8_C(0x05)
#define BMA530_FEAT_AXIS_EX_DEFAULT_6            UINT8_C(0x06)
#define BMA530_FEAT_AXIS_EX_DEFAULT_7            UINT8_C(0x07)

#define BMA530_FEAT_X_INV_DEFAULT                UINT8_C(0)
#define BMA530_FEAT_X_INV_INVERT                 UINT8_C(1)

#define BMA530_FEAT_Y_INV_DEFAULT                UINT8_C(0)
#define BMA530_FEAT_Y_INV_INVERT                 UINT8_C(1)

#define BMA530_FEAT_Z_INV_DEFAULT                UINT8_C(0)
#define BMA530_FEAT_Z_INV_INVERT                 UINT8_C(1)

/*! Feature interrupts base address definitions */
#define BMA530_BASE_ADDR_FEAT_CONF_ERR           UINT8_C(0x02)
#define BMA530_BASE_ADDR_GENERAL_SETTINGS        UINT8_C(0x03)
#define BMA530_BASE_ADDR_GENERIC_INT1            UINT8_C(0x04)
#define BMA530_BASE_ADDR_GENERIC_INT2            UINT8_C(0x0B)
#define BMA530_BASE_ADDR_GENERIC_INT3            UINT8_C(0x12)
#define BMA530_BASE_ADDR_STEP_COUNTER            UINT8_C(0x19)
#define BMA530_BASE_ADDR_SIG_MOTION              UINT8_C(0x2B)
#define BMA530_BASE_ADDR_TILT                    UINT8_C(0x2E)
#define BMA530_BASE_ADDR_ORIENTATION             UINT8_C(0x30)
#define BMA530_BASE_ADDR_ACC_FOC                 UINT8_C(0x32)

#define BMA530_AXIS_SEL_DEFAULT                  UINT8_C(0x07)
#define BMA530_DURATION_DEFAULT                  UINT8_C(0x0A)
#define BMA530_WAIT_TIME_DEFAULT                 UINT8_C(0x03)
#define BMA530_QUIET_TIME_DEFAULT                UINT8_C(0x40)
#define BMA530_ACC_REF_X_DEFAULT                 UINT8_C(0x00)
#define BMA530_ACC_REF_Y_DEFAULT                 UINT8_C(0x00)
#define BMA530_ACC_REF_Z_DEFAULT                 UINT8_C(0x800)

/*! Error status of accel config for features */
#define BMA530_GEN_INT1_CONF_ERR_OKAY            UINT8_C(0)
#define BMA530_GEN_INT1_CONF_ERR_ERR             UINT8_C(1)
#define BMA530_GEN_INT2_CONF_ERR_OKAY            UINT8_C(0)
#define BMA530_GEN_INT2_CONF_ERR_ERR             UINT8_C(1)
#define BMA530_GEN_INT3_CONF_ERR_OKAY            UINT8_C(0)
#define BMA530_GEN_INT3_CONF_ERR_ERR             UINT8_C(1)
#define BMA530_STEP_CONF_ERR_OKAY                UINT8_C(0)
#define BMA530_STEP_CONF_ERR_ERR                 UINT8_C(1)
#define BMA530_SIG_MO_CONF_ERR_OKAY              UINT8_C(0)
#define BMA530_SIG_MO_CONF_ERR_ERR               UINT8_C(1)
#define BMA530_TILT_CONF_ERR_OKAY                UINT8_C(0)
#define BMA530_TILT_CONF_ERR_ERR                 UINT8_C(1)
#define BMA530_ORIENT_CONF_ERR_OKAY              UINT8_C(0)
#define BMA530_ORIENT_CONF_ERR_ERR               UINT8_C(1)
#define BMA530_ACC_FOC_CONF_ERR_OKAY             UINT8_C(0)
#define BMA530_ACC_FOC_CONF_ERR_ERR              UINT8_C(1)

/*! Accel foc axis 1G macros */
#define BMA530_ACC_FOC_AXIS_Z_PLUS_1G            UINT8_C(0)
#define BMA530_ACC_FOC_AXIS_Z_MINUS_1G           UINT8_C(1)
#define BMA530_ACC_FOC_AXIS_Y_PLUS_1G            UINT8_C(2)
#define BMA530_ACC_FOC_AXIS_Y_MINUS_1G           UINT8_C(3)
#define BMA530_ACC_FOC_AXIS_X_PLUS_1G            UINT8_C(4)
#define BMA530_ACC_FOC_AXIS_X_MINUS_1G           UINT8_C(5)
#define BMA530_AXIS_SEL_X                        UINT8_C(0x01)
#define BMA530_AXIS_SEL_Y                        UINT8_C(0x02)
#define BMA530_AXIS_SEL_Z                        UINT8_C(0x04)
#define BMA530_AXIS_SEL_XYZ                      UINT8_C(0x07)

/*! Feature axis Exchange macros */
#define BMA530_FEAT_AXIS_EX_SEL_X                UINT8_C(0x01)
#define BMA530_FEAT_AXIS_EX_SEL_Y                UINT8_C(0x02)
#define BMA530_FEAT_AXIS_EX_SEL_Z                UINT8_C(0x04)
#define BMA530_FEAT_AXIS_EX_SEL_XYZ              UINT8_C(0x07)

/*! Soft-reset delay is 2ms */
#define BMA530_SOFT_RESET_DELAY                  UINT16_C(2000)

#define BMA530_GEN_INT_1                         UINT8_C(0)
#define BMA530_GEN_INT_2                         UINT8_C(1)
#define BMA530_GEN_INT_3                         UINT8_C(2)

/******************************************************************************/
/***************** Structures for handling register content *******************/
/******************************************************************************/

struct bma530_feat_conf_err
{
    /*! Internal filter cannot produce enough samples for generic interrupt 1 feature */
    uint8_t gen_int1_conf_err;

    /*! Internal filter cannot produce enough samples for generic interrupt 2 feature */
    uint8_t gen_int2_conf_err;

    /*! Internal filter cannot produce enough samples for generic interrupt 3 feature */
    uint8_t gen_int3_conf_err;

    /*!Internal filter cannot produce enough samples for step counter and/or step detection features*/
    uint8_t step_conf_err;

    /*! Internal filter cannot produce enough samples for significant motion detection feature.*/
    uint8_t sig_mo_conf_err;

    /*!Internal filter cannot produce enough samples for tilt detection feature.*/
    uint8_t tilt_conf_err;

    /*! Internal filter cannot produce enough samples for orientation detection feature*/
    uint8_t orient_conf_err;

    /*! Internal filter cannot produce enough samples for accelerometer fast-offset compensation feature */
    uint8_t acc_foc_conf_err;
};

/*!
 *  @brief Structure to store feature axis config
 */
struct bma530_feat_axis
{
    /*! Axes exchange scheme that is applied in host software  */
    uint8_t feat_axis_ex;

    /*! Invert polarity of X-axis data after axis exchange */
    uint8_t feat_x_inv;

    /*! Invert polarity of Y-axis data after axis exchange */
    uint8_t feat_y_inv;

    /*! Invert polarity of Z-axis data after axis exchange */
    uint8_t feat_z_inv;
};

/*!
 *  @brief Structure to store generic interrupt config
 */
struct bma530_generic_interrupt
{
    /*! Minimum/maximum slope of acceleration signal for interrupt detection based on selected motion criterion. */
    uint16_t slope_thres;

    /*! Logical evaluation condition between enabled axis status
     * BMA530_GEN_INT_COMB_SEL_LOGICAL_OR   - 0x00
     * BMA530_GEN_INT_COMB_SEL_LOGICAL_AND  - 0x01
     */
    uint8_t comb_sel;

    /*! Enabling of axis for generic interrupt detection */
    uint8_t axis_sel;

    /*! Hysteresis for the slope of the acceleration signal */
    uint16_t hysteresis;

    /*! Logical evaluation condition between enabled axis status
     * BMA530_GEN_INT_CRI_SEL_INACT - 0x00
     * BMA530_GEN_INT_CRI_SEL_ACT   - 0x01
     */
    uint8_t criterion_sel;

    /*! Mode of the acceleration reference update
     * BMA530_GEN_INT_ACC_REF_UP_ON_EVENT   -  0x00
     * BMA530_GEN_INT_ACC_REF_UP_ON_ALWAYS  -  0x01
     * BMA530_GEN_INT_ACC_REF_UP_ON_MANUAL  -  0x02
     */
    uint8_t acc_ref_up;

    /*! Minimum duration for which the selected criterion is true for interrupt detection. */
    uint16_t duration;

    /*! Wait time for clearing the event after condition evaluates false */
    uint8_t wait_time;

    /*! Quiet time after an interrupt where no additional interrupts are detected */
    uint16_t quiet_time;

    /*! Reference acceleration signal for x-axis */
    int16_t ref_acc_x;

    /*! Reference acceleration signal for y-axis */
    int16_t ref_acc_y;

    /*! Reference acceleration signal for z-axis */
    int16_t ref_acc_z;
};

/*!
 *  @brief Structure to holding generic interrupt configuration.
 */
struct bma530_generic_interrupt_types
{
    /*! Specify generic interrupt 1, 2, or 3 */
    uint8_t generic_interrupt;

    /*! Generic Interrupt Configuration */
    struct bma530_generic_interrupt gen_int;
};

/*!
 *  @brief Structure to store accel FOC config
 */
struct bma530_accel_foc_config
{
    /*! Accel foc offset x-axis */
    uint16_t foc_off_x;

    /*! Accel foc offset y-axis */
    uint16_t foc_off_y;

    /*! Accel foc offset z-axis */
    uint16_t foc_off_z;

    /*! Accel foc correlation */
    uint8_t foc_apply_corr;

    /*! Accel foc filter coefficient */
    uint8_t foc_filter_coeff;

    /*! Accel foc axis for 1G
     * BMA530_ACC_FOC_AXIS_Z_PLUS_1G   -   0x00
     * BMA530_ACC_FOC_AXIS_Z_MINUS_1G   -   0x01
     * BMA530_ACC_FOC_AXIS_Y_PLUS_1G   -   0x02
     * BMA530_ACC_FOC_AXIS_Y_MINUS_1G   -   0x03
     * BMA530_ACC_FOC_AXIS_X_PLUS_1G   -   0x04
     * BMA530_ACC_FOC_AXIS_X_MINUS_1G   -   0x05
     */
    uint8_t foc_axis_1g;
};

/*!
 *  @brief Structure to store step counter config
 */
struct bma530_step_cntr
{
    /*! An interrupt will be triggered every time the difference in number of
       * steps counted from last event is equal to (set value * 20). If 0, the interrupt is disabled */
    uint16_t watermark_level;

    /*! Reset the accumulated step count value */
    uint8_t reset_counter;

    /*! Enable step detector */
    uint8_t sd_en;

    /*! Enable step counter */
    uint8_t sc_en;

    /*! Threshold for upper peak of acceleration magnitude for step detection */
    uint16_t envelope_up_thres;

    /*! Adaptive upper peak threshold decay coefficient */
    uint16_t envelope_up_decay_coeff;

    /*! Threshold for lower peak of acceleration magnitude for step detection */
    uint16_t envelope_down_thres;

    /*! Adaptive lower peak threshold decay coefficient */
    uint16_t envelope_down_decay_coeff;

    /*! Exponential smoothing filter coefficient for computing mean of acceleration magnitude */
    uint16_t acc_mean_decay_coeff;

    /*! Exponential smoothing filter coefficient for computing mean duration between steps */
    uint16_t step_dur_mean_decay_coeff;

    /*! Minimum number of consecutive steps to be detected for updating step count */
    uint8_t step_buffer_size;

    /*! Enable or disable cascading of filters */
    uint8_t filter_cascade_enabled;

    /*! Scale factor for the step count to handle overcounting to undercounting */
    uint16_t step_counter_increment;

    /*! Enable or disable detection of half step */
    uint8_t en_half_step;

    /*! Minimum duration between two consecutive steps while walking */
    uint8_t peak_duration_min_walking;

    /*! Minimum duration between two consecutive steps while running */
    uint8_t peak_duration_min_running;

    /*! Ratio of acceleration magnitude variance during running to walking */
    uint8_t activity_detection_factor;

    /*! Acceleration magnitude variance threshold for activity classification */
    uint16_t activity_detection_thres;

    /*! Maximum duration between two consecutive step occurance */
    uint8_t step_duration_max;

    /*! Maximum duration since last step where next step shall be detected to add missed step, if any */
    uint8_t step_duration_window;

    /*! Enable or disable post-processing for duration between steps */
    uint8_t en_step_dur_pp;

    /*! Scale factor for mean step duration for step processing */
    uint8_t step_dur_thres;

    /*! Enable or disable post-processing of step based on mean crossing */
    uint8_t en_mcr_pp;

    /*! Threshold for number of mean crossings between two consecutive steps */
    uint16_t mcr_thres;

    /*! Filter coefficient B2 of 2nd order IIR filter */
    uint16_t filter_coeff_b_2;

    /*! Filter coefficient B1 of 2nd order IIR filter */
    uint16_t filter_coeff_b_1;

    /*! Filter coefficient B0 of 2nd order IIR filter */
    uint16_t filter_coeff_b_0;

    /*! Filter coefficient A2 of 2nd order IIR filter */
    uint16_t filter_coeff_a_2;

    /*! Filter coefficient A1 of 2nd order IIR filter */
    uint16_t filter_coeff_a_1;

    /*! Scaling of filter coefficients A of 2nd order IIR filter */
    uint8_t filter_coeff_scale_a;

    /*! Scaling of filter coefficients B of 2nd order IIR filter */
    uint8_t filter_coeff_scale_b;
};

/*!
 *  @brief Structure to store sig motion config
 */
struct bma530_sig_motion
{
    /*! Size of the segment for detection of significant motion of the device */
    uint16_t block_size;

    /*! Minimum value of the peak to peak acceleration magnitude */
    uint16_t p2p_min;

    /*! Minimum number of mean crossing per second in acceleration magnitude */
    uint8_t mcr_min;

    /*! Maximum value of the peak to peak acceleration magnitude */
    uint16_t p2p_max;

    /*! Maximum number of mean crossing per second in acceleration magnitude */
    uint8_t mcr_max;
};

/*!
 *  @brief Structure to store tilt config
 */
struct bma530_tilt
{
    /*! Duration for which the acceleration vector is averaged to be reference vector */
    uint8_t segment_size;

    /*! Minimum angle by which the device shall be tilted for event detection */
    uint8_t min_tilt_angle;

    /*! Exponential smoothing coefficient for computing low-pass mean of acceleration vector */
    uint16_t beta_acc_mean;
};

/*!
 *  @brief Structure to store orientation config
 */
struct bma530_orient
{
    /*! Selection of upside down orientation detection
     * BMA530_ENABLE  -  0x01
     * BMA530_DISABLE -  0x00
     */
    uint8_t ud_en;

    /*! Selection of mode for orientation spread in the detection plane
     * BMA530_ORIENT_MODE_SYMM       - 0x00
     * BMA530_ORIENT_MODE_HIGH_ASYMM - 0x01
     * BMA530_ORIENT_MODE_LOW_ASYMM  - 0x02
     */
    uint8_t mode;

    /*! Blocking allows to prevent change of orientation during large movement of device
     * BMA530_ORIENT_BLOCKING_MODE_0 - 0x00
     * BMA530_ORIENT_BLOCKING_MODE_1 - 0x01
     * BMA530_ORIENT_BLOCKING_MODE_2 - 0x02
     * BMA530_ORIENT_BLOCKING_MODE_3 - 0x03
     */
    uint8_t blocking;

    /*! Maximum allowed tilt angle for device to be in flat state */
    uint8_t theta;

    /*! Minimum duration the device shall be in new orientation for change detection */
    uint8_t hold_time;

    /*! Minimum slope between consecutive acceleration samples to pervent the change of orientation during large
     * movement */
    uint8_t slope_thres;

    /*! Hysteresis of acceleration for orientation change detection */
    uint8_t hysteresis;
};

/*!
 * @brief Structure holds: feature engine general purpose register feature output
 */
struct bma530_feat_eng_feat_out
{
    /*! Stores step counter output */
    uint32_t step_cntr_out;

    /*! Output of orientation detection feature. Device orientation can be either portrait or landscape. Value after
     * device initialization is 0b00 i.e. portrait up.
     */
    uint8_t orientation_portrait_landscape;

    /*! Output of orientation detection feature. Output is only valid if "ud_en" is enabled. Device orientation can be
     * either face up or face down. Value after device initialization is 0b0 i.e. face up.
     */
    uint8_t orientation_face_up_down;

    /*! Status of user activity reported by step counter */
    uint8_t activ_stat;

    /*! Status of generic interrupt 1 motion detection */
    uint8_t gen_int1_stat;

    /*! Status of generic interrupt 2 motion detection */
    uint8_t gen_int2_stat;

    /*! Status of generic interrupt 3 motion detection */
    uint8_t gen_int3_stat;

    /*! Feature engine initialization status
        Value   Name    Description
        00  init_not_ok Feature engine is not initialized
        01  init_ok Feature engine is initialized
     */
    uint8_t feat_init_stat;

    /*! Feature engine error related to accelerometer configuration.
         Bit is set to 1 when acc_perf_mode=0 and acc_odr is less than sample rate required by features.
         Bit is cleared to 0 when acc_perf_mode=1 or acc_odr is set to minimum sample rate required by features. */
    uint8_t feat_conf_err;

    /*! Bit is set to '1' if fast-offset compensation feature is being executed. Bit is cleared to '0' at the end of
     * feature compensation. User should not change the accelerometer configuration while the feature is running */
    uint8_t foc_running;
};

/******************************************************************************/
/********************** Function prototype declarations ***********************/
/******************************************************************************/

/**
 * \ingroup bma530
 * \defgroup bma530ApiInit BMA530 Initialization
 * @brief Initialize the sensor and device structure
 */

/*!
 * \ingroup bma530ApiInit
 * \page bma530_api_bma530_init bma530_init
 * \code
 * int8_t bma530_init(struct bma5_dev *dev);
 * \endcode
 * @details This API reads the chip-id of the sensor which is the first step to
 * verify the sensor and also it configures the read mechanism of SPI and
 * I2C interface. As this API is the entry point, call this API before using other APIs.
 *
 * @param[in,out] dev : Structure instance of bma5_dev
 *
 * @return Result of API execution status
 *
 * @retval Zero     -> Success
 * @retval Positive -> Warning
 * @retval Negative -> Error/Failure
 */
int8_t bma530_init(struct bma5_dev *dev);

/**
 * \ingroup bma530
 * \defgroup bma530FeatureEngineApiRegs BMA530 Feature Engine APIs
 * @brief Feature Engine APIs
 */

/*!
 * \ingroup bma530FeatureEngineApiRegs
 * \page bma530_api_bma530_get_feat_eng_feature_out bma530_get_feat_eng_feature_out
 * \code
 * int8_t bma530_get_feat_eng_feature_out(struct bma530_feat_eng_feat_out *feat_out, struct bma5_dev *dev);
 * \endcode
 * @details This API gets feature output status from the register.
 *
 * @param[out] feat_out   : Structure instance of bma530_feat_eng_feat_out.
 * @param[in,out] dev        : Structure instance of bma5_dev.
 *
 *  @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma530_get_feat_eng_feature_out(struct bma530_feat_eng_feat_out *feat_out, struct bma5_dev *dev);

/**
 * \ingroup bma530
 * \defgroup bma530FeatApiRegs BMA530 Feature configuration registers
 * @brief Set / Get data from the given Feature configuration register address of the sensor
 */

/*!
 * \ingroup bma530FeatApiRegs
 * \page bma530_api_bma530_set_android_comp_mode bma530_set_android_comp_mode
 * \code
 * int8_t bma530_set_android_comp_mode(const uint8_t *android_comp, struct bma5_dev *dev);
 * \endcode
 * @details This API sets android compatibility mode
 *
 * @param[in] comp_data      : Variable holds android compatibility mode data
 * @param[in] dev            : Structure instance of bma5_dev.
 *
 *  @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */

int8_t bma530_set_android_comp_mode(const uint8_t *android_comp, struct bma5_dev *dev);

/*!
 * \ingroup bma530FeatApiRegs
 * \page bma530_api_bma530_set_feat_conf_err bma530_set_feat_conf_err
 * \code
 * int8_t bma530_set_feat_conf_err(const struct bma530_feat_conf_err *feat_conf_err, struct bma5_dev *dev)
 * \endcode
 * @details This API sets Bits which reflects the error status of accel config for features.
 *
 * @param[in] feat_conf_err      : Structure instance of bma530_feat_conf_err.
 * @param[in] dev                : Structure instance of bma5_dev.
 *
 *  @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma530_set_feat_conf_err(const struct bma530_feat_conf_err *feat_conf_err, struct bma5_dev *dev);

/*!
 * \ingroup bma530FeatApiRegs
 * \page bma530_api_bma530_get_feat_conf_err bma530_get_feat_conf_err
 * \code
 * int8_t bma530_get_feat_conf_err(const struct bma530_feat_conf_err *feat_conf_err, struct bma5_dev *dev)
 * \endcode
 * @details This API gets Bits which reflects the error status of accel config for features.
 *
 * @param[in] feat_conf_err      : Structure instance of bma530_feat_conf_err.
 * @param[in] dev                : Structure instance of bma5_dev.
 *
 *  @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma530_get_feat_conf_err(struct bma530_feat_conf_err *feat_conf_err, struct bma5_dev *dev);

/*!
 * \ingroup bma530FeatApiRegs
 * \page bma530_api_bma530_get_android_comp_mode bma530_get_android_comp_mode
 * \code
 * int8_t bma530_get_android_comp_mode(uint8_t *android_comp, struct bma5_dev *dev);
 * \endcode
 * @details This API gets android compatibility mode data
 *
 * @param[out] android_comp      : Variable holds android compatibility mode data
 * @param[in,out] dev                : Structure instance of bma5_dev.
 *
 *  @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma530_get_android_comp_mode(uint8_t *android_comp, struct bma5_dev *dev);

/*!
 * \ingroup bma530FeatApiRegs
 * \page bma530_api_bma530_set_step_counter_config bma530_set_step_counter_config
 * \code
 * int8_t bma530_set_step_counter_config(const struct bma530_step_cntr *step_cntr, struct bma5_dev *dev);
 * \endcode
 * @details This API sets step counter configuration
 *
 * @param[in] step_cntr      : Structure instance of bma530_step_cntr.
 * @param[in] dev            : Structure instance of bma5_dev.
 *
 *  @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma530_set_step_counter_config(const struct bma530_step_cntr *step_cntr, struct bma5_dev *dev);

/*!
 * \ingroup bma530FeatApiRegs
 * \page bma530_api_bma530_get_step_counter_config bma530_get_step_counter_config
 * \code
 * int8_t bma530_get_step_counter_config(struct bma530_step_cntr *step_cntr, struct bma5_dev *dev);
 * \endcode
 * @details This API gets step counter configuration
 *
 * @param[out] step_cntr      : Structure instance of bma530_step_cntr.
 * @param[in,out] dev            : Structure instance of bma5_dev.
 *
 *  @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma530_get_step_counter_config(struct bma530_step_cntr *step_cntr, struct bma5_dev *dev);

/*!
 * \ingroup bma530FeatApiRegs
 * \page bma530_api_bma530_get_default_step_counter_config bma530_get_step_counter_config
 * \code
 * int8_t bma530_get_default_step_counter_config(struct bma530_step_cntr *step_cntr, struct bma5_dev *dev);
 * \endcode
 * @details This API gets step counter configuration
 *
 * @param[out] step_cntr      : Structure instance of bma530_step_cntr.
 * @param[in,out] dev         : Structure instance of bma5_dev.
 *
 *  @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma530_get_default_step_counter_config(struct bma530_step_cntr *step_cntr, struct bma5_dev *dev);

/*!
 * \ingroup bma530FeatApiRegs
 * \page bma530_api_bma530_set_sig_motion_config bma530_set_sig_motion_config
 * \code
 * int8_t bma530_set_sig_motion_config(const struct bma530_sig_motion *sig_mot, struct bma5_dev *dev);
 * \endcode
 * @details This API sets sig-motion configuration
 *
 * @param[in] sig_mot        : Structure instance of bma530_sig_motion.
 * @param[in] dev            : Structure instance of bma5_dev.
 *
 *  @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma530_set_sig_motion_config(const struct bma530_sig_motion *sig_mot, struct bma5_dev *dev);

/*!
 * \ingroup bma530FeatApiRegs
 * \page bma530_api_bma530_get_sig_motion_config bma530_get_sig_motion_config
 * \code
 * int8_t bma530_get_sig_motion_config(struct bma530_sig_motion *sig_mot, struct bma5_dev *dev);
 * \endcode
 * @details This API gets sig-motion configuration
 *
 * @param[out] sig_mot        : Structure instance of bma530_sig_motion.
 * @param[in,out] dev            : Structure instance of bma5_dev.
 *
 *  @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma530_get_sig_motion_config(struct bma530_sig_motion *sig_mot, struct bma5_dev *dev);

/*!
 * \ingroup bma530FeatApiRegs
 * \page bma530_api_bma530_get_default_sig_motion_config bma530_get_sig_motion_config
 * \code
 * int8_t bma530_get_default_sig_motion_config(struct bma530_sig_motion *sig_mot, struct bma5_dev *dev);
 * \endcode
 * @details This API gets sig-motion configuration
 *
 * @param[out] sig_mot        : Structure instance of bma530_sig_motion.
 * @param[in,out] dev            : Structure instance of bma5_dev.
 *
 *  @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma530_get_default_sig_motion_config(struct bma530_sig_motion *sig_mot, struct bma5_dev *dev);

/*!
 * \ingroup bma530FeatApiRegs
 * \page bma530_api_bma530_set_tilt_config bma530_set_tilt_config
 * \code
 * int8_t bma530_set_tilt_config(const struct bma530_tilt *tilt, struct bma5_dev *dev);
 * \endcode
 * @details This API sets tilt configuration
 *
 * @param[in] tilt           : Structure instance of bma530_tilt.
 * @param[in] dev            : Structure instance of bma5_dev.
 *
 *  @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma530_set_tilt_config(const struct bma530_tilt *tilt, struct bma5_dev *dev);

/*!
 * \ingroup bma530FeatApiRegs
 * \page bma530_api_bma530_get_tilt_config bma530_get_tilt_config
 * \code
 * int8_t bma530_get_tilt_config(struct bma530_tilt *tilt, struct bma5_dev *dev);
 * \endcode
 * @details This API gets tilt configuration
 *
 * @param[out] tilt           : Structure instance of bma530_tilt.
 * @param[in,out] dev            : Structure instance of bma5_dev.
 *
 *  @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma530_get_tilt_config(struct bma530_tilt *tilt, struct bma5_dev *dev);

/*!
 * \ingroup bma530FeatApiRegs
 * \page bma530_api_bma530_set_orient_config bma530_set_orient_config
 * \code
 * int8_t bma530_set_orient_config(const struct bma530_orient *orient, struct bma5_dev *dev);
 * \endcode
 * @details This API sets orientation configuration
 *
 * @param[in] orient         : Structure instance of bma530_orient.
 * @param[in] dev            : Structure instance of bma5_dev.
 *
 *  @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma530_set_orient_config(const struct bma530_orient *orient, struct bma5_dev *dev);

/*!
 * \ingroup bma530FeatApiRegs
 * \page bma530_api_bma530_get_orient_config bma530_get_orient_config
 * \code
 * int8_t bma530_get_orient_config(struct bma530_orient *orient, struct bma5_dev *dev);
 * \endcode
 * @details This API gets orientation configuration
 *
 * @param[out] orient         : Structure instance of bma530_orient
 * @param[in,out] dev            : Structure instance of bma5_dev.
 *
 *  @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma530_get_orient_config(struct bma530_orient *orient, struct bma5_dev *dev);

/*!
 * \ingroup bma530FeatApiRegs
 * \page bma530_api_bma530_get_default_generic_int_config bma530_get_default_generic_int_config
 * \code
 * int8_t bma530_get_default_generic_int_config(struct bma530_generic_interrupt_types *gen_int, uint8_t n_ints, struct bma5_dev *dev);
 * \endcode
 * @details This API gets default values generic interrupt configurations.
 *
 * @param[out] gen_int        : Structure instance of bma530_generic_interrupt.
 * @param[in]  n_ints         : Number of default generic interrupt configurations to get.
 * @param[in,out] dev         : Structure instance of bma5_dev.
 *
 *  @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma530_get_default_generic_int_config(struct bma530_generic_interrupt_types *gen_int,
                                             uint8_t n_ints,
                                             struct bma5_dev *dev);

/*!
 * \ingroup bma530FeatApiRegs
 * \page bma530_api_bma530_set_generic_int_config bma530_set_generic_int_config
 * \code
 * int8_t bma530_set_generic_int_config(const struct bma530_generic_interrupt *gen_int, uint8_t n_ints, struct bma5_dev *dev);
 * \endcode
 * @details This API sets generic interrupt configurations.
 *
 * @param[in] gen_int        : Structure instance of bma530_generic_interrupt.
 * @param[in] n_ints         : Number of default generic interrupt configurations to set.
 * @param[in] dev            : Structure instance of bma5_dev.
 *
 *  @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma530_set_generic_int_config(const struct bma530_generic_interrupt_types *gen_int,
                                     uint8_t n_ints,
                                     struct bma5_dev *dev);

/*!
 * \ingroup bma530FeatApiRegs
 * \page bma530_api_bma530_get_generic_int_config bma530_get_generic_int_config
 * \code
 * int8_t bma530_get_generic_int_config(struct bma530_generic_interrupt_types *gen_int, uint8_t n_ints, struct bma5_dev *dev);
 * \endcode
 * @details This API gets generic interrupt configurations.
 *
 * @param[out] gen_int        : Structure instance of bma530_generic_interrupt.
 * @param[in]  n_ints         : Number of default generic interrupt configurations to get.
 * @param[in,out] dev         : Structure instance of bma5_dev.
 *
 *  @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma530_get_generic_int_config(struct bma530_generic_interrupt_types *gen_int,
                                     uint8_t n_ints,
                                     struct bma5_dev *dev);

/*!
 * \ingroup bma530FeatApiRegs
 * \page bma530_api_bma530_set_accel_foc_config bma530_set_accel_foc_config
 * \code
 * int8_t bma530_set_accel_foc_config(const struct bma530_accel_foc_config *acc_foc, struct bma5_dev *dev);
 * \endcode
 * @details This API sets accel foc configuration
 *
 * @param[in] acc_foc        : Structure instance of bma530_accel_foc_config.
 * @param[in] dev            : Structure instance of bma5_dev.
 *
 *  @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma530_set_accel_foc_config(const struct bma530_accel_foc_config *acc_foc, struct bma5_dev *dev);

/*!
 * \ingroup bma530FeatApiRegs
 * \page bma530_api_bma530_get_accel_foc_config bma530_get_accel_foc_config
 * \code
 * int8_t bma530_get_accel_foc_config(struct bma530_accel_foc_config *acc_foc, struct bma5_dev *dev);
 * \endcode
 * @details This API gets accel foc configuration
 *
 * @param[out] acc_foc        : Structure instance of bma530_accel_foc_config.
 * @param[in,out] dev            : Structure instance of bma5_dev.
 *
 *  @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma530_get_accel_foc_config(struct bma530_accel_foc_config *acc_foc, struct bma5_dev *dev);

/*!
 * \ingroup bma530FeatApiRegs
 * \page bma530_api_bma530_set_feature_axis_config bma530_set_feature_axis_config
 * \code
 * int8_t bma530_set_feature_axis_config(const struct bma530_feat_axis *feat_axis, struct bma5_dev *dev);
 * \endcode
 * @details This API sets feature axis configurations.
 *
 * @param[in] feat_axis      : Structure instance of bma530_feat_axis.
 * @param[in] dev            : Structure instance of bma5_dev.
 *
 *  @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma530_set_feature_axis_config(const struct bma530_feat_axis *feat_axis, struct bma5_dev *dev);

/*!
 * \ingroup bma530FeatApiRegs
 * \page bma530_api_bma530_get_feature_axis_config bma530_get_feature_axis_config
 * \code
 * int8_t bma530_get_feature_axis_config(struct bma530_feat_axis *feat_axis, struct bma5_dev *dev);
 * \endcode
 * @details This API gets feature axis configurations.
 *
 * @param[out] feat_axis      : Structure instance of bma530_feat_axis.
 * @param[in,out] dev         : Structure instance of bma5_dev.
 *
 *  @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma530_get_feature_axis_config(struct bma530_feat_axis *feat_axis, struct bma5_dev *dev);

/**
 * \ingroup bma530
 * \defgroup bma530ApiSR BMA530 Soft-reset
 * @brief Set / Get data from the given register address of the sensor
 */

/*!
 * \ingroup bma530ApiSR
 * \page bma530_api_bma530_soft_reset bma530_soft_reset
 * \code
 * int8_t bma530_soft_reset(struct bma5_dev *dev);
 * \endcode
 * @details This API resets sensor. All registers are overwritten with
 * their default values.
 *
 * @param[in] dev : Structure instance of bma5_dev.
 *
 *  @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma530_soft_reset(struct bma5_dev *dev);

#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif /* _BMA530_FEATURES_H */
