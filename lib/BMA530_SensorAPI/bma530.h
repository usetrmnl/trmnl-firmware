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
* @file       bma530.h
* @date       2024-07-29
* @version    v4.2.0
*
*/

#ifndef _BMA530_H
#define _BMA530_H

/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************/
/****************************** Header files **********************************/
/******************************************************************************/
#include  "bma5.h"

/******************************************************************************/
/************************* General macro definitions **************************/
/******************************************************************************/
/*! Chip ID of BMA530 */
#define BMA530_CHIP_ID                                         UINT8_C(0xC2)

/******************************************************************************/
/***************************** Register addresses *****************************/
/******************************************************************************/
/*! The product chip_id. Upper four bits are fix and lower four bits are boot loaded from OTP. */
#define BMA530_REG_CHIP_ID                                     UINT8_C(0x00)

/*! INT1 interrupt status register 0 */
#define BMA530_REG_INT_STATUS_INT1_0                           UINT8_C(0x12)

/*! INT1 interrupt status register 1 */
#define BMA530_REG_INT_STATUS_INT1_1                           UINT8_C(0x13)

/*! INT2 interrupt status register 0 */
#define BMA530_REG_INT_STATUS_INT2_0                           UINT8_C(0x14)

/*! INT2 interrupt status register 1 */
#define BMA530_REG_INT_STATUS_INT2_1                           UINT8_C(0x15)

/*! I3C interrupt status register 0 */
#define BMA530_REG_INT_STATUS_I3C_0                            UINT8_C(0x16)

/*! I3C interrupt status register 1 */
#define BMA530_REG_INT_STATUS_I3C_1                            UINT8_C(0x17)

/*! Interrupt mapping register 0 */
#define BMA530_REG_INT_MAP_0                                   UINT8_C(0x36)

/*! Interrupt mapping register 1 */
#define BMA530_REG_INT_MAP_1                                   UINT8_C(0x37)

/*! Interrupt mapping register 2 */
#define BMA530_REG_INT_MAP_2                                   UINT8_C(0x38)

/*! Interrupt mapping register 3 */
#define BMA530_REG_INT_MAP_3                                   UINT8_C(0x39)

/*! feature engine ('feat_eng') general purpose flags */
#define BMA530_REG_FEAT_ENG_GP_FLAGS                           UINT8_C(0x52)

/*! feature engine ('feat_eng') general purpose register 0 */
#define BMA530_REG_FEAT_ENG_GPR_0                              UINT8_C(0x55)

/*! feature engine ('feat_eng') general purpose register 1 */
#define BMA530_REG_FEAT_ENG_GPR_1                              UINT8_C(0x56)

/*! feature engine ('feat_eng') general purpose register 2 */
#define BMA530_REG_FEAT_ENG_GPR_2                              UINT8_C(0x57)

/*! feature engine ('feat_eng') general purpose register 3 */
#define BMA530_REG_FEAT_ENG_GPR_3                              UINT8_C(0x58)

/*! feature engine ('feat_eng') general purpose register 4 */
#define BMA530_REG_FEAT_ENG_GPR_4                              UINT8_C(0x59)

/*! feature engine ('feat_eng') general purpose register 5 */
#define BMA530_REG_FEAT_ENG_GPR_5                              UINT8_C(0x5A)

/******************************************************************************/
/********************** Register macros for bit masking ***********************/
/******************************************************************************/
/*! Chip id */
#define BMA530_CHIP_ID_MSK                                     UINT8_C(0xFF)
#define BMA530_CHIP_ID_POS                                     UINT8_C(0x00)

/*|
 * @brief Mask and Position values for Interrupt Status for INT1, INT2, I3C Interrupts
 */

/*! Accelerometer data ready interrupt status */
#define BMA530_ACC_DRDY_INT_STATUS_MSK                         UINT8_C(0x01)
#define BMA530_ACC_DRDY_INT_STATUS_POS                         UINT8_C(0x00)

/*! FIFO watermark interrupt status */
#define BMA530_FIFO_WM_INT_STATUS_MSK                          UINT8_C(0x02)
#define BMA530_FIFO_WM_INT_STATUS_POS                          UINT8_C(0x01)

/*! FIFO full interrupt status */
#define BMA530_FIFO_FULL_INT_STATUS_MSK                        UINT8_C(0x04)
#define BMA530_FIFO_FULL_INT_STATUS_POS                        UINT8_C(0x02)

/*! Generic interrupt 1 interrupt status */
#define BMA530_GEN_INT1_INT_STATUS_MSK                         UINT8_C(0x08)
#define BMA530_GEN_INT1_INT_STATUS_POS                         UINT8_C(0x03)

/*! Generic interrupt 2 interrupt status */
#define BMA530_GEN_INT2_INT_STATUS_MSK                         UINT8_C(0x10)
#define BMA530_GEN_INT2_INT_STATUS_POS                         UINT8_C(0x04)

/*! Generic interrupt 3 interrupt status */
#define BMA530_GEN_INT3_INT_STATUS_MSK                         UINT8_C(0x20)
#define BMA530_GEN_INT3_INT_STATUS_POS                         UINT8_C(0x05)

/*! Step detection interrupt status */
#define BMA530_STEP_DET_INT_STATUS_MSK                         UINT8_C(0x40)
#define BMA530_STEP_DET_INT_STATUS_POS                         UINT8_C(0x06)

/*! Step counter watermark interrupt status */
#define BMA530_STEP_CNT_INT_STATUS_MSK                         UINT8_C(0x80)
#define BMA530_STEP_CNT_INT_STATUS_POS                         UINT8_C(0x07)

/*! Significant motion detection interrupt status */
#define BMA530_SIG_MO_INT_STATUS_MSK                           UINT8_C(0x01)
#define BMA530_SIG_MO_INT_STATUS_POS                           UINT8_C(0x00)

/*! Tilt detection interrupt status */
#define BMA530_TILT_INT_STATUS_MSK                             UINT8_C(0x02)
#define BMA530_TILT_INT_STATUS_POS                             UINT8_C(0x01)

/*! Orientation detection status */
#define BMA530_ORIENT_INT_STATUS_MSK                           UINT8_C(0x04)
#define BMA530_ORIENT_INT_STATUS_POS                           UINT8_C(0x02)

/*! Accelerometer FOC completion status */
#define BMA530_ACC_FOC_INT_STATUS_MSK                          UINT8_C(0x08)
#define BMA530_ACC_FOC_INT_STATUS_POS                          UINT8_C(0x03)

/*! MCU error interrupt status */
#define BMA530_FEAT_ENG_ERR_INT_STATUS_MSK                     UINT8_C(0x10)
#define BMA530_FEAT_ENG_ERR_INT_STATUS_POS                     UINT8_C(0x04)

/*! Data ready interrupt mapping */
#define BMA530_ACC_DRDY_INT_MAP_MSK                            UINT8_C(0x03)
#define BMA530_ACC_DRDY_INT_MAP_POS                            UINT8_C(0x00)

/*! FIFO watermark interrupt mapping */
#define BMA530_FIFO_WM_INT_MAP_MSK                             UINT8_C(0x0C)
#define BMA530_FIFO_WM_INT_MAP_POS                             UINT8_C(0x02)

/*! FIFO full interrupt mapping */
#define BMA530_FIFO_FULL_INT_MAP_MSK                           UINT8_C(0x30)
#define BMA530_FIFO_FULL_INT_MAP_POS                           UINT8_C(0x04)

/*! Generic interrupt 1 interrupt mapping */
#define BMA530_GEN_INT1_INT_MAP_MSK                            UINT8_C(0xC0)
#define BMA530_GEN_INT1_INT_MAP_POS                            UINT8_C(0x06)

/*! Generic interrupt 2 interrupt mapping */
#define BMA530_GEN_INT2_INT_MAP_MSK                            UINT8_C(0x03)
#define BMA530_GEN_INT2_INT_MAP_POS                            UINT8_C(0x00)

/*! Generic interrupt 3 interrupt mapping */
#define BMA530_GEN_INT3_INT_MAP_MSK                            UINT8_C(0x0C)
#define BMA530_GEN_INT3_INT_MAP_POS                            UINT8_C(0x02)

/*! Step detection interrupt mapping */
#define BMA530_STEP_DET_INT_MAP_MSK                            UINT8_C(0x30)
#define BMA530_STEP_DET_INT_MAP_POS                            UINT8_C(0x04)

/*! Step counter watermark interrupt mapping */
#define BMA530_STEP_CNT_INT_MAP_MSK                            UINT8_C(0xC0)
#define BMA530_STEP_CNT_INT_MAP_POS                            UINT8_C(0x06)

/*! Significant motion detection interrupt mapping */
#define BMA530_SIG_MO_INT_MAP_MSK                              UINT8_C(0x03)
#define BMA530_SIG_MO_INT_MAP_POS                              UINT8_C(0x00)

/*! Tilt detection interrupt mapping */
#define BMA530_TILT_INT_MAP_MSK                                UINT8_C(0x0C)
#define BMA530_TILT_INT_MAP_POS                                UINT8_C(0x02)

/*! Orientation detection interrupt mapping */
#define BMA530_ORIENT_INT_MAP_MSK                              UINT8_C(0x30)
#define BMA530_ORIENT_INT_MAP_POS                              UINT8_C(0x04)

/*! Accelerometer FOC completion interrupt mapping */
#define BMA530_ACC_FOC_INT_MAP_MSK                             UINT8_C(0xC0)
#define BMA530_ACC_FOC_INT_MAP_POS                             UINT8_C(0x06)

/*! MCU error interrupt mapping */
#define BMA530_FEAT_ENG_ERR_INT_MAP_MSK                        UINT8_C(0x03)
#define BMA530_FEAT_ENG_ERR_INT_MAP_POS                        UINT8_C(0x00)

/*!  Feature engine initialization status  */
#define BMA530_FEAT_INIT_STAT_MSK                              UINT8_C(0x03)
#define BMA530_FEAT_INIT_STAT_POS                              UINT8_C(0x00)

/*!  Bit is set to '1' if fast-offset compensation feature is being executed. Bit is cleared to '0' at the end of
 * feature compensation. User should not change the accelerometer configuration while the feature is running.  */
#define BMA530_FOC_RUNNING_MSK                                 UINT8_C(0x04)
#define BMA530_FOC_RUNNING_POS                                 UINT8_C(0x02)

/*!  Bit is set when FIFO size is changed by feature engine. Bit is cleared, when default FIFO size (512bytes) is set
 *  */
#define BMA530_FIFO_SIZE_CHANGED_MSK                           UINT8_C(0x08)
#define BMA530_FIFO_SIZE_CHANGED_POS                           UINT8_C(0x03)

/*! Enables generic interrupt 1 feature */
#define BMA530_GEN_INT1_EN_MSK                                 UINT8_C(0x01)
#define BMA530_GEN_INT1_EN_POS                                 UINT8_C(0x00)

/*! Enables generic interrupt 2 feature */
#define BMA530_GEN_INT2_EN_MSK                                 UINT8_C(0x02)
#define BMA530_GEN_INT2_EN_POS                                 UINT8_C(0x01)

/*! Enables generic interrupt 3 feature */
#define BMA530_GEN_INT3_EN_MSK                                 UINT8_C(0x04)
#define BMA530_GEN_INT3_EN_POS                                 UINT8_C(0x02)

/*! Enables step counter and/or step detection features */
#define BMA530_STEP_EN_MSK                                     UINT8_C(0x08)
#define BMA530_STEP_EN_POS                                     UINT8_C(0x03)

/*! Enables significant motion detection feature */
#define BMA530_SIG_MO_EN_MSK                                   UINT8_C(0x10)
#define BMA530_SIG_MO_EN_POS                                   UINT8_C(0x04)

/*! Enables tilt detection feature */
#define BMA530_TILT_EN_MSK                                     UINT8_C(0x20)
#define BMA530_TILT_EN_POS                                     UINT8_C(0x05)

/*! Enables orientation detection feature */
#define BMA530_ORIENT_EN_MSK                                   UINT8_C(0x40)
#define BMA530_ORIENT_EN_POS                                   UINT8_C(0x06)

/*! Enables accelerometer fast-offset compensation */
#define BMA530_ACC_FOC_EN_MSK                                  UINT8_C(0x80)
#define BMA530_ACC_FOC_EN_POS                                  UINT8_C(0x07)

/*! Data source selection for gen_int1 feature */
#define BMA530_GEN_INT1_DATA_SRC_MSK                           UINT8_C(0x03)
#define BMA530_GEN_INT1_DATA_SRC_POS                           UINT8_C(0x00)

/*! Data source selection for gen_int2 feature */
#define BMA530_GEN_INT2_DATA_SRC_MSK                           UINT8_C(0x0C)
#define BMA530_GEN_INT2_DATA_SRC_POS                           UINT8_C(0x02)

/*! Data source selection for gen_int3 feature */
#define BMA530_GEN_INT3_DATA_SRC_MSK                           UINT8_C(0x30)
#define BMA530_GEN_INT3_DATA_SRC_POS                           UINT8_C(0x04)

/*! Step counter value byte-0 */
#define BMA530_STEP_CNT_OUT_0_MSK                              UINT8_C(0xFF)
#define BMA530_STEP_CNT_OUT_0_POS                              UINT8_C(0x00)

/*! Step counter value byte-1 */
#define BMA530_STEP_CNT_OUT_1_MSK                              UINT8_C(0xFF)
#define BMA530_STEP_CNT_OUT_1_POS                              UINT8_C(0x00)

/*! Step counter value byte-2 */
#define BMA530_STEP_CNT_OUT_2_MSK                              UINT8_C(0xFF)
#define BMA530_STEP_CNT_OUT_2_POS                              UINT8_C(0x00)

/*! Output of orientation detection feature. Device orientation can be either portrait or landscape. Value after device
 * initialization is 0b00 i.e. portrait up. */
#define BMA530_ORIENTATION_PORTRAIT_LANDSCAPE_MSK              UINT8_C(0x03)
#define BMA530_ORIENTATION_PORTRAIT_LANDSCAPE_POS              UINT8_C(0x00)

/*! Output of orientation detection feature. Output is only valid if "ud_en" is enabled. Device orientation can be
 * either face up or face down. Value after device initialization is 0b0 i.e. face up. */
#define BMA530_ORIENTATION_FACE_UP_DOWN_MSK                    UINT8_C(0x04)
#define BMA530_ORIENTATION_FACE_UP_DOWN_POS                    UINT8_C(0x02)

/*! Status of user activity reported by step counter */
#define BMA530_ACTIV_STAT_MSK                                  UINT8_C(0x18)
#define BMA530_ACTIV_STAT_POS                                  UINT8_C(0x03)

/*! Status of generic interrupt 1 motion detection */
#define BMA530_GEN_INT1_STAT_MSK                               UINT8_C(0x20)
#define BMA530_GEN_INT1_STAT_POS                               UINT8_C(0x05)

/*! Status of generic interrupt 2 motion detection */
#define BMA530_GEN_INT2_STAT_MSK                               UINT8_C(0x40)
#define BMA530_GEN_INT2_STAT_POS                               UINT8_C(0x06)

/*! Status of generic interrupt 3 motion detection */
#define BMA530_GEN_INT3_STAT_MSK                               UINT8_C(0x80)
#define BMA530_GEN_INT3_STAT_POS                               UINT8_C(0x07)

/******************************************************************************/
/********************* Macros for supported field values **********************/
/******************************************************************************/
/* Macros to define the supported chip_id values */
#define BMA530_CHIP_ID_BMA530                                  UINT8_C(0xC2) /*! product identifier for BMA530 */

/* Macros to define the supported acc_drdy_int_map values */
#define BMA530_ACC_DRDY_INT_MAP_UNMAPPED                       UINT8_C(0x00) /*! Interrupt is not mapped to any
                                                                              * destination. */
#define BMA530_ACC_DRDY_INT_MAP_INT1                           UINT8_C(0x01) /*! Interrupt is mapped to INT1 pin. */
#define BMA530_ACC_DRDY_INT_MAP_INT2                           UINT8_C(0x02) /*! Interrupt is mapped to INT2 pin. */
#define BMA530_ACC_DRDY_INT_MAP_I3C                            UINT8_C(0x03) /*! Interrupt is mapped to I3C in-band
                                                                              * interrupts. */

/* Macros to define the supported fifo_wm_int_map values */
#define BMA530_FIFO_WM_INT_MAP_UNMAPPED                        UINT8_C(0x00) /*! Interrupt is not mapped to any
                                                                              * destination. */
#define BMA530_FIFO_WM_INT_MAP_INT1                            UINT8_C(0x01) /*! Interrupt is mapped to INT1 pin. */
#define BMA530_FIFO_WM_INT_MAP_INT2                            UINT8_C(0x02) /*! Interrupt is mapped to INT2 pin. */
#define BMA530_FIFO_WM_INT_MAP_I3C                             UINT8_C(0x03) /*! Interrupt is mapped to I3C in-band
                                                                              * interrupts. */

/* Macros to define the supported fifo_full_int_map values */
#define BMA530_FIFO_FULL_INT_MAP_UNMAPPED                      UINT8_C(0x00) /*! Interrupt is not mapped to any
                                                                              * destination. */
#define BMA530_FIFO_FULL_INT_MAP_INT1                          UINT8_C(0x01) /*! Interrupt is mapped to INT1 pin. */
#define BMA530_FIFO_FULL_INT_MAP_INT2                          UINT8_C(0x02) /*! Interrupt is mapped to INT2 pin. */
#define BMA530_FIFO_FULL_INT_MAP_I3C                           UINT8_C(0x03) /*! Interrupt is mapped to I3C in-band
                                                                              * interrupts. */

/* Macros to define the supported gen_int1_int_map values */
#define BMA530_GEN_INT1_INT_MAP_UNMAPPED                       UINT8_C(0x00) /*! Interrupt is not mapped to any
                                                                              * destination. */
#define BMA530_GEN_INT1_INT_MAP_INT1                           UINT8_C(0x01) /*! Interrupt is mapped to INT1 pin. */
#define BMA530_GEN_INT1_INT_MAP_INT2                           UINT8_C(0x02) /*! Interrupt is mapped to INT2 pin. */
#define BMA530_GEN_INT1_INT_MAP_I3C                            UINT8_C(0x03) /*! Interrupt is mapped to I3C in-band
                                                                              * interrupts. */

/* Macros to define the supported gen_int2_int_map values */
#define BMA530_GEN_INT2_INT_MAP_UNMAPPED                       UINT8_C(0x00) /*! Interrupt is not mapped to any
                                                                              * destination. */
#define BMA530_GEN_INT2_INT_MAP_INT1                           UINT8_C(0x01) /*! Interrupt is mapped to INT1 pin. */
#define BMA530_GEN_INT2_INT_MAP_INT2                           UINT8_C(0x02) /*! Interrupt is mapped to INT2 pin. */
#define BMA530_GEN_INT2_INT_MAP_I3C                            UINT8_C(0x03) /*! Interrupt is mapped to I3C in-band
                                                                              * interrupts. */

/* Macros to define the supported gen_int3_int_map values */
#define BMA530_GEN_INT3_INT_MAP_UNMAPPED                       UINT8_C(0x00) /*! Interrupt is not mapped to any
                                                                              * destination. */
#define BMA530_GEN_INT3_INT_MAP_INT1                           UINT8_C(0x01) /*! Interrupt is mapped to INT1 pin. */
#define BMA530_GEN_INT3_INT_MAP_INT2                           UINT8_C(0x02) /*! Interrupt is mapped to INT2 pin. */
#define BMA530_GEN_INT3_INT_MAP_I3C                            UINT8_C(0x03) /*! Interrupt is mapped to I3C in-band
                                                                              * interrupts. */

/* Macros to define the supported step_det_int_map values */
#define BMA530_STEP_DET_INT_MAP_UNMAPPED                       UINT8_C(0x00) /*! Interrupt is not mapped to any
                                                                              * destination. */
#define BMA530_STEP_DET_INT_MAP_INT1                           UINT8_C(0x01) /*! Interrupt is mapped to INT1 pin. */
#define BMA530_STEP_DET_INT_MAP_INT2                           UINT8_C(0x02) /*! Interrupt is mapped to INT2 pin. */
#define BMA530_STEP_DET_INT_MAP_I3C                            UINT8_C(0x03) /*! Interrupt is mapped to I3C in-band
                                                                              * interrupts. */

/* Macros to define the supported step_cnt_int_map values */
#define BMA530_STEP_CNT_INT_MAP_UNMAPPED                       UINT8_C(0x00) /*! Interrupt is not mapped to any
                                                                              * destination. */
#define BMA530_STEP_CNT_INT_MAP_INT1                           UINT8_C(0x01) /*! Interrupt is mapped to INT1 pin. */
#define BMA530_STEP_CNT_INT_MAP_INT2                           UINT8_C(0x02) /*! Interrupt is mapped to INT2 pin. */
#define BMA530_STEP_CNT_INT_MAP_I3C                            UINT8_C(0x03) /*! Interrupt is mapped to I3C in-band
                                                                              * interrupts. */

/* Macros to define the supported sig_mo_int_map values */
#define BMA530_SIG_MO_INT_MAP_UNMAPPED                         UINT8_C(0x00) /*! Interrupt is not mapped to any
                                                                              * destination. */
#define BMA530_SIG_MO_INT_MAP_INT1                             UINT8_C(0x01) /*! Interrupt is mapped to INT1 pin. */
#define BMA530_SIG_MO_INT_MAP_INT2                             UINT8_C(0x02) /*! Interrupt is mapped to INT2 pin. */
#define BMA530_SIG_MO_INT_MAP_I3C                              UINT8_C(0x03) /*! Interrupt is mapped to I3C in-band
                                                                              * interrupts. */

/* Macros to define the supported tilt_int_map values */
#define BMA530_TILT_INT_MAP_UNMAPPED                           UINT8_C(0x00) /*! Interrupt is not mapped to any
                                                                              * destination. */
#define BMA530_TILT_INT_MAP_INT1                               UINT8_C(0x01) /*! Interrupt is mapped to INT1 pin. */
#define BMA530_TILT_INT_MAP_INT2                               UINT8_C(0x02) /*! Interrupt is mapped to INT2 pin. */
#define BMA530_TILT_INT_MAP_I3C                                UINT8_C(0x03) /*! Interrupt is mapped to I3C in-band
                                                                              * interrupts. */

/* Macros to define the supported orient_int_map values */
#define BMA530_ORIENT_INT_MAP_UNMAPPED                         UINT8_C(0x00) /*! Interrupt is not mapped to any
                                                                              * destination. */
#define BMA530_ORIENT_INT_MAP_INT1                             UINT8_C(0x01) /*! Interrupt is mapped to INT1 pin. */
#define BMA530_ORIENT_INT_MAP_INT2                             UINT8_C(0x02) /*! Interrupt is mapped to INT2 pin. */
#define BMA530_ORIENT_INT_MAP_I3C                              UINT8_C(0x03) /*! Interrupt is mapped to I3C in-band
                                                                              * interrupts. */

/* Macros to define the supported acc_foc_int_map values */
#define BMA530_ACC_FOC_INT_MAP_UNMAPPED                        UINT8_C(0x00) /*! Interrupt is not mapped to any
                                                                              * destination. */
#define BMA530_ACC_FOC_INT_MAP_INT1                            UINT8_C(0x01) /*! Interrupt is mapped to INT1 pin. */
#define BMA530_ACC_FOC_INT_MAP_INT2                            UINT8_C(0x02) /*! Interrupt is mapped to INT2 pin. */
#define BMA530_ACC_FOC_INT_MAP_I3C                             UINT8_C(0x03) /*! Interrupt is mapped to I3C in-band
                                                                              * interrupts. */

/* Macros to define the supported feat_eng_err_int_map values */
#define BMA530_FEAT_ENG_ERR_INT_MAP_UNMAPPED                   UINT8_C(0x00) /*! Interrupt is not mapped to any
                                                                              * destination. */
#define BMA530_FEAT_ENG_ERR_INT_MAP_INT1                       UINT8_C(0x01) /*! Interrupt is mapped to INT1 pin. */
#define BMA530_FEAT_ENG_ERR_INT_MAP_INT2                       UINT8_C(0x02) /*! Interrupt is mapped to INT2 pin. */
#define BMA530_FEAT_ENG_ERR_INT_MAP_I3C                        UINT8_C(0x03) /*! Interrupt is mapped to I3C in-band
                                                                              * interrupts. */

/* Macros to define the supported feat_init_stat values */
#define BMA530_FEAT_INIT_STAT_INIT_NOT_OK                      UINT8_C(0x00) /*! Feature engine is not initialized */
#define BMA530_FEAT_INIT_STAT_INIT_OK                          UINT8_C(0x01) /*! Feature engine is initialized */
#define BMA530_FEAT_INIT_STAT_UN_DEF_1                         UINT8_C(0x02) /*! Reserved */
#define BMA530_FEAT_INIT_STAT_UN_DEF_2                         UINT8_C(0x03) /*! Reserved */

/* Macros to define the supported gen_int1_data_src values */
#define BMA530_GEN_INT1_DATA_SRC_DATA_SRC_1                    UINT8_C(0x00) /*! Uses 50Hz filter data */
#define BMA530_GEN_INT1_DATA_SRC_DATA_SRC_2                    UINT8_C(0x01) /*! Uses 200Hz filter data */
#define BMA530_GEN_INT1_DATA_SRC_DATA_SRC_3                    UINT8_C(0x02) /*! Uses user filter data */
#define BMA530_GEN_INT1_DATA_SRC_DATA_SRC_4                    UINT8_C(0x03) /*! Uses 50Hz filter data. Same as
                                                                              * data_src_1 */

/* Macros to define the supported gen_int2_data_src values */
#define BMA530_GEN_INT2_DATA_SRC_DATA_SRC_1                    UINT8_C(0x00) /*! Uses 50Hz filter data */
#define BMA530_GEN_INT2_DATA_SRC_DATA_SRC_2                    UINT8_C(0x01) /*! Uses 200Hz filter data */
#define BMA530_GEN_INT2_DATA_SRC_DATA_SRC_3                    UINT8_C(0x02) /*! Uses user filter data */
#define BMA530_GEN_INT2_DATA_SRC_DATA_SRC_4                    UINT8_C(0x03) /*! Uses 50Hz filter data. Same as
                                                                              * data_src_1 */

/* Macros to define the supported gen_int3_data_src values */
#define BMA530_GEN_INT3_DATA_SRC_DATA_SRC_1                    UINT8_C(0x00) /*! Uses 50Hz filter data */
#define BMA530_GEN_INT3_DATA_SRC_DATA_SRC_2                    UINT8_C(0x01) /*! Uses 200Hz filter data */
#define BMA530_GEN_INT3_DATA_SRC_DATA_SRC_3                    UINT8_C(0x02) /*! Uses user filter data */
#define BMA530_GEN_INT3_DATA_SRC_DATA_SRC_4                    UINT8_C(0x03) /*! Uses 50Hz filter data. Same as
                                                                              * data_src_1 */

/* Macros to define the supported orientation_portrait_landscape values */
#define BMA530_ORIENTATION_PORTRAIT_LANDSCAPE_PORTRAIT_UP      UINT8_C(0x00) /*! Portrait up orientation */
#define BMA530_ORIENTATION_PORTRAIT_LANDSCAPE_LANDSCAPE_LEFT   UINT8_C(0x01) /*! Landscape left orientation */
#define BMA530_ORIENTATION_PORTRAIT_LANDSCAPE_PORTRAIT_DOWN    UINT8_C(0x02) /*! Portrait down orientation */
#define BMA530_ORIENTATION_PORTRAIT_LANDSCAPE_LANDSCAPE_RIGHT  UINT8_C(0x03) /*! Landscape right orientation */

/* Macros to define the supported orientation_face_up_down values */
#define BMA530_ORIENTATION_FACE_UP_DOWN_FACE_UP                UINT8_C(0x00) /*! Face up orientation */
#define BMA530_ORIENTATION_FACE_UP_DOWN_FACE_DOWN              UINT8_C(0x01) /*! Face down orientation */

/* Macros to define the supported activ_stat values */
#define BMA530_ACTIV_STAT_UNKNOWN                              UINT8_C(0x00) /*! After device reset or while step
                                                                              * counter is disabled */
#define BMA530_ACTIV_STAT_STILL                                UINT8_C(0x01) /*! User is stationary */
#define BMA530_ACTIV_STAT_WALKING                              UINT8_C(0x02) /*! User is walking */
#define BMA530_ACTIV_STAT_RUNNING                              UINT8_C(0x03) /*! User is running */

#define BMA530_INT_STATUS_INT1                                 UINT8_C(1)
#define BMA530_INT_STATUS_INT2                                 UINT8_C(2)
#define BMA530_INT_STATUS_I3C                                  UINT8_C(3)

/******************************************************************************/
/***************** Structures for handling register content *******************/
/******************************************************************************/

/*!
 * @brief Structure holding interrupt status register
 */
struct bma530_int_status
{
    /*! Accelerometer data ready interrupt status */
    uint8_t acc_drdy_int_status;

    /*! FIFO watermark interrupt status */
    uint8_t fifo_wm_int_status;

    /*! FIFO full interrupt status */
    uint8_t fifo_full_int_status;

    /*! Generic interrupt 1 interrupt status */
    uint8_t gen_int1_int_status;

    /*! Generic interrupt 2 interrupt status */
    uint8_t gen_int2_int_status;

    /*! Generic interrupt 3 interrupt status */
    uint8_t gen_int3_int_status;

    /*! Step detection interrupt status */
    uint8_t step_det_int_status;

    /*! Step counter watermark interrupt status */
    uint8_t step_cnt_int_status;

    /*! Significant motion detection interrupt status */
    uint8_t sig_mo_int_status;

    /*! Tilt detection interrupt status */
    uint8_t tilt_int_status;

    /*! Orientation detection status */
    uint8_t orient_int_status;

    /*! Accelerometer FOC completion status */
    uint8_t acc_foc_int_status;

    /*! MCU error interrupt status */
    uint8_t feat_eng_err_int_status;
};

/*!
 * @brief Structure holding Interrupt source and its configuration
 */
struct bma530_int_status_types
{
    /*! Interrupt soruce */
    uint8_t int_src;

    /*! Interrupt status configuration */
    struct bma530_int_status int_status;
};

/*!
 * @brief Structure holding Interrupt mapping register
 */
struct bma530_int_map
{
    /*! Data ready interrupt mapping */
    uint8_t acc_drdy_int_map;

    /*! FIFO watermark interrupt mapping */
    uint8_t fifo_wm_int_map;

    /*! FIFO full interrupt mapping */
    uint8_t fifo_full_int_map;

    /*! Generic interrupt 1 interrupt mapping */
    uint8_t gen_int1_int_map;

    /*! Generic interrupt 2 interrupt mapping */
    uint8_t gen_int2_int_map;

    /*! Generic interrupt 3 interrupt mapping */
    uint8_t gen_int3_int_map;

    /*! Step detection interrupt mapping */
    uint8_t step_det_int_map;

    /*! Step counter watermark interrupt mapping */
    uint8_t step_cnt_int_map;

    /*! Significant motion detection interrupt mapping */
    uint8_t sig_mo_int_map;

    /*! Tilt detection interrupt mapping */
    uint8_t tilt_int_map;

    /*! Orientation detection interrupt mapping */
    uint8_t orient_int_map;

    /*! Accelerometer FOC completion interrupt mapping */
    uint8_t acc_foc_int_map;

    /*! MCU error interrupt mapping */
    uint8_t feat_eng_err_int_map;
};

/*!
 * @brief Structure holding feature engine ('feat_eng') general purpose flags
 */
struct bma530_feat_eng_gp_flags
{
    /*!  Feature engine initialization status  */
    uint8_t feat_init_stat;

    /*!  Bit is set to '1' if fast-offset compensation feature is being executed. Bit is cleared to '0' at the end of
     * feature compensation. User should not change the accelerometer configuration while the feature is running.  */
    uint8_t foc_running;

    /*!  Bit is set when FIFO size is changed by feature engine. Bit is cleared, when default FIFO size (512bytes) is
     * set  */
    uint8_t fifo_size_changed;
};

/*!
 * @brief Structure holding feature engine ('feat_eng') general purpose register 0
 */
struct bma530_feat_eng_gpr_0
{
    /*! Enables generic interrupt 1 feature */
    uint8_t gen_int1_en;

    /*! Enables generic interrupt 2 feature */
    uint8_t gen_int2_en;

    /*! Enables generic interrupt 3 feature */
    uint8_t gen_int3_en;

    /*! Enables step counter and/or step detection features */
    uint8_t step_en;

    /*! Enables significant motion detection feature */
    uint8_t sig_mo_en;

    /*! Enables tilt detection feature */
    uint8_t tilt_en;

    /*! Enables orientation detection feature */
    uint8_t orient_en;

    /*! Enables accelerometer fast-offset compensation */
    uint8_t acc_foc_en;
};

/*!
 * @brief Structure holding feature engine ('feat_eng') general purpose register 1
 */
struct bma530_feat_eng_gpr_1
{
    /*! Data source selection for gen_int1 feature */
    uint8_t gen_int1_data_src;

    /*! Data source selection for gen_int2 feature */
    uint8_t gen_int2_data_src;

    /*! Data source selection for gen_int3 feature */
    uint8_t gen_int3_data_src;
};

/*!
 * @brief Structure holding feature engine ('feat_eng') general purpose register 5
 */
struct bma530_feat_eng_gpr_5
{
    /*! Output of orientation detection feature. Device orientation can be either portrait or landscape. Value after
     * device initialization is 0b00 i.e. portrait up. */
    uint8_t orientation_portrait_landscape;

    /*! Output of orientation detection feature. Output is only valid if "ud_en" is enabled. Device orientation can be
     * either face up or face down. Value after device initialization is 0b0 i.e. face up. */
    uint8_t orientation_face_up_down;

    /*! Status of user activity reported by step counter */
    uint8_t activ_stat;

    /*! Status of generic interrupt 1 motion detection */
    uint8_t gen_int1_stat;

    /*! Status of generic interrupt 2 motion detection */
    uint8_t gen_int2_stat;

    /*! Status of generic interrupt 3 motion detection */
    uint8_t gen_int3_stat;
};

/******************************************************************************/
/********************** Function prototype declarations ***********************/
/******************************************************************************/

/**
 * \ingroup bma530
 * \defgroup bma530ApiRegs BMA530 Registers
 * @brief Set / Get data from the given register address of the sensor
 */

/*!
 * \ingroup bma530ApiRegs
 * \page bma530_api_bma530_get_chip_id bma530_get_chip_id
 * \code
 * int8_t bma530_get_chip_id(uint8_t *chip_id, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to get the The product chip_id. Upper four bits are fix and lower four bits are boot loaded from OTP.
 *
 * @param[out] chip_id : Chip id
 * @param[in]  dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma530_get_chip_id(uint8_t *chip_id, struct bma5_dev *dev);

/*!
 * \ingroup bma530ApiRegs
 * \page bma530_api_bma530_get_int_status_int1_0 bma530_get_int_status_int1_0
 * \code
 * int8_t bma530_get_int_status(struct bma530_int_status_types *config, uint8_t n_status, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to get the INT1 interrupt status register 0
 *
 * @param[out] config : Structure instance of bma530_int_status_int1_0
 * @param[in]  dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma530_get_int_status(struct bma530_int_status_types *config, uint8_t n_status, struct bma5_dev *dev);

/*!
 * \ingroup bma530ApiRegs
 * \page bma530_api_bma530_get_int_status_int1_0 bma530_get_int_status_int1_0
 * \code
 * int8_t bma530_set_int_status(const struct bma530_int_status_types *config, uint8_t n_status, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to set the INT1 interrupt status register 0
 *
 * @param[in] config : Structure instance of bma530_int_status_int1_0
 * @param[in] dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma530_set_int_status(const struct bma530_int_status_types *config, uint8_t n_status, struct bma5_dev *dev);

/*!
 * \ingroup bma530ApiRegs
 * \page bma530_api_bma530_get_int_map_0 bma530_get_int_map_0
 * \code
 * int8_t bma530_get_int_map_0(struct bma530_int_map *config, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to get the Interrupt mapping register 0
 *
 * @param[out] config : Structure instance of bma530_int_map_0
 * @param[in]  dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma530_get_int_map(struct bma530_int_map *config, struct bma5_dev *dev);

/*!
 * \ingroup bma530ApiRegs
 * \page bma530_api_bma530_get_int_map_0 bma530_get_int_map_0
 * \code
 * int8_t bma530_set_int_map(const struct bma530_int_map *config, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to set the Interrupt mapping register 0
 *
 * @param[in] config : Structure instance of bma530_int_map_0
 * @param[in] dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma530_set_int_map(const struct bma530_int_map *config, struct bma5_dev *dev);

/*!
 * \ingroup bma530ApiRegs
 * \page bma530_api_bma530_get_feat_eng_gp_flags bma530_get_feat_eng_gp_flags
 * \code
 * int8_t bma530_get_feat_eng_gp_flags(struct bma530_feat_eng_gp_flags *config, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to get the feature engine ('feat_eng') general purpose flags
 *
 * @param[out] config : Structure instance of bma530_feat_eng_gp_flags
 * @param[in]  dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma530_get_feat_eng_gp_flags(struct bma530_feat_eng_gp_flags *config, struct bma5_dev *dev);

/*!
 * \ingroup bma530ApiRegs
 * \page bma530_api_bma530_get_feat_eng_gpr_0 bma530_get_feat_eng_gpr_0
 * \code
 * int8_t bma530_get_feat_eng_gpr_0(struct bma530_feat_eng_gpr_0 *config, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to get the feature engine ('feat_eng') general purpose register 0
 *
 * @param[out] config : Structure instance of bma530_feat_eng_gpr_0
 * @param[in]  dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma530_get_feat_eng_gpr_0(struct bma530_feat_eng_gpr_0 *config, struct bma5_dev *dev);

/*!
 * \ingroup bma530ApiRegs
 * \page bma530_api_bma530_get_feat_eng_gpr_0 bma530_get_feat_eng_gpr_0
 * \code
 * int8_t bma530_set_feat_eng_gpr_0(const struct bma530_feat_eng_gpr_0 *config, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to set the feature engine ('feat_eng') general purpose register 0
 *
 * @param[in] config : Structure instance of bma530_feat_eng_gpr_0
 * @param[in] dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma530_set_feat_eng_gpr_0(const struct bma530_feat_eng_gpr_0 *config, struct bma5_dev *dev);

/*!
 * \ingroup bma530ApiRegs
 * \page bma530_api_bma530_get_feat_eng_gpr_1 bma530_get_feat_eng_gpr_1
 * \code
 * int8_t bma530_get_feat_eng_gpr_1(struct bma530_feat_eng_gpr_1 *config, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to get the feature engine ('feat_eng') general purpose register 1
 *
 * @param[out] config : Structure instance of bma530_feat_eng_gpr_1
 * @param[in]  dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma530_get_feat_eng_gpr_1(struct bma530_feat_eng_gpr_1 *config, struct bma5_dev *dev);

/*!
 * \ingroup bma530ApiRegs
 * \page bma530_api_bma530_get_feat_eng_gpr_1 bma530_get_feat_eng_gpr_1
 * \code
 * int8_t bma530_set_feat_eng_gpr_1(const struct bma530_feat_eng_gpr_1 *config, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to set the feature engine ('feat_eng') general purpose register 1
 *
 * @param[in] config : Structure instance of bma530_feat_eng_gpr_1
 * @param[in] dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma530_set_feat_eng_gpr_1(const struct bma530_feat_eng_gpr_1 *config, struct bma5_dev *dev);

/*!
 * \ingroup bma530ApiRegs
 * \page bma530_api_bma530_get_feat_eng_gpr_2 bma530_get_feat_eng_gpr_2
 * \code
 * int8_t bma530_get_feat_eng_gpr_2(uint8_t *step_cnt_out_0, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to get the feature engine ('feat_eng') general purpose register 2
 *
 * @param[out] step_cnt_out_0 : Step counter value byte-0
 * @param[in]  dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma530_get_feat_eng_gpr_2(uint8_t *step_cnt_out_0, struct bma5_dev *dev);

/*!
 * \ingroup bma530ApiRegs
 * \page bma530_api_bma530_get_feat_eng_gpr_3 bma530_get_feat_eng_gpr_3
 * \code
 * int8_t bma530_get_feat_eng_gpr_3(uint8_t *step_cnt_out_1, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to get the feature engine ('feat_eng') general purpose register 3
 *
 * @param[out] step_cnt_out_1 : Step counter value byte-1
 * @param[in]  dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma530_get_feat_eng_gpr_3(uint8_t *step_cnt_out_1, struct bma5_dev *dev);

/*!
 * \ingroup bma530ApiRegs
 * \page bma530_api_bma530_get_feat_eng_gpr_4 bma530_get_feat_eng_gpr_4
 * \code
 * int8_t bma530_get_feat_eng_gpr_4(uint8_t *step_cnt_out_2, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to get the feature engine ('feat_eng') general purpose register 4
 *
 * @param[out] step_cnt_out_2 : Step counter value byte-2
 * @param[in]  dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma530_get_feat_eng_gpr_4(uint8_t *step_cnt_out_2, struct bma5_dev *dev);

/*!
 * \ingroup bma530ApiRegs
 * \page bma530_api_bma530_get_feat_eng_gpr_5 bma530_get_feat_eng_gpr_5
 * \code
 * int8_t bma530_get_feat_eng_gpr_5(struct bma530_feat_eng_gpr_5 *config, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to get the feature engine ('feat_eng') general purpose register 5
 *
 * @param[out] config : Structure instance of bma530_feat_eng_gpr_5
 * @param[in]  dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma530_get_feat_eng_gpr_5(struct bma530_feat_eng_gpr_5 *config, struct bma5_dev *dev);

#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif /*_BMA530_H */
