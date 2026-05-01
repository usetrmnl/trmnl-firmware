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
* @file       bma5_defs.h
* @date       2024-07-29
* @version    v4.2.0
*
*/

#ifndef _BMA5_DEFS_H
#define _BMA5_DEFS_H

/******************************************************************************/
/****************************** Header files **********************************/
/******************************************************************************/
#ifdef __KERNEL__
#include <linux/types.h>
#include <linux/kernel.h>
#else
#include <stdint.h>
#include <stddef.h>
#endif

/******************************************************************************/
/****************************** Common macros *********************************/
/******************************************************************************/
#ifdef __KERNEL__
#if !defined(UINT8_C) && !defined(INT8_C)
#define INT8_C(x)    S8_C(x)
#define UINT8_C(x)   U8_C(x)
#endif

#if !defined(UINT16_C) && !defined(INT16_C)
#define INT16_C(x)   S16_C(x)
#define UINT16_C(x)  U16_C(x)
#endif

#if !defined(INT32_C) && !defined(UINT32_C)
#define INT32_C(x)   S32_C(x)
#define UINT32_C(x)  U32_C(x)
#endif

#if !defined(INT64_C) && !defined(UINT64_C)
#define INT64_C(x)   S64_C(x)
#define UINT64_C(x)  U64_C(x)
#endif
#endif

/*! C standard macros */
#ifndef NULL
#ifdef __cplusplus
#define NULL         0
#else
#define NULL         ((void *) 0)
#endif
#endif

/******************************************************************************/
/************************* General macro definitions **************************/
/******************************************************************************/
/*! Macro to get bits from register data */
#define BMA5_GET_BITS(reg_data, bitname) \
    ((reg_data & (bitname##_MSK)) >> (bitname##_POS))

/*! Macro to get bit value at position zero of register data */
#define BMA5_GET_BITS_POS_0(reg_data, bitname) \
    (reg_data & (bitname##_MSK))

/*! Macro to set bits from register data */
#define BMA5_SET_BITS(reg_data, bitname, data) \
    ((reg_data & ~(bitname##_MSK)) | ((data << bitname##_POS) & bitname##_MSK))

/*! Macro to set bit value at position zero of register data */
#define BMA5_SET_BITS_POS_0(reg_data, bitname, data) \
    ((reg_data & ~(bitname##_MSK)) | (data & bitname##_MSK))

/*! Maximum buffer size of temporary buffer used for serial communication,
    in bytes */
#define BMA5_MAX_BUFFER_SIZE                       UINT8_C(128)

/*! Mask definitions for SPI read/write address */
#define BMA5_SPI_RD_MSK                            UINT8_C(0x80)
#define BMA5_SPI_WR_MSK                            UINT8_C(0x7F)

/*! I2C address */
#define BMA5_I2C_ADDRESS                           UINT8_C(0x18)

/******************************************************************************/
/*********************** Interface specific definitions ***********************/
/******************************************************************************/
/*! Serial communication interface return type */
#ifndef BMA5_INTF_RET_TYPE
#define BMA5_INTF_RET_TYPE                         int8_t
#endif

/*! Serial communication interface success code */
#define BMA5_INTF_RET_SUCCESS                      INT8_C(0)

/******************************************************************************/
/***************************** API status codes *******************************/
/******************************************************************************/
/*! API success code */
#define BMA5_OK                                    INT8_C(0)

/*! API error status codes */
#define BMA5_E_NULL_PTR                            INT8_C(-1)
#define BMA5_E_COM_FAIL                            INT8_C(-2)
#define BMA5_E_DEV_NOT_FOUND                       INT8_C(-3)
#define BMA5_E_INVALID_CONTEXT_PARAM               INT8_C(-4)
#define BMA5_E_INVALID_INTERRUPT                   INT8_C(-5)
#define BMA5_E_INVALID_GEN_INT                     INT8_C(-6)
#define BMA5_E_INVALID_INT_STATUS                  INT8_C(-7)

/******************************************************************************/
/***************************** Register addresses *****************************/
/******************************************************************************/
/*! This register contains internal health status information */
#define BMA5_REG_HEALTH_STATUS                     UINT8_C(0x02)

/*! Command register to activate suspend mode. */
#define BMA5_REG_CMD_SUSPEND                       UINT8_C(0x04)

/*! Global error flags */
#define BMA5_REG_CONFIG_STATUS                     UINT8_C(0x10)

/*! Global status flags */
#define BMA5_REG_SENSOR_STATUS                     UINT8_C(0x11)

/*! ACC data register 0 */
#define BMA5_REG_ACC_DATA_0                        UINT8_C(0x18)

/*! ACC data register 1 */
#define BMA5_REG_ACC_DATA_1                        UINT8_C(0x19)

/*! ACC data register 2 */
#define BMA5_REG_ACC_DATA_2                        UINT8_C(0x1A)

/*! ACC data register 3 */
#define BMA5_REG_ACC_DATA_3                        UINT8_C(0x1B)

/*! ACC data register 4 */
#define BMA5_REG_ACC_DATA_4                        UINT8_C(0x1C)

/*! ACC data register 5 */
#define BMA5_REG_ACC_DATA_5                        UINT8_C(0x1D)

/*! Temperature data register */
#define BMA5_REG_TEMP_DATA                         UINT8_C(0x1E)

/*! Sensor time register 0 */
#define BMA5_REG_SENSOR_TIME_0                     UINT8_C(0x1F)

/*! Sensor time register 1 */
#define BMA5_REG_SENSOR_TIME_1                     UINT8_C(0x20)

/*! Sensor time register 2 */
#define BMA5_REG_SENSOR_TIME_2                     UINT8_C(0x21)

/*! FIFO fill level register (LSB) */
#define BMA5_REG_FIFO_LEVEL_0                      UINT8_C(0x22)

/*! FIFO fill level register (MSB) */
#define BMA5_REG_FIFO_LEVEL_1                      UINT8_C(0x23)

/*! FIFO data register */
#define BMA5_REG_FIFO_DATA_OUT                     UINT8_C(0x24)

/*! Accelerometer configuration register 0 */
#define BMA5_REG_ACC_CONF_0                        UINT8_C(0x30)

/*! Accelerometer configuration register 1 */
#define BMA5_REG_ACC_CONF_1                        UINT8_C(0x31)

/*! Accelerometer configuration register 2 */
#define BMA5_REG_ACC_CONF_2                        UINT8_C(0x32)

/*! Temperature Sensor configuration register */
#define BMA5_REG_TEMP_CONF                         UINT8_C(0x33)

/*! Configuration register for INT1 */
#define BMA5_REG_INT1_CONF                         UINT8_C(0x34)

/*! Configuration register for INT2 */
#define BMA5_REG_INT2_CONF                         UINT8_C(0x35)

/*! Serial interface settings */
#define BMA5_REG_IF_CONF_0                         UINT8_C(0x3A)

/*! Serial interface settings */
#define BMA5_REG_IF_CONF_1                         UINT8_C(0x3B)

/*! FIFO control register */
#define BMA5_REG_FIFO_CTRL                         UINT8_C(0x40)

/*! FIFO configuration register 0 */
#define BMA5_REG_FIFO_CONF_0                       UINT8_C(0x41)

/*! FIFO configuration register 1 */
#define BMA5_REG_FIFO_CONF_1                       UINT8_C(0x42)

/*! FIFO watermark level register (LSB) */
#define BMA5_REG_FIFO_WM_0                         UINT8_C(0x43)

/*! FIFO watermark level register (MSB) */
#define BMA5_REG_FIFO_WM_1                         UINT8_C(0x44)

/*! feature engine ('feat_eng') configuration register */
#define BMA5_REG_FEAT_ENG_CONF                     UINT8_C(0x50)

/*! feature engine ('feat_eng') status register */
#define BMA5_REG_FEAT_ENG_STATUS                   UINT8_C(0x51)

/*! feature engine ('feat_eng') general purpose register configuration register */
#define BMA5_REG_FEAT_ENG_GPR_CONF                 UINT8_C(0x53)

/*! feature engine ('feat_eng') general purpose register control register */
#define BMA5_REG_FEAT_ENG_GPR_CTRL                 UINT8_C(0x54)

/*! feature engine ('feat_eng') feature data start address */
#define BMA5_REG_FEATURE_DATA_ADDR                 UINT8_C(0x5E)

/*! feature engine ('feat_eng') feature data */
#define BMA5_REG_FEATURE_DATA_TX                   UINT8_C(0x5F)

/*! user offset (x-axis) */
#define BMA5_REG_ACC_OFFSET_0                      UINT8_C(0x70)

/*! user offset (x-axis) */
#define BMA5_REG_ACC_OFFSET_1                      UINT8_C(0x71)

/*! user offset (y-axis) */
#define BMA5_REG_ACC_OFFSET_2                      UINT8_C(0x72)

/*! user offset (y-axis) */
#define BMA5_REG_ACC_OFFSET_3                      UINT8_C(0x73)

/*! user offset (z-axis) */
#define BMA5_REG_ACC_OFFSET_4                      UINT8_C(0x74)

/*! user offset (z-axis) */
#define BMA5_REG_ACC_OFFSET_5                      UINT8_C(0x75)

/*! Select NORMAL/SELF_TEST mode and test data. If you write to this register, the ACC data path is reset. */
#define BMA5_REG_ACC_SELF_TEST                     UINT8_C(0x76)

/*! Command Register */
#define BMA5_REG_CMD                               UINT8_C(0x7E)

/******************************************************************************/
/********************** Register macros for bit masking ***********************/
/******************************************************************************/
/*! The value 0xF indicate a good internal health state. */
#define BMA5_SENSOR_HEALTH_STATUS_MSK              UINT8_C(0x0F)
#define BMA5_SENSOR_HEALTH_STATUS_POS              UINT8_C(0x00)

/*! Write '1' to activate suspend mode. The register content prior to entering this power mode will NOT be lost. */
#define BMA5_SUSPEND_MSK                           UINT8_C(0x01)
#define BMA5_SUSPEND_POS                           UINT8_C(0x00)

/*! Set by feature engine in case of feature engine error condition. Needs to be reseted by the host. For more details
 * there are further status register in the feature engine section and inside the DMA region. */
#define BMA5_FEAT_ENG_ERR_MSK                      UINT8_C(0x01)
#define BMA5_FEAT_ENG_ERR_POS                      UINT8_C(0x00)

/*! This flag is set if the ACC configuration in ACC_CONF_0, ACC_CONF_1, and ACC_CONF_2 is an invalid combination. */
#define BMA5_ACC_CONF_ERR_MSK                      UINT8_C(0x02)
#define BMA5_ACC_CONF_ERR_POS                      UINT8_C(0x01)

/*! Set when new ACC data is available. This flag can be cleared by writing '1' to it. */
#define BMA5_ACC_DATA_RDY_MSK                      UINT8_C(0x01)
#define BMA5_ACC_DATA_RDY_POS                      UINT8_C(0x00)

/*! Set when new temperature data is available. This flag can be cleared by writing '1' to it. */
#define BMA5_TEMPERATURE_RDY_MSK                   UINT8_C(0x02)
#define BMA5_TEMPERATURE_RDY_POS                   UINT8_C(0x01)

/*! Sensor is ready for operation. */
#define BMA5_SENSOR_RDY_MSK                        UINT8_C(0x04)
#define BMA5_SENSOR_RDY_POS                        UINT8_C(0x02)

/*! Accelerometer data for x-axis. (LSB). The full 16bit range cover the selected g-range. (e.g. 8G-range: 1LSB =
 * 16/65536=0.244 mg) */
#define BMA5_ACC_X_7_0_MSK                         UINT8_C(0xFF)
#define BMA5_ACC_X_7_0_POS                         UINT8_C(0x00)

/*! Accelerometer data for x-axis. (MSB). The full 16bit range cover the selected g-range. (e.g. 8G-range: 1LSB =
 * 16/65536=0.244 mg) */
#define BMA5_ACC_X_15_8_MSK                        UINT8_C(0xFF)
#define BMA5_ACC_X_15_8_POS                        UINT8_C(0x00)

/*! Accelerometer data for y-axis. (LSB). The full 16bit range cover the selected g-range. (e.g. 8G-range: 1LSB =
 * 16/65536=0.244 mg) */
#define BMA5_ACC_Y_7_0_MSK                         UINT8_C(0xFF)
#define BMA5_ACC_Y_7_0_POS                         UINT8_C(0x00)

/*! Accelerometer data for y-axis. (MSB). The full 16bit range cover the selected g-range. (e.g. 8G-range: 1LSB =
 * 16/65536=0.244 mg) */
#define BMA5_ACC_Y_15_8_MSK                        UINT8_C(0xFF)
#define BMA5_ACC_Y_15_8_POS                        UINT8_C(0x00)

/*! Accelerometer data for z-axis. (LSB). The full 16bit range cover the selected g-range. (e.g. 8G-range: 1LSB =
 * 16/65536=0.244 mg) */
#define BMA5_ACC_Z_7_0_MSK                         UINT8_C(0xFF)
#define BMA5_ACC_Z_7_0_POS                         UINT8_C(0x00)

/*! Accelerometer data for z-axis. (MSB). The full 16bit range cover the selected g-range. (e.g. 8G-range: 1LSB =
 * 16/65536=0.244 mg) */
#define BMA5_ACC_Z_15_8_MSK                        UINT8_C(0xFF)
#define BMA5_ACC_Z_15_8_POS                        UINT8_C(0x00)

/*! Calculated temperature. Resolution: 1 K/LSB. The value 0 represents 23°C. */
#define BMA5_TEMP_DATA_MSK                         UINT8_C(0xFF)
#define BMA5_TEMP_DATA_POS                         UINT8_C(0x00)

/*! Sensor time in units 1 LSB = 312.5us */
#define BMA5_SENSOR_TIME_7_0_MSK                   UINT8_C(0xFF)
#define BMA5_SENSOR_TIME_7_0_POS                   UINT8_C(0x00)

/*! Sensor time in units 1 LSB = 312.5us */
#define BMA5_SENSOR_TIME_15_8_MSK                  UINT8_C(0xFF)
#define BMA5_SENSOR_TIME_15_8_POS                  UINT8_C(0x00)

/*! Sensor time in units 1 LSB = 312.5us */
#define BMA5_SENSOR_TIME_23_16_MSK                 UINT8_C(0xFF)
#define BMA5_SENSOR_TIME_23_16_POS                 UINT8_C(0x00)

/*! The fill level of the fifo only reflects the stored data. The frame header are not stored and not part of the FIFO
 * fill level. To read out complete FIFO, the best way is to read as long as valid frames are read. LSB of the FIFO fill
 * level. Should be read before the MSB register.  */
#define BMA5_FIFO_FILL_LEVEL_7_0_MSK               UINT8_C(0xFF)
#define BMA5_FIFO_FILL_LEVEL_7_0_POS               UINT8_C(0x00)

/*! MSB of the FIFO fill level. Should be read after the LSB register. */
#define BMA5_FIFO_FILL_LEVEL_10_8_MSK              UINT8_C(0x07)
#define BMA5_FIFO_FILL_LEVEL_10_8_POS              UINT8_C(0x00)

/*! Output of the FIFO. During burst reads on this address the address increment stops and the FIFO can be read out with
 * help of the burst read. The type of data stored in the FIFO depends on configuration stored in FIFO_CONF_* registers.
 * */
#define BMA5_FIFO_DATA_OUT_MSK                     UINT8_C(0xFF)
#define BMA5_FIFO_DATA_OUT_POS                     UINT8_C(0x00)

/*! This bit enables/disables the accelerometer and the temperature sensor. */
#define BMA5_SENSOR_CTRL_MSK                       UINT8_C(0x0F)
#define BMA5_SENSOR_CTRL_POS                       UINT8_C(0x00)

/*! The ODR (Output Data Rate) in Hz. Not all settings are available in all power modes */
#define BMA5_ACC_ODR_MSK                           UINT8_C(0x0F)
#define BMA5_ACC_ODR_POS                           UINT8_C(0x00)

/*! Accelerometer bandwith parameter. This parameter determines the filter configuration. The different settings have a
 * different impact depending on the setting of the power_mode bit. The name of the settings are therefore
 * (HPM-setting)_(LPM-setting). (e.g. norm_avg4 means norm mode for HPM and avg4 for LPM) */
#define BMA5_ACC_BWP_MSK                           UINT8_C(0x70)
#define BMA5_ACC_BWP_POS                           UINT8_C(0x04)

/*! With this config bit, it is possible to set the basic measurement power mode. There are two possible settings:LPM
 * (Low Power Mode) with duty cycling or HPM (High Performance Mode) with continous measurement. This setting has an
 * influence on the signal path and the filter settings, too. */
#define BMA5_POWER_MODE_MSK                        UINT8_C(0x80)
#define BMA5_POWER_MODE_POS                        UINT8_C(0x07)

/*! The measurement range of the accelerometer. This setting has influence on the scaling of the ACC_DATA registers. */
#define BMA5_ACC_RANGE_MSK                         UINT8_C(0x03)
#define BMA5_ACC_RANGE_POS                         UINT8_C(0x00)

/*! Select roll-off of IIR filter in continuous mode. */
#define BMA5_ACC_IIR_RO_MSK                        UINT8_C(0x0C)
#define BMA5_ACC_IIR_RO_POS                        UINT8_C(0x02)

/*! Select the performance mode of the sensor. The choice is between high performance with lower noise or reduce the
 * power consumption but with an increased noise level. The default is the high performance (lower noise). Changing this
 * setting from default migh also influence the sensor behaviour like offset. This configuration should only be used in
 * HPM.  */
#define BMA5_NOISE_MODE_MSK                        UINT8_C(0x10)
#define BMA5_NOISE_MODE_POS                        UINT8_C(0x04)

/*! Configuration bit to enable/disable the auto clear mechanism of the data ready interrupt. If enabled, a clock like
 * with freq=odr can be enabled on the external interrupt pin. */
#define BMA5_ACC_DRDY_INT_AUTO_CLEAR_MSK           UINT8_C(0x80)
#define BMA5_ACC_DRDY_INT_AUTO_CLEAR_POS           UINT8_C(0x07)

/*! Select rate in Hz at which the temperature is sampled. */
#define BMA5_TEMP_RATE_MSK                         UINT8_C(0x07)
#define BMA5_TEMP_RATE_POS                         UINT8_C(0x00)

/*! Select the input source for the temperature ADC. */
#define BMA5_TEMP_MEAS_SRC_MSK                     UINT8_C(0x08)
#define BMA5_TEMP_MEAS_SRC_POS                     UINT8_C(0x03)

/*! Select the external pin as source for temperature ADC. */
#define BMA5_TEMP_EXT_SEL_MSK                      UINT8_C(0x10)
#define BMA5_TEMP_EXT_SEL_POS                      UINT8_C(0x04)

/*! Output enable for INT pin */
#define BMA5_INT_MODE_MSK                          UINT8_C(0x03)
#define BMA5_INT_MODE_POS                          UINT8_C(0x00)

/*! Configure behaviour of INT pin to open drain. */
#define BMA5_INT_OD_MSK                            UINT8_C(0x04)
#define BMA5_INT_OD_POS                            UINT8_C(0x02)

/*! Configure level of INT pin */
#define BMA5_INT_LVL_MSK                           UINT8_C(0x08)
#define BMA5_INT_LVL_POS                           UINT8_C(0x03)

/*! I2C slave address of this device. */
#define BMA5_IF_I2C_SLV_ADDR_MSK                   UINT8_C(0x7F)
#define BMA5_IF_I2C_SLV_ADDR_POS                   UINT8_C(0x00)

/*! Configuration of I3C mode */
#define BMA5_IF_I3C_CFG_MSK                        UINT8_C(0x01)
#define BMA5_IF_I3C_CFG_POS                        UINT8_C(0x00)

/*! Configuration of SPI3 mode(SPI 3 wire protocol). */
#define BMA5_IF_SPI3_CFG_MSK                       UINT8_C(0x02)
#define BMA5_IF_SPI3_CFG_POS                       UINT8_C(0x01)

/*! Configuration of CSB pullup in SPI mode */
#define BMA5_IF_CSB_PULLUP_MSK                     UINT8_C(0x04)
#define BMA5_IF_CSB_PULLUP_POS                     UINT8_C(0x02)

/*! Pad drive strength in I2C mode. */
#define BMA5_IF_PAD_DRV_MSK                        UINT8_C(0x38)
#define BMA5_IF_PAD_DRV_POS                        UINT8_C(0x03)

/*! select drive strength in I2C mode */
#define BMA5_IF_I2C_DRV_SEL_MSK                    UINT8_C(0x40)
#define BMA5_IF_I2C_DRV_SEL_POS                    UINT8_C(0x06)

/*! FIFO reset trigger. Writing '1' to this field synchronously resets the FIFO. */
#define BMA5_FIFO_RST_MSK                          UINT8_C(0x01)
#define BMA5_FIFO_RST_POS                          UINT8_C(0x00)

/*! FIFO frame synchronization trigger. Writing '1' to this field tells the FIFO that another frame is about to be
 * written to FIFO_DATA_IN. */
#define BMA5_FIFO_FRAME_SYNC_MSK                   UINT8_C(0x02)
#define BMA5_FIFO_FRAME_SYNC_POS                   UINT8_C(0x01)

/*! Enable bit for the FIFO. Cannot be set to 1 if fifo_size equals 0. */
#define BMA5_FIFO_CFG_MSK                          UINT8_C(0x01)
#define BMA5_FIFO_CFG_POS                          UINT8_C(0x00)

/*! Configuration bit to enable the storage of the x-axis acceleration data in the FIFO. */
#define BMA5_FIFO_ACC_X_MSK                        UINT8_C(0x02)
#define BMA5_FIFO_ACC_X_POS                        UINT8_C(0x01)

/*! Configuration bit to enable the storage of the y-axis acceleration data in the FIFO. */
#define BMA5_FIFO_ACC_Y_MSK                        UINT8_C(0x04)
#define BMA5_FIFO_ACC_Y_POS                        UINT8_C(0x02)

/*! Configuration bit to enable the storage of the z-axis acceleration data in the FIFO. */
#define BMA5_FIFO_ACC_Z_MSK                        UINT8_C(0x08)
#define BMA5_FIFO_ACC_Z_POS                        UINT8_C(0x03)

/*! Enable bit for FIFO data compression. */
#define BMA5_FIFO_COMPRESSION_MSK                  UINT8_C(0x10)
#define BMA5_FIFO_COMPRESSION_POS                  UINT8_C(0x04)

/*! FIFO size. Since FIFO and feature engine share a common RAM, the size for the FIFO share has to be adjusted. Cannot
 * be changed if locked by the feature engine. In Order to change this value, first disable the feature engine. If the
 * feature engine is turned on again, a minimum share migh be needed and this setting might be changed by the feature
 * engine. */
#define BMA5_FIFO_SIZE_MSK                         UINT8_C(0x03)
#define BMA5_FIFO_SIZE_POS                         UINT8_C(0x00)

/*! FIFO sensor time configuration. */
#define BMA5_FIFO_SENSOR_TIME_MSK                  UINT8_C(0x0C)
#define BMA5_FIFO_SENSOR_TIME_POS                  UINT8_C(0x02)

/*! If set, the FIFO stops storing new data if it is full. Otherwise the oldest frame is dropped in order to make room
 * for a new frame. */
#define BMA5_FIFO_STOP_ON_FULL_MSK                 UINT8_C(0x10)
#define BMA5_FIFO_STOP_ON_FULL_POS                 UINT8_C(0x04)

/*! LSB of the FIFO watermark level. */
#define BMA5_FIFO_WATERMARK_LEVEL_7_0_MSK          UINT8_C(0xFF)
#define BMA5_FIFO_WATERMARK_LEVEL_7_0_POS          UINT8_C(0x00)

/*! LSB of the FIFO watermark level. */
#define BMA5_FIFO_WATERMARK_LEVEL_10_8_MSK         UINT8_C(0x07)
#define BMA5_FIFO_WATERMARK_LEVEL_10_8_POS         UINT8_C(0x00)

/*! An enable/disable switch for the feature engine. The feature engine is internally reseted, once the engine is
 * disabled and the enabled again. */
#define BMA5_FEAT_ENG_CTRL_MSK                     UINT8_C(0x01)
#define BMA5_FEAT_ENG_CTRL_POS                     UINT8_C(0x00)

/*! When this field equals 1, the feature engine is currently halted. This means that the "halt" instruction has been
 * executed and that the processor waits for a wakeup trigger. */
#define BMA5_FEAT_ENG_HALTED_MSK                   UINT8_C(0x01)
#define BMA5_FEAT_ENG_HALTED_POS                   UINT8_C(0x00)

/*! When this field equals 1, the feature engine is currently executing code. */
#define BMA5_FEAT_ENG_RUNNING_MSK                  UINT8_C(0x02)
#define BMA5_FEAT_ENG_RUNNING_POS                  UINT8_C(0x01)

/*!
              This field reads 1’b1 as long as an update of the host-owned GPRs is pending.
             */
#define BMA5_HOST_GPR_UPDATE_PENDING_MSK           UINT8_C(0x04)
#define BMA5_HOST_GPR_UPDATE_PENDING_POS           UINT8_C(0x02)

/*!
              This field reads 1’b1 as long as an update of the feature engine-owned GPRs is pending.
             */
#define BMA5_FEAT_ENG_GPR_UPDATE_PENDING_MSK       UINT8_C(0x08)
#define BMA5_FEAT_ENG_GPR_UPDATE_PENDING_POS       UINT8_C(0x03)

/*! host Direction for GP register 0 ('0': feature engine has write access, '1': host has write access). This field is
 * only writeable by the feature engine. */
#define BMA5_FEAT_ENG_GPR_0_DIR_MSK                UINT8_C(0x01)
#define BMA5_FEAT_ENG_GPR_0_DIR_POS                UINT8_C(0x00)

/*! host Direction for GP register 1 ('0': feature engine has write access, '1': host has write access). This field is
 * only writeable by the feature engine. */
#define BMA5_FEAT_ENG_GPR_1_DIR_MSK                UINT8_C(0x02)
#define BMA5_FEAT_ENG_GPR_1_DIR_POS                UINT8_C(0x01)

/*! host Direction for GP register 2 ('0': feature engine has write access, '1': host has write access). This field is
 * only writeable by the feature engine. */
#define BMA5_FEAT_ENG_GPR_2_DIR_MSK                UINT8_C(0x04)
#define BMA5_FEAT_ENG_GPR_2_DIR_POS                UINT8_C(0x02)

/*! host direction for GP register 3 ('0': feature engine has write access, '1': host has write access). This field is
 * only writeable by the feature engine. */
#define BMA5_FEAT_ENG_GPR_3_DIR_MSK                UINT8_C(0x08)
#define BMA5_FEAT_ENG_GPR_3_DIR_POS                UINT8_C(0x03)

/*! host direction for GP register 4 ('0': feature engine has write access, '1': host has write access). This field is
 * only writeable by the feature engine. */
#define BMA5_FEAT_ENG_GPR_4_DIR_MSK                UINT8_C(0x10)
#define BMA5_FEAT_ENG_GPR_4_DIR_POS                UINT8_C(0x04)

/*! host direction for GP register 5 ('0': feature engine has write access, '1': host has write access). This field is
 * only writeable by the feature engine. */
#define BMA5_FEAT_ENG_GPR_5_DIR_MSK                UINT8_C(0x20)
#define BMA5_FEAT_ENG_GPR_5_DIR_POS                UINT8_C(0x05)

/*! host direction for GP register 6 ('0': feature engine has write access, '1': host has write access). This field is
 * only writeable by the feature engine. */
#define BMA5_FEAT_ENG_GPR_6_DIR_MSK                UINT8_C(0x40)
#define BMA5_FEAT_ENG_GPR_6_DIR_POS                UINT8_C(0x06)

/*! If the host writes 1’b1 to this field, it requests that the host-owned first stage registers
 * are copied to the host-owned second stage registers. If the feature engine writes 1’b1 to this field,
 * it requests that the feature engine-owned first stage registers are copied to the feature engine-owned
 * second stage registers. */
#define BMA5_UPDATE_GPRS_MSK                       UINT8_C(0x01)
#define BMA5_UPDATE_GPRS_POS                       UINT8_C(0x00)

/*! If the host writes 1’b1 to this field, it releases the lock of the feature engine-owned GPRs
 * and thus allows for an update of the feature engine-owned second stage registers.
 * If the feature engine writes 1’b1 to this field, it releases the lock of the host-owned GPRs
 * and thus allows for an update of the host-owned second stage registers. */
#define BMA5_UNLOCK_GPRS_MSK                       UINT8_C(0x02)
#define BMA5_UNLOCK_GPRS_POS                       UINT8_C(0x01)

/*! Feature data address. For the address values see the extended memory map. */
#define BMA5_FEATURE_DATA_ADDR_MSK                 UINT8_C(0x7F)
#define BMA5_FEATURE_DATA_ADDR_POS                 UINT8_C(0x00)

/*! The data port associated with feature_data_addr. During burst read/write operations on this address the address
 * increment stops and the burst operation can be used to read/write multiple feature_data words. See the extendend
 * memory map for details. */
#define BMA5_FEATURE_DATA_MSK                      UINT8_C(0xFF)
#define BMA5_FEATURE_DATA_POS                      UINT8_C(0x00)

/*! Accelerometer offset correction values */
#define BMA5_ACC_DOFF_7_0_MSK                      UINT8_C(0xFF)
#define BMA5_ACC_DOFF_8_MSK                        UINT8_C(0x01)

/*! Accelerometer offset correction value for x-axis */
#define BMA5_ACC_DOFF_X_7_0_MSK                    UINT8_C(0xFF)
#define BMA5_ACC_DOFF_X_7_0_POS                    UINT8_C(0x00)

/*! Accelerometer offset correction value for x-axis */
#define BMA5_ACC_DOFF_X_8_MSK                      UINT8_C(0x01)
#define BMA5_ACC_DOFF_X_8_POS                      UINT8_C(0x00)

/*! Accelerometer offset correction value for y-axis */
#define BMA5_ACC_DOFF_Y_7_0_MSK                    UINT8_C(0xFF)
#define BMA5_ACC_DOFF_Y_7_0_POS                    UINT8_C(0x00)

/*! Accelerometer offset correction value for y-axis */
#define BMA5_ACC_DOFF_Y_8_MSK                      UINT8_C(0x01)
#define BMA5_ACC_DOFF_Y_8_POS                      UINT8_C(0x00)

/*! Accelerometer offset correction value for z-axis */
#define BMA5_ACC_DOFF_Z_7_0_MSK                    UINT8_C(0xFF)
#define BMA5_ACC_DOFF_Z_7_0_POS                    UINT8_C(0x00)

/*! Accelerometer offset correction value for z-axis */
#define BMA5_ACC_DOFF_Z_8_MSK                      UINT8_C(0x01)
#define BMA5_ACC_DOFF_Z_8_POS                      UINT8_C(0x00)

/*! Enable flag for the self test mode. */
#define BMA5_SELF_TEST_MSK                         UINT8_C(0x01)
#define BMA5_SELF_TEST_POS                         UINT8_C(0x00)

/*! Select sign of self test excitation */
#define BMA5_SELF_TEST_SIGN_MSK                    UINT8_C(0x02)
#define BMA5_SELF_TEST_SIGN_POS                    UINT8_C(0x01)

/*! Available commands (Note: Register will always read as 0x00): */
#define BMA5_CMD_MSK                               UINT8_C(0xFF)
#define BMA5_CMD_POS                               UINT8_C(0x00)

/******************************************************************************/
/********************* Macros for supported field values **********************/
/******************************************************************************/
/* Macros to define the supported suspend values */
#define BMA5_SUSPEND_DISABLE                       UINT8_C(0x00) /*! Suspend mode is disabled. Sensor in normal operation
                                                                 * mode. */
#define BMA5_SUSPEND_ENABLE                        UINT8_C(0x01) /*! Suspend mode is enabled. Only Register CHIP_ID and
                                                                 * this register are accessible. */

/* Macros to define the supported feat_eng_err values */
#define BMA5_FEAT_ENG_ERR_OKAY                     UINT8_C(0x00)        /*! feature engine is okay */
#define BMA5_FEAT_ENG_ERR_ERR                      UINT8_C(0x01)        /*! feature engine indicates error */

/* Macros to define the supported acc_conf_err values */
#define BMA5_ACC_CONF_ERR_OKAY                     UINT8_C(0x00)        /*! sensor configuration okay */
#define BMA5_ACC_CONF_ERR_ERR                      UINT8_C(0x01)        /*! sensor configuration invalid */

/* Macros to define the supported sensor_ctrl values */
#define BMA5_SENSOR_CTRL_DISABLE                   UINT8_C(0x00)        /*! The accelerometer and the temperature sensor
                                                                         * are disabled. */
#define BMA5_SENSOR_CTRL_ENABLE                    UINT8_C(0x0F)        /*! The accelerometer and the temperature sensor
                                                                         * are enabled. */
#define BMA5_SENSOR_CTRL_DIS_ERR                   UINT8_C(0x0E)        /*! A wrong configuration was found: The
                                                                         * accelerometer and the temperature sensor are
                                                                         * disabled. */

/* Macros to define the supported acc_odr values */
#define BMA5_ACC_ODR_HZ_1P5625                     UINT8_C(0x00)        /*! 1.5625 Hz. Only available in duty cycling
                                                                         * mode (LPM). */
#define BMA5_ACC_ODR_HZ_3P125                      UINT8_C(0x01)        /*! 3.125 Hz. Only available in duty cycling
                                                                         * mode (LPM). */
#define BMA5_ACC_ODR_HZ_6P25                       UINT8_C(0x02)        /*! 6.25 Hz.Only available in duty cycling mode
                                                                         * (LPM). */
#define BMA5_ACC_ODR_HZ_12P5                       UINT8_C(0x03)        /*! 12.5 Hz. */
#define BMA5_ACC_ODR_HZ_25                         UINT8_C(0x04)        /*! 25 Hz. */
#define BMA5_ACC_ODR_HZ_50                         UINT8_C(0x05)        /*! 50 Hz. */
#define BMA5_ACC_ODR_HZ_100                        UINT8_C(0x06)        /*! 100 Hz. */
#define BMA5_ACC_ODR_HZ_200                        UINT8_C(0x07)        /*! 200 Hz. */
#define BMA5_ACC_ODR_HZ_400                        UINT8_C(0x08)        /*! 400 Hz. */
#define BMA5_ACC_ODR_HZ_800                        UINT8_C(0x09)        /*! 800 Hz. Only available in continuous mode
                                                                         * (HPM). */
#define BMA5_ACC_ODR_HZ_1K6                        UINT8_C(0x0A)        /*! 1.6 kHz. Only available in continuous mode
                                                                         * (HPM). */
#define BMA5_ACC_ODR_HZ_3K2                        UINT8_C(0x0B)        /*! 3.2 kHz. Only available in continuous mode
                                                                         * (HPM). */
#define BMA5_ACC_ODR_HZ_6K4                        UINT8_C(0x0C)        /*! 6.4 kHz. Only available in continuous mode
                                                                         * (HPM). */

/* Macros to define the supported acc_bwp values */
#define BMA5_ACC_BWP_OSR4_AVG1                     UINT8_C(0x00)        /*! HPM --> OSR4 mode; LPM --> no averaging. */
#define BMA5_ACC_BWP_OSR2_AVG2                     UINT8_C(0x01)        /*! HPM --> OSR2 mode; LPM --> average 2
                                                                         * samples. */
#define BMA5_ACC_BWP_NORM_AVG4                     UINT8_C(0x02)        /*! HPM --> normal mode; LPM --> average 4
                                                                         * samples. */
#define BMA5_ACC_BWP_CIC_AVG8                      UINT8_C(0x03)        /*! HPM --> CIC mode; LPM --> average 8 samples.
                                                                         * */

/* Macros to define the supported power_mode values */
#define BMA5_POWER_MODE_LPM                        UINT8_C(0x00)        /*! LPM: Low power mode (Duty Cycling mode) */
#define BMA5_POWER_MODE_HPM                        UINT8_C(0x01)        /*! HPM: High Performance Mode (Continuous mode)
                                                                         * */

/* Macros to define the supported acc_range values */
#define BMA5_ACC_RANGE_MAX_2G                      UINT8_C(0x00)        /*! measurement range: +/-2g. */
#define BMA5_ACC_RANGE_MAX_4G                      UINT8_C(0x01)        /*! measurement range: +/-4g. */
#define BMA5_ACC_RANGE_MAX_8G                      UINT8_C(0x02)        /*! measurement range: +/-8g. */
#define BMA5_ACC_RANGE_MAX_16G                     UINT8_C(0x03)        /*! measurement range: +/-16g. */

/* Macros to define the supported acc_iir_ro values */
#define BMA5_ACC_IIR_RO_DB_20                      UINT8_C(0x01)        /*! -20dB roll-off */
#define BMA5_ACC_IIR_RO_DB_40                      UINT8_C(0x02)        /*! -40dB roll-off */
#define BMA5_ACC_IIR_RO_DB_60                      UINT8_C(0x03)        /*! -60dB roll-off */

/* Macros to define the supported noise_mode values */
#define BMA5_NOISE_MODE_LOWER_NOISE                UINT8_C(0x00)        /*! Default config. Lower noise level. */
#define BMA5_NOISE_MODE_LOWER_POWER                UINT8_C(0x01)        /*! Lower power consumption. Higher noise level.
                                                                         * This setting should only be used in HPM mode!
                                                                         * */

/* Macros to define the supported acc_drdy_int_auto_clear values */
#define BMA5_ACC_DRDY_INT_AUTO_CLEAR_DISABLED      UINT8_C(0x00)        /*! The status flag of acc_drdy_int is not
                                                                         * cleared automatically. */
#define BMA5_ACC_DRDY_INT_AUTO_CLEAR_ENABLED       UINT8_C(0x01)        /*! The status flag of acc_drdy_int is cleared
                                                                         * automatically after approximately 1/(2*ODR).
                                                                         * */

/* Macros to define the supported temp_rate values */
#define BMA5_TEMP_RATE_HZ_1P5625                   UINT8_C(0x00)        /*! Sample temperature at 1.5625 Hz. */
#define BMA5_TEMP_RATE_HZ_3P125                    UINT8_C(0x01)        /*! Sample temperature at 3.125 Hz. */
#define BMA5_TEMP_RATE_HZ_6P25                     UINT8_C(0x02)        /*! Sample temperature at 6.25 Hz. */
#define BMA5_TEMP_RATE_HZ_12P5                     UINT8_C(0x03)        /*! Sample temperature at 12.5 Hz. */
#define BMA5_TEMP_RATE_HZ_25                       UINT8_C(0x04)        /*! Sample temperature at 25 Hz. */
#define BMA5_TEMP_RATE_HZ_50                       UINT8_C(0x05)        /*! Sample temperature at 50 Hz. */
#define BMA5_TEMP_RATE_HZ_100                      UINT8_C(0x06)        /*! Sample temperature at 100 Hz. */
#define BMA5_TEMP_RATE_HZ_200                      UINT8_C(0x07)        /*! Sample temperature at 200 Hz. */

/* Macros to define the supported temp_meas_src values */
#define BMA5_TEMP_MEAS_SRC_TMP_INT                 UINT8_C(0x00)        /*! internal temperature diode */
#define BMA5_TEMP_MEAS_SRC_TMP_EXT                 UINT8_C(0x01)        /*! external input as configured by tmp_pin_sel
                                                                         * */

/* Macros to define the supported temp_ext_sel values */
#define BMA5_TEMP_EXT_SEL_INT1                     UINT8_C(0x00)        /*! interrupt pin INT1 */
#define BMA5_TEMP_EXT_SEL_INT2                     UINT8_C(0x01)        /*! interrupt pin INT2 */

/* Macros to define the supported int1_mode values */
#define BMA5_INT1_MODE_OFF                         UINT8_C(0x00)        /*! Output disabled. */
#define BMA5_INT1_MODE_LATCHED                     UINT8_C(0x01)        /*! Latched (level triggered) interrupts. */
#define BMA5_INT1_MODE_PULSED_SHORT                UINT8_C(0x02)        /*! Pulsed (edge triggered) interrupts with
                                                                         *
                                                                         *
                                                                         *
                                                                         *
                                                                         *
                                                                         *
                                                                         *                                            *
                                                                         * short pulses. */
#define BMA5_INT1_MODE_PULSED_LONG                 UINT8_C(0x03)        /*! Pulsed (edge triggered) interrupts with long
                                                                         * pulses. */

/* Macros to define the supported int1_od values */
#define BMA5_INT1_OD_PUSH_PULL                     UINT8_C(0x00)        /*! push-pull */
#define BMA5_INT1_OD_OPEN_DRAIN                    UINT8_C(0x01)        /*! open drain */

/* Macros to define the supported int1_lvl values */
#define BMA5_INT1_LVL_ACTIVE_LOW                   UINT8_C(0x00)        /*! active low */
#define BMA5_INT1_LVL_ACTIVE_HIGH                  UINT8_C(0x01)        /*! active high */

/* Macros to define the supported int2_mode values */
#define BMA5_INT2_MODE_OFF                         UINT8_C(0x00)        /*! Output disabled. */
#define BMA5_INT2_MODE_LATCHED                     UINT8_C(0x01)        /*! Latched (level triggered) interrupts. */
#define BMA5_INT2_MODE_PULSED_SHORT                UINT8_C(0x02)        /*! Pulsed (edge triggered) interrupts with
                                                                         * short pulses. */
#define BMA5_INT2_MODE_PULSED_LONG                 UINT8_C(0x03)        /*! Pulsed (edge triggered) interrupts with long
                                                                         * pulses. */

/* Macros to define the supported int2_od values */
#define BMA5_INT2_OD_PUSH_PULL                     UINT8_C(0x00)        /*! push-pull */
#define BMA5_INT2_OD_OPEN_DRAIN                    UINT8_C(0x01)        /*! open drain */

/* Macros to define the supported int2_lvl values */
#define BMA5_INT2_LVL_ACTIVE_LOW                   UINT8_C(0x00)        /*! active low */
#define BMA5_INT2_LVL_ACTIVE_HIGH                  UINT8_C(0x01)        /*! active high */

/* Macros to define the supported if_i2c_slv_addr values */
#define BMA5_IF_I2C_SLV_ADDR_I2C_DEFAULT_ADDR      UINT8_C(0x18)        /*! the default i2c slave address of this device
                                                                         * */

/* Macros to define the supported if_i3c_cfg values */
#define BMA5_IF_I3C_CFG_DISABLE                    UINT8_C(0x00)        /*! The I3C mode is disabled. */
#define BMA5_IF_I3C_CFG_ENABLE                     UINT8_C(0x01)        /*! The I3C mode is enabled. */

/* Macros to define the supported if_spi3_cfg values */
#define BMA5_IF_SPI3_CFG_DISABLE                   UINT8_C(0x00)        /*! The SPI3 mode is disabled. */
#define BMA5_IF_SPI3_CFG_ENABLE                    UINT8_C(0x01)        /*! The SPI3 mode is enabled. */

/* Macros to define the supported if_csb_pullup values */
#define BMA5_IF_CSB_PULLUP_DISABLE                 UINT8_C(0x00)        /*! The pullup is disabled. */
#define BMA5_IF_CSB_PULLUP_ENABLE                  UINT8_C(0x01)        /*! The pullup is enabled. */

/* Macros to define the supported if_i2c_drv_sel values */
#define BMA5_IF_I2C_DRV_SEL_I2C_MAX_PAD_DRV        UINT8_C(0x00)        /*! use maximum pad drive strength */
#define BMA5_IF_I2C_DRV_SEL_I2C_CFG_PAD_DRV        UINT8_C(0x01)        /*! use drive strength settings of if_pad_drv */

/* Macros to define the supported fifo_cfg values */
#define BMA5_FIFO_CFG_DISABLE                      UINT8_C(0x00)        /*! The FIFO is disabled. */
#define BMA5_FIFO_CFG_ENABLE                       UINT8_C(0x01)        /*! The FIFO is enabled. */

/* Macros to define the supported fifo_acc_x values */
#define BMA5_FIFO_ACC_X_DISABLE                    UINT8_C(0x00)        /*! The FIFO x-axis acceleration channel is
                                                                         * disabled. */
#define BMA5_FIFO_ACC_X_ENABLE                     UINT8_C(0x01)        /*! The FIFO x-axis acceleration channel is
                                                                         * disabled. */

/* Macros to define the supported fifo_acc_y values */
#define BMA5_FIFO_ACC_Y_DISABLE                    UINT8_C(0x00)        /*! The FIFO y-axis acceleration channel is
                                                                         * disabled. */
#define BMA5_FIFO_ACC_Y_ENABLE                     UINT8_C(0x01)        /*! The FIFO y-axis acceleration channel is
                                                                         * disabled. */

/* Macros to define the supported fifo_acc_z values */
#define BMA5_FIFO_ACC_Z_DISABLE                    UINT8_C(0x00)        /*! The FIFO z-axis acceleration channel is
                                                                         * disabled. */
#define BMA5_FIFO_ACC_Z_ENABLE                     UINT8_C(0x01)        /*! The FIFO z-axis acceleration channel is
                                                                         * disabled. */

/* Macros to define the supported fifo_compression values */
#define BMA5_FIFO_COMPRESSION_ACC_16BIT            UINT8_C(0x00)        /*! compression disbaled. full 16bit
                                                                         * acceleration data. */
#define BMA5_FIFO_COMPRESSION_ACC_8BIT             UINT8_C(0x01)        /*! compression enabled. 8bit compressed
                                                                         * acceleration data. */

/* Macros to define the supported fifo_size values */
#define BMA5_FIFO_SIZE_MAX_0_BYTES                 UINT8_C(0x00)        /*! The FIFO has a size of 0 bytes. The feature
                                                                         * engine ('feat_eng') has a RAM size of 1024
                                                                         * bytes. This setting forces fifo_en to 0. */
#define BMA5_FIFO_SIZE_MAX_256_BYTES               UINT8_C(0x01)        /*! The FIFO has a size of 256 bytes. The
                                                                         * feature engine ('feat_eng') has a RAM size of
                                                                         * 768 bytes. */
#define BMA5_FIFO_SIZE_MAX_512_BYTES               UINT8_C(0x02)        /*! The FIFO has a size of 512 bytes. The
                                                                         * feature engine ('feat_eng') has a RAM size of
                                                                         * 512 bytes. */
#define BMA5_FIFO_SIZE_MAX_1024_BYTES              UINT8_C(0x03)        /*! The FIFO has a size of 1024 bytes. The
                                                                         * feature engine ('feat_eng') has a RAM size of
                                                                         * 0 bytes. This setting forces feature engine
                                                                         * ('feat_eng')_en to 0. */

/* Macros to define the supported fifo_sensor_time values */
#define BMA5_FIFO_SENSOR_TIME_OFF                  UINT8_C(0x00)        /*! The FIFO does not transmit the sensor time.
                                                                         * */
#define BMA5_FIFO_SENSOR_TIME_DEDICATED_FRAME      UINT8_C(0x01)        /*! The FIFO sends a dedicated sensor time frame
                                                                         * when the FIFO runs empty during a read burst.
                                                                         * */
#define BMA5_FIFO_SENSOR_TIME_EACH_FRAME           UINT8_C(0x02)        /*! The FIFO has appends the sensor time to each
                                                                         * frame. */

/* Macros to define the supported fifo_stop_on_full values */
#define BMA5_FIFO_STOP_ON_FULL_DISABLED            UINT8_C(0x00)        /*! feature disbaled.The FIFO will be filled
                                                                         * continously with new data, old data will be
                                                                         * dropped. */
#define BMA5_FIFO_STOP_ON_FULL_ENABLED             UINT8_C(0x01)        /*! feature enabled. The FIFO will stop, when it
                                                                         * is full. */

/* Macros to define the supported feat_eng_ctrl values */
#define BMA5_FEAT_ENG_CTRL_DISABLE                 UINT8_C(0x00)        /*! the feature engine is disabled (and reset)
                                                                         * */
#define BMA5_FEAT_ENG_CTRL_ENABLE                  UINT8_C(0x01)        /*! the feature engine is enabled. */

/* Macros to define the supported self_test values */
#define BMA5_SELF_TEST_DISABLE                     UINT8_C(0x00)        /*! normal operation mode */
#define BMA5_SELF_TEST_ENABLE                      UINT8_C(0x01)        /*! built-in test excitation mode */

/* Macros to define the supported self_test_sign values */
#define BMA5_SELF_TEST_SIGN_NEGATIVE               UINT8_C(0x00)        /*! negative */
#define BMA5_SELF_TEST_SIGN_POSITIVE               UINT8_C(0x01)        /*! positive */

/* Macros to define the supported cmd values */
#define BMA5_CMD_SOFTRESET                         UINT8_C(0xB6)        /*! Triggers a reset, all user configuration
                                                                         * settings are overwritten with their default
                                                                         * state. If this register is set using I2C, an
                                                                         * ACK will NOT be transmitted to the host */

/* Macros to define different interrupts */
#define BMA5_INT_1                                 UINT8_C(0)
#define BMA5_INT_2                                 UINT8_C(1)

#define BMA5_16_BIT_RESOLUTION                     UINT8_C(16)
#define BMA5_8_BIT_RESOLUTION                      UINT8_C(8)

#define BMA5_FIFO_8_BIT_ACCEL_XYZ_DATA             UINT8_C(3)
#define BMA5_FIFO_8_BIT_ACCEL_XYZ_SENS_TIME_DATA   UINT8_C(6)
#define BMA5_FIFO_16_BIT_ACCEL_XYZ_DATA            UINT8_C(6)
#define BMA5_FIFO_16_BIT_ACCEL_XYZ_SENS_TIME_DATA  UINT8_C(9)

/*! FIFO header byte mask */
#define BMA5_FIFO_SENS_TIME_EN_HDR_BYTE_MSK        UINT8_C(0x01)

/*! FIFO sensor time length definitions */
#define BMA5_SENSOR_TIME_LENGTH                    UINT8_C(3)

/*! FIFO warning macros */
#define BMA5_W_FIFO_EMPTY                          UINT8_C(1)
#define BMA5_W_FIFO_PARTIAL_READ                   UINT8_C(2)

#define BMA5_FIFO_SENS_TIME_EN_DEDI_FRM            UINT8_C(0xA1)
#define BMA5_FIFO_SENS_TIME_EN_EACH_FRM            UINT8_C(0xC1)

#define BMA5_FIFO_FRAME_HDR_MSK                    UINT8_C(0xFF)

/*! FIFO sensortime overhead byte macros
 *  NOTE : Only payload is stored in the FIFO memory.
 *  The header is always generated on demand when the FIFO is read.
 *  Overhead byte is added to accommodate various FIFO combinations.
 */
#define BMA5_SENSORTIME_OVERHEAD_BYTE              UINT16_C(1024)

/*! Enable/Disable macros */
#define BMA5_ENABLE                                UINT8_C(1)
#define BMA5_DISABLE                               UINT8_C(0)

/*! Self-test macros */
#define BMA5_SELF_TEST_DELAY                       UINT16_C(10000)
#define BMA5_SELF_TEST_ACCEL_DISABLE_DELAY         UINT16_C(50000)
#define BMA5_SELF_TEST_MIN_THRESHOLD_X             UINT16_C(17500)
#define BMA5_SELF_TEST_MIN_THRESHOLD_Y             UINT16_C(17500)
#define BMA5_SELF_TEST_MIN_THRESHOLD_Z             UINT16_C(8000)

/*! Accel foc axis 1G macros */
#define BMA5_ACC_FOC_AXIS_Z_PLUS_1G                UINT8_C(0)
#define BMA5_ACC_FOC_AXIS_Z_MINUS_1G               UINT8_C(1)
#define BMA5_ACC_FOC_AXIS_Y_PLUS_1G                UINT8_C(2)
#define BMA5_ACC_FOC_AXIS_Y_MINUS_1G               UINT8_C(3)
#define BMA5_ACC_FOC_AXIS_X_PLUS_1G                UINT8_C(4)
#define BMA5_ACC_FOC_AXIS_X_MINUS_1G               UINT8_C(5)
#define BMA5_AXIS_SEL_X                            UINT8_C(0x01)
#define BMA5_AXIS_SEL_Y                            UINT8_C(0x02)
#define BMA5_AXIS_SEL_Z                            UINT8_C(0x04)
#define BMA5_AXIS_SEL_XYZ                          UINT8_C(0x07)

#define BMA5_FIFO_RST_TRIGGER_ENABLE               UINT8_C(0x01)
#define BMA5_FIFO_RST_TRIGGER_DISABLE              UINT8_C(0x00)

/*! @name For defining absolute values */
#define BMA5_ABS(a)                                ((a) > 0 ? (a) : -(a))

/******************************************************************************/
/***************************** Function pointers ******************************/
/******************************************************************************/

/*!
 * @brief Pointer to be mapped to the platform specific bus read function of the
 *        user
 *
 * @param[in]     reg_addr : Address of the register which needs to be read
 * @param[out]    reg_data : Buffer to carry the data from the specified address
 * @param[in]     length   : Length of the data to be read in bytes
 * @param[in,out] intf_ptr : Pointer that can enable the linking of descriptors
 *                           for interface related callbacks
 *
 * @retval Zero for success
 * @retval Non-zero for failure
 */
typedef BMA5_INTF_RET_TYPE (*bma5_read_fptr_t)(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);

/*!
 * @brief Pointer to be mapped to the platform specific bus write function of
 *        the user
 *
 * @param[in]     reg_addr : Address of the register which needs to be written
 * @param[in]     reg_data : Data to be written to the specified address
 * @param[in]     length   : Number of bytes to be written to the address
 * @param[in,out] intf_ptr : Pointer that can enable the linking of descriptors
 *                           for interface related callbacks
 *
 * @retval Zero for success
 * @retval Non-zero for failure
 */
typedef BMA5_INTF_RET_TYPE (*bma5_write_fptr_t)(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length,
                                                void *intf_ptr);

/*!
 * @brief Pointer to be mapped to the platform specific delay function of user
 *
 * @param[in]     period   : Time period in microseconds
 * @param[in,out] intf_ptr : Pointer that can enable the linking of descriptors
 *                           for interface related callbacks
 */
typedef void (*bma5_delay_us_fptr_t)(uint32_t period, void *intf_ptr);

/******************************************************************************/
/******************************** Enumerators *********************************/
/******************************************************************************/

/*!
 * Enumerator to describe the interfaces
 */
enum bma5_intf {
    /*! SPI interface */
    BMA5_SPI_INTF = 0,
    /*! I2C interface */
    BMA5_I2C_INTF
};

/*!
 * Enumerator to describe the interfaces
 */
enum bma5_context {
    /*! Hearable */
    BMA5_HEARABLE = 0,
    /*! Wearable */
    BMA5_WEARABLE,
    /*! Smartphone */
    BMA5_SMARTPHONE
};

/******************************************************************************/
/************************** Primary device structure **************************/
/******************************************************************************/

/*
 * @brief Primary device handle
 */
struct bma5_dev
{
    /*! Chip identification code for BMA5 */
    uint8_t chip_id;

    /*! SPI/I2C interface selection */
    enum bma5_intf intf;

    /*! Platform specific function to perform platform specific read operation
     *  corresponding to the selected interface */
    bma5_read_fptr_t bus_read;

    /*! Platform specific function to perform platform specific write operation
     *  corresponding to the selected interface */
    bma5_write_fptr_t bus_write;

    /*! Platform specific function to perform platform specific delay operation
     *  in microseconds */
    bma5_delay_us_fptr_t delay_us;

    /*! Pointer to enable the user to link their interface descriptors for
     *  reference during the implementation of the read and write interfaces to
     *  the hardware.
     */
    void *intf_ptr;

    /*! The underlying interface operation results */
    BMA5_INTF_RET_TYPE intf_rslt;

    /*! The number of dummy bytes expected over serial communication interface.
     *  This field can further be used for specifying the dummy bytes expected
     *  over the serial communication with the auxillary as well.
     */
    uint8_t dummy_byte;

    /*! Context parameter selection */
    enum bma5_context context;
};

/******************************************************************************/
/***************** Structures for handling register content *******************/
/******************************************************************************/

/*!
 * @brief Structure holding Global error flags
 */
struct bma5_config_status
{
    /*! Set by feature engine in case of feature engine error condition. Needs to be reseted by the host. For more
     * details there are further status register in the feature engine section and inside the DMA region. */
    uint8_t feat_eng_err;

    /*! This flag is set if the ACC configuration in ACC_CONF_0, ACC_CONF_1, and ACC_CONF_2 is an invalid combination.
     * */
    uint8_t acc_conf_err;

};

/*!
 * @brief Structure holding Global status flags
 */
struct bma5_sensor_status
{
    /*! Set when new ACC data is available. This flag can be cleared by writing '1' to it. */
    uint8_t acc_data_rdy;

    /*! Set when new temperature data is available. This flag can be cleared by writing '1' to it. */
    uint8_t temperature_rdy;

    /*! Sensor is ready for operation. */
    uint8_t sensor_rdy;

};

/*!
 * @brief Structure holding Accelerometer configuration register
 */
struct bma5_acc_conf
{
    /*! The ODR (Output Data Rate) in Hz. Not all settings are available in all power modes */
    uint8_t acc_odr;

    /*! Accelerometer bandwith parameter. This parameter determines the filter configuration. The different settings
     * have a different impact depending on the setting of the power_mode bit. The name of the settings are therefore
     * (HPM-setting)_(LPM-setting). (e.g. norm_avg4 means norm mode for HPM and avg4 for LPM) */
    uint8_t acc_bwp;

    /*! With this config bit, it is possible to set the basic measurement power mode. There are two possible
     * settings:LPM (Low Power Mode) with duty cycling or HPM (High Performance Mode) with continous measurement. This
     * setting has an influence on the signal path and the filter settings, too. */
    uint8_t power_mode;

    /*! The measurement range of the accelerometer. This setting has influence on the scaling of the ACC_DATA registers.
     * */
    uint8_t acc_range;

    /*! Select roll-off of IIR filter in continuous mode. */
    uint8_t acc_iir_ro;

    /*! Select the performance mode of the sensor. The choice is between high performance with lower noise or reduce the
     * power consumption but with an increased noise level. The default is the high performance (lower noise). Changing
     * this setting from default migh also influence the sensor behaviour like offset. This configuration should only be
     * used in HPM.  */
    uint8_t noise_mode;

    /*! Configuration bit to enable/disable the auto clear mechanism of the data ready interrupt. If enabled, a clock
     * like with freq=odr can be enabled on the external interrupt pin. */
    uint8_t acc_drdy_int_auto_clear;
};

/*!
 * @brief Structure holding Temperature Sensor configuration register
 */
struct bma5_temp_conf
{
    /*! Select rate in Hz at which the temperature is sampled. */
    uint8_t temp_rate;

    /*! Select the input source for the temperature ADC. */
    uint8_t temp_meas_src;

    /*! Select the external pin as source for temperature ADC. */
    uint8_t temp_ext_sel;

};

/*!
 * @brief Structure holding Configuration register for INT
 */
struct bma5_int_conf
{
    /*! Output enable for INT1 pin */
    uint8_t int_mode;

    /*! Configure behaviour of INT1 pin to open drain. */
    uint8_t int_od;

    /*! Configure level of INT1 pin */
    uint8_t int_lvl;
};

/*!
 * @brief Structure to define interrupt source and its configuration
 */
struct bma5_int_conf_types
{
    /*! Interrupt source */
    uint8_t int_src;

    /*! Interrupt configuration */
    struct bma5_int_conf int_conf;
};

/*!
 * @brief Structure holding Serial interface settings
 */
struct bma5_if_conf
{
    /*! Configuration of I3C mode */
    uint8_t if_i3c_cfg;

    /*! Configuration of SPI3 mode(SPI 3 wire protocol). */
    uint8_t if_spi3_cfg;

    /*! Configuration of CSB pullup in SPI mode */
    uint8_t if_csb_pullup;

    /*! Pad drive strength in I2C mode. */
    uint8_t if_pad_drv;

    /*! select drive strength in I2C mode */
    uint8_t if_i2c_drv_sel;

};

/*!
 * @brief Structure holding FIFO control register
 */
struct bma5_fifo_ctrl
{
    /*! FIFO reset trigger. Writing '1' to this field synchronously resets the FIFO. */
    uint8_t fifo_rst;

    /*! FIFO frame synchronization trigger. Writing '1' to this field tells the FIFO that another frame is about to be
     * written to FIFO_DATA_IN. */
    uint8_t fifo_frame_sync;

};

/*!
 * @brief Structure holding FIFO configuration register 0
 */
struct bma5_fifo_conf
{
    /*! Enable bit for the FIFO. Cannot be set to 1 if fifo_size equals 0. */
    uint8_t fifo_cfg;

    /*! Configuration bit to enable the storage of the x-axis acceleration data in the FIFO. */
    uint8_t fifo_acc_x;

    /*! Configuration bit to enable the storage of the y-axis acceleration data in the FIFO. */
    uint8_t fifo_acc_y;

    /*! Configuration bit to enable the storage of the z-axis acceleration data in the FIFO. */
    uint8_t fifo_acc_z;

    /*! Enable bit for FIFO data compression. */
    uint8_t fifo_compression;

    /*! FIFO size. Since FIFO and feature engine share a common RAM, the size for the FIFO share has to be adjusted.
     * Cannot be changed if locked by the feature engine. In Order to change this value, first disable the feature
     * engine. If the feature engine is turned on again, a minimum share migh be needed and this setting might be
     * changed by the feature engine. */
    uint8_t fifo_size;

    /*! FIFO sensor time configuration. */
    uint8_t fifo_sensor_time;

    /*! If set, the FIFO stops storing new data if it is full. Otherwise the oldest frame is dropped in order to make
     * room for a new frame. */
    uint8_t fifo_stop_on_full;
};

/*! @brief Structure to define accelerometer sensor axes data in 16 bit
 */
struct bma5_sens_fifo_axes_data_16_bit
{
    /*! Data in x-axis */
    int16_t x;

    /*! Data in y-axis */
    int16_t y;

    /*! Data in z-axis */
    int16_t z;

    /*! Sensortime */
    uint32_t sensor_time;
};

/*! @brief Structure to define accelerometer sensor axes data in 8 bit
 */
struct bma5_sens_fifo_axes_data_8_bit
{
    /*! Data in x-axis */
    int8_t x;

    /*! Data in y-axis */
    int8_t y;

    /*! Data in z-axis */
    int8_t z;

    /*! Sensortime */
    uint32_t sensor_time;
};

/*!
 *  @brief This structure holds the information for usage of
 *  FIFO by the user.
 */
struct bma5_fifo_frame
{
    /*! Data buffer of user defined length is to be mapped here */
    uint8_t *data;

    /*! Stores available FIFO length */
    uint16_t fifo_avail_len;

    /*! Stores available FIFO frames */
    uint16_t fifo_avail_frames;

    /*! To index accelerometer bytes */
    uint16_t acc_byte_start_idx;
};

/*!
 * @brief Structure holding feature engine ('feat_eng') status register
 */
struct bma5_feat_eng_status
{
    /*! When this field equals 1, the feature engine is currently halted. This means that the "halt" instruction has
     * been executed and that the processor waits for a wakeup trigger. */
    uint8_t feat_eng_halted;

    /*! When this field equals 1, the feature engine is currently executing code. */
    uint8_t feat_eng_running;

    /*!
              This field reads 1’b1 as long as an update of the host-owned GPRs is pending.
             */
    uint8_t host_gpr_update_pending;

    /*!
              This field reads 1’b1 as long as an update of the feature engine-owned GPRs is pending.
             */
    uint8_t feat_eng_gpr_update_pending;

};

/*!
 * @brief Structure holding feature engine ('feat_eng') general purpose register configuration register
 */
struct bma5_feat_eng_gpr_conf
{
    /*! host Direction for GP register 0 ('0': feature engine has write access, '1': host has write access). This field
     * is only writeable by the feature engine. */
    uint8_t feat_eng_gpr_0_dir;

    /*! host Direction for GP register 1 ('0': feature engine has write access, '1': host has write access). This field
     * is only writeable by the feature engine. */
    uint8_t feat_eng_gpr_1_dir;

    /*! host Direction for GP register 2 ('0': feature engine has write access, '1': host has write access). This field
     * is only writeable by the feature engine. */
    uint8_t feat_eng_gpr_2_dir;

    /*! host direction for GP register 3 ('0': feature engine has write access, '1': host has write access). This field
     * is only writeable by the feature engine. */
    uint8_t feat_eng_gpr_3_dir;

    /*! host direction for GP register 4 ('0': feature engine has write access, '1': host has write access). This field
     * is only writeable by the feature engine. */
    uint8_t feat_eng_gpr_4_dir;

    /*! host direction for GP register 5 ('0': feature engine has write access, '1': host has write access). This field
     * is only writeable by the feature engine. */
    uint8_t feat_eng_gpr_5_dir;

    /*! host direction for GP register 6 ('0': feature engine has write access, '1': host has write access). This field
     * is only writeable by the feature engine. */
    uint8_t feat_eng_gpr_6_dir;

};

/*!
 * @brief Structure holding feature engine ('feat_eng') general purpose register control register
 */
struct bma5_feat_eng_gpr_ctrl
{
    /*! If the host writes 1’b1 to this field, it requests that the host-owned first stage registers
     * are copied to the host-owned second stage registers.
     * If the feature engine writes 1’b1 to this field, it requests that the feature engine-owned
     * first stage registers are copied to the feature engine-owned second stage registers. */
    uint8_t update_gprs;

    /*! If the host writes 1’b1 to this field, it releases the lock of the feature engine-owned GPRs
     * and thus allows for an update of the feature engine-owned second stage registers.
     * If the feature engine writes 1’b1 to this field, it releases the lock of the host-owned GPRs
     * and thus allows for an update of the host-owned second stage registers. */
    uint8_t unlock_gprs;
};

/*!
 * @brief Structure holding Select NORMAL/SELF_TEST mode and test data. If you write to this register, the ACC data path is reset.
 */
struct bma5_acc_self_test
{
    /*! Enable flag for the self test mode. */
    uint8_t self_test;

    /*! Select sign of self test excitation */
    uint8_t self_test_sign;

};

/*!
 * @brief Structures holding accelerometer values
 */
struct bma5_accel
{
    /*! Accel X value. */
    int16_t x;

    /*! Accel Y value. */
    int16_t y;

    /*! Accel Z value. */
    int16_t z;
};

/*!
 * @brief Structures holding accelerometer doff values
 */
struct bma5_accel_doff
{
    /*! Accel X value. */
    int16_t x_doff;

    /*! Accel Y value. */
    int16_t y_doff;

    /*! Accel Z value. */
    int16_t z_doff;
};

#endif /* _BMA5_DEFS_H */
