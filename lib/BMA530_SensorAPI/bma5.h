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
* @file       bma5.h
* @date       2024-07-29
* @version    v4.2.0
*
*/

#ifndef _BMA5_H
#define _BMA5_H

/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************/
/****************************** Header files **********************************/
/******************************************************************************/
#include  "bma5_defs.h"

/******************************************************************************/
/********************** Function prototype declarations ***********************/
/******************************************************************************/

/**
 * \ingroup bma5
 * \defgroup bma5ApiRegs BMA5 Registers
 * @brief Set / Get data from the given register address of the sensor
 */

/*!
 * \ingroup bma5ApiRegs
 * \page bma5_api_bma5_set_regs bma5_set_regs
 * \code
 * int8_t bma5_set_regs(uint8_t addr, const uint8_t *data, uint32_t len, struct bma5_dev *dev);
 * \endcode
 * @details This API writes data to the given register address of bma5
 *          sensor.
 *
 * @param[in] addr : Register address to which the data is written.
 * @param[in] data : Pointer to data buffer in which data to be written is stored.
 * @param[in] len  : No. of bytes of data to be written.
 * @param[in] dev  : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma5_set_regs(uint8_t addr, const uint8_t *data, uint32_t len, struct bma5_dev *dev);

/*!
 * \ingroup bma5ApiRegs
 * \page bma5_api_bma5_get_regs bma5_get_regs
 * \code
 * int8_t bma5_get_regs(uint8_t addr, uint8_t *data, uint32_t len, struct bma5_dev *dev);
 * \endcode
 * @details This API reads the data from the given register address of bma5
 *          sensor.
 *
 * @param[in]  addr : Register address from which data is read.
 * @param[out] data : Pointer to data buffer where read data is stored.
 * @param[in]  len  : No. of bytes of data to be read.
 * @param[in]  dev  : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma5_get_regs(uint8_t addr, uint8_t *data, uint32_t len, struct bma5_dev *dev);

/*!
 * \ingroup bma5ApiRegs
 * \page bma5_api_bma5_get_health_status bma5_get_health_status
 * \code
 * int8_t bma5_get_health_status(uint8_t *sensor_health_status, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to get the This register contains internal health status information
 *
 * @param[out] sensor_health_status : The value 0xF indicate a good internal health state.
 * @param[in]  dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma5_get_health_status(uint8_t *sensor_health_status, struct bma5_dev *dev);

/*!
 * \ingroup bma5ApiRegs
 * \page bma5_api_bma5_get_cmd_suspend bma5_get_cmd_suspend
 * \code
 * int8_t bma5_get_cmd_suspend(uint8_t *suspend, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to get the Command register to activate suspend mode.
 *
 * @param[out] suspend : Write '1' to activate suspend mode. The register content prior to entering this power mode will NOT be lost.
 * @param[in]  dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma5_get_cmd_suspend(uint8_t *suspend, struct bma5_dev *dev);

/*!
 * \ingroup bma5ApiRegs
 * \page bma5_api_bma5_get_cmd_suspend bma5_get_cmd_suspend
 * \code
 * int8_t bma5_set_cmd_suspend(const uint8_t suspend, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to set the Command register to activate suspend mode.
 *
 * @param[in] suspend : Write '1' to activate suspend mode. The register content prior to entering this power mode will NOT be lost.
 * @param[in] dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma5_set_cmd_suspend(const uint8_t suspend, struct bma5_dev *dev);

/*!
 * \ingroup bma5ApiRegs
 * \page bma5_api_bma5_get_config_status bma5_get_config_status
 * \code
 * int8_t bma5_get_config_status(struct bma5_config_status *config, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to get the Global error flags
 *
 * @param[out] config : Structure instance of bma5_config_status
 * @param[in]  dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma5_get_config_status(struct bma5_config_status *config, struct bma5_dev *dev);

/*!
 * \ingroup bma5ApiRegs
 * \page bma5_api_bma5_get_config_status bma5_get_config_status
 * \code
 * int8_t bma5_set_config_status(const struct bma5_config_status *config, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to set the Global error flags
 *
 * @param[in] config : Structure instance of bma5_config_status
 * @param[in] dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma5_set_config_status(const struct bma5_config_status *config, struct bma5_dev *dev);

/*!
 * \ingroup bma5ApiRegs
 * \page bma5_api_bma5_get_sensor_status bma5_get_sensor_status
 * \code
 * int8_t bma5_get_sensor_status(struct bma5_sensor_status *config, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to get the Global status flags
 *
 * @param[out] config : Structure instance of bma5_sensor_status
 * @param[in]  dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma5_get_sensor_status(struct bma5_sensor_status *config, struct bma5_dev *dev);

/*!
 * \ingroup bma5ApiRegs
 * \page bma5_api_bma5_get_sensor_status bma5_get_sensor_status
 * \code
 * int8_t bma5_set_sensor_status(const struct bma5_sensor_status *config, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to set the Global status flags
 *
 * @param[in] config : Structure instance of bma5_sensor_status
 * @param[in] dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma5_set_sensor_status(const struct bma5_sensor_status *config, struct bma5_dev *dev);

/*!
 * \ingroup bma5ApiRegs
 * \page bma5_api_bma5_get_temp_data bma5_get_temp_data
 * \code
 * int8_t bma5_get_temp_data(uint8_t *temp_data, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to get the Temperature data register
 *
 * @param[out] temp_data : Calculated temperature. Resolution: 1 K/LSB. The value 0 represents 23°C.
 * @param[in]  dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma5_get_temp_data(uint8_t *temp_data, struct bma5_dev *dev);

/*!
 * \ingroup bma5ApiRegs
 * \page bma5_api_bma5_get_fifo_data_out bma5_get_fifo_data_out
 * \code
 * int8_t bma5_get_fifo_data_out(uint8_t *fifo_data_out, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to get the FIFO data register
 *
 * @param[out] fifo_data_out : Output of the FIFO. During burst reads on this address
 *                             the address increment stops and the FIFO can be read out with help of the burst read.
 *                             The type of data stored in the FIFO depends on configuration stored in FIFO_CONF_* registers.
 * @param[in]  dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma5_get_fifo_data_out(uint8_t *fifo_data_out, struct bma5_dev *dev);

/*!
 * \ingroup bma5ApiRegs
 * \page bma5_api_bma5_get_acc_conf_0 bma5_get_acc_conf_0
 * \code
 * int8_t bma5_get_acc_conf_0(uint8_t *sensor_ctrl, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to get the Accelerometer configuration register 0
 *
 * @param[out] sensor_ctrl : This bit enables/disables the accelerometer and the temperature sensor.
 * @param[in]  dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma5_get_acc_conf_0(uint8_t *sensor_ctrl, struct bma5_dev *dev);

/*!
 * \ingroup bma5ApiRegs
 * \page bma5_api_bma5_get_acc_conf_0 bma5_get_acc_conf_0
 * \code
 * int8_t bma5_set_acc_conf_0(const uint8_t sensor_ctrl, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to set the Accelerometer configuration register 0
 *
 * @param[in] sensor_ctrl : This bit enables/disables the accelerometer and the temperature sensor.
 * @param[in] dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma5_set_acc_conf_0(const uint8_t sensor_ctrl, struct bma5_dev *dev);

/*!
 * \ingroup bma5ApiRegs
 * \page bma5_api_bma5_get_acc_conf bma5_get_acc_conf
 * \code
 * int8_t bma5_get_acc_conf(struct bma5_acc_conf *config, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to get the Accelerometer configuration register
 *
 * @param[out] config : Structure instance of bma5_acc_conf
 * @param[in]  dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma5_get_acc_conf(struct bma5_acc_conf *config, struct bma5_dev *dev);

/*!
 * \ingroup bma5ApiRegs
 * \page bma5_api_bma5_set_acc_conf bma5_get_acc_conf
 * \code
 * int8_t bma5_set_acc_conf(const struct bma5_acc_conf *config, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to set the Accelerometer configuration register
 *
 * @param[in] config : Structure instance of bma5_acc_conf
 * @param[in] dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma5_set_acc_conf(const struct bma5_acc_conf *config, struct bma5_dev *dev);

/*!
 * \ingroup bma5ApiRegs
 * \page bma5_api_bma5_get_temp_conf bma5_get_temp_conf
 * \code
 * int8_t bma5_get_temp_conf(struct bma5_temp_conf *config, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to get the Temperature Sensor configuration register
 *
 * @param[out] config : Structure instance of bma5_temp_conf
 * @param[in]  dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma5_get_temp_conf(struct bma5_temp_conf *config, struct bma5_dev *dev);

/*!
 * \ingroup bma5ApiRegs
 * \page bma5_api_bma5_get_temp_conf bma5_get_temp_conf
 * \code
 * int8_t bma5_set_temp_conf(const struct bma5_temp_conf *config, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to set the Temperature Sensor configuration register
 *
 * @param[in] config : Structure instance of bma5_temp_conf
 * @param[in] dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma5_set_temp_conf(const struct bma5_temp_conf *config, struct bma5_dev *dev);

/*!
 * \ingroup bma5ApiRegs
 * \page bma5_api_bma5_get_int_conf bma5_get_int_conf
 * \code
 * int8_t bma5_get_int_conf(struct bma5_int_conf_types *config, uint8_t int_configs, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to get the Configuration register for INT2
 *
 * @param[out] config    : Structure instance of bma5_int_conf
 * @param[in]  int_configs : Number of interrupts configurations to get.
 * @param[in]  dev       : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma5_get_int_conf(struct bma5_int_conf_types *config, uint8_t int_configs, struct bma5_dev *dev);

/*!
 * \ingroup bma5ApiRegs
 * \page bma5_api_bma5_get_int_conf bma5_get_int_conf
 * \code
 * int8_t bma5_set_int_conf(const struct bma5_int_conf_types *config, uint8_t int_configs, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to set the Configuration register for INT2
 *
 * @param[in] config : Structure instance of bma5_int_conf
 * @param[in] int_configs : Number of interrupts configurations to set.
 * @param[in] dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma5_set_int_conf(const struct bma5_int_conf_types *config, uint8_t int_configs, struct bma5_dev *dev);

/*!
 * \ingroup bma5ApiRegs
 * \page bma5_api_bma5_get_if_conf_0 bma5_get_if_conf_0
 * \code
 * int8_t bma5_get_if_conf_0(uint8_t *if_i2c_slv_addr, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to get the Serial interface settings
 *
 * @param[out] if_i2c_slv_addr : I2C slave address of this device.
 * @param[in]  dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma5_get_if_conf_0(uint8_t *if_i2c_slv_addr, struct bma5_dev *dev);

/*!
 * \ingroup bma5ApiRegs
 * \page bma5_api_bma5_get_if_conf bma5_get_if_conf
 * \code
 * int8_t bma5_get_if_conf(struct bma5_if_conf *config, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to get the Serial interface settings
 *
 * @param[out] config : Structure instance of bma5_if_conf
 * @param[in]  dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma5_get_if_conf(struct bma5_if_conf *config, struct bma5_dev *dev);

/*!
 * \ingroup bma5ApiRegs
 * \page bma5_api_bma5_get_if_conf bma5_get_if_conf
 * \code
 * int8_t bma5_set_if_conf(const struct bma5_if_conf *config, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to set the Serial interface settings
 *
 * @param[in] config : Structure instance of bma5_if_conf
 * @param[in] dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma5_set_if_conf(const struct bma5_if_conf *config, struct bma5_dev *dev);

/*!
 * \ingroup bma5ApiRegs
 * \page bma5_api_bma5_get_fifo_ctrl bma5_get_fifo_ctrl
 * \code
 * int8_t bma5_set_fifo_ctrl(const struct bma5_fifo_ctrl *config, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to set the FIFO control register
 *
 * @param[in] config : Structure instance of bma5_fifo_ctrl
 * @param[in] dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma5_set_fifo_ctrl(const struct bma5_fifo_ctrl *config, struct bma5_dev *dev);

/*!
 * \ingroup bma5ApiRegs
 * \page bma5_api_bma5_get_fifo_conf bma5_get_fifo_conf
 * \code
 * int8_t bma5_get_fifo_conf(struct bma5_fifo_conf *config, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to get the FIFO configuration register
 *
 * @param[out] config : Structure instance of bma5_fifo_conf
 * @param[in]  dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma5_get_fifo_conf(struct bma5_fifo_conf *config, struct bma5_dev *dev);

/*!
 * \ingroup bma5ApiRegs
 * \page bma5_api_bma5_get_fifo_conf bma5_get_fifo_conf
 * \code
 * int8_t bma5_set_fifo_conf(const struct bma5_fifo_conf *config, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to set the FIFO configuration register
 *
 * @param[in] config : Structure instance of bma5_fifo_conf
 * @param[in] dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma5_set_fifo_conf(const struct bma5_fifo_conf *config, struct bma5_dev *dev);

/*!
 * \ingroup bma5ApiRegs
 * \page bma5_api_bma5_get_feat_eng_conf bma5_get_feat_eng_conf
 * \code
 * int8_t bma5_get_feat_eng_conf(uint8_t *feat_eng_ctrl, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to get the feature engine ('feat_eng') configuration register
 *
 * @param[out] feat_eng_ctrl : An enable/disable switch for the feature engine. The feature engine is internally reseted, once the engine is disabled and the enabled again.
 * @param[in]  dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma5_get_feat_eng_conf(uint8_t *feat_eng_ctrl, struct bma5_dev *dev);

/*!
 * \ingroup bma5ApiRegs
 * \page bma5_api_bma5_get_feat_eng_conf bma5_get_feat_eng_conf
 * \code
 * int8_t bma5_set_feat_eng_conf(const uint8_t feat_eng_ctrl, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to set the feature engine ('feat_eng') configuration register
 *
 * @param[in] feat_eng_ctrl : An enable/disable switch for the feature engine. The feature engine is internally reseted, once the engine is disabled and the enabled again.
 * @param[in] dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma5_set_feat_eng_conf(const uint8_t feat_eng_ctrl, struct bma5_dev *dev);

/*!
 * \ingroup bma5ApiRegs
 * \page bma5_api_bma5_get_feat_eng_status bma5_get_feat_eng_status
 * \code
 * int8_t bma5_get_feat_eng_status(struct bma5_feat_eng_status *config, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to get the feature engine ('feat_eng') status register
 *
 * @param[out] config : Structure instance of bma5_feat_eng_status
 * @param[in]  dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma5_get_feat_eng_status(struct bma5_feat_eng_status *config, struct bma5_dev *dev);

/*!
 * \ingroup bma5ApiRegs
 * \page bma5_api_bma5_get_feat_eng_gpr_conf bma5_get_feat_eng_gpr_conf
 * \code
 * int8_t bma5_get_feat_eng_gpr_conf(struct bma5_feat_eng_gpr_conf *config, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to get the feature engine ('feat_eng') general purpose register configuration register
 *
 * @param[out] config : Structure instance of bma5_feat_eng_gpr_conf
 * @param[in]  dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma5_get_feat_eng_gpr_conf(struct bma5_feat_eng_gpr_conf *config, struct bma5_dev *dev);

/*!
 * \ingroup bma5ApiRegs
 * \page bma5_api_bma5_get_feat_eng_gpr_conf bma5_get_feat_eng_gpr_conf
 * \code
 * int8_t bma5_set_feat_eng_gpr_conf(const struct bma5_feat_eng_gpr_conf *config, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to set the feature engine ('feat_eng') general purpose register configuration register
 *
 * @param[in] config : Structure instance of bma5_feat_eng_gpr_conf
 * @param[in] dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma5_set_feat_eng_gpr_conf(const struct bma5_feat_eng_gpr_conf *config, struct bma5_dev *dev);

/*!
 * \ingroup bma5ApiRegs
 * \page bma5_api_bma5_get_feat_eng_gpr_ctrl bma5_get_feat_eng_gpr_ctrl
 * \code
 * int8_t bma5_set_feat_eng_gpr_ctrl(const struct bma5_feat_eng_gpr_ctrl *config, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to set the feature engine ('feat_eng') general purpose register control register
 *
 * @param[in] config : Structure instance of bma5_feat_eng_gpr_ctrl
 * @param[in] dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma5_set_feat_eng_gpr_ctrl(const struct bma5_feat_eng_gpr_ctrl *config, struct bma5_dev *dev);

/*!
 * \ingroup bma5ApiRegs
 * \page bma5_api_bma5_get_feature_data_addr bma5_get_feature_data_addr
 * \code
 * int8_t bma5_get_feature_data_addr(uint8_t *feature_data_addr, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to get the feature engine ('feat_eng') feature data start address
 *
 * @param[out] feature_data_addr : Feature data address. For the address values see the extended memory map.
 * @param[in]  dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma5_get_feature_data_addr(uint8_t *feature_data_addr, struct bma5_dev *dev);

/*!
 * \ingroup bma5ApiRegs
 * \page bma5_api_bma5_get_feature_data_addr bma5_get_feature_data_addr
 * \code
 * int8_t bma5_set_feature_data_addr(const uint8_t feature_data_addr, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to set the feature engine ('feat_eng') feature data start address
 *
 * @param[in] feature_data_addr : Feature data address. For the address values see the extended memory map.
 * @param[in] dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma5_set_feature_data_addr(const uint8_t feature_data_addr, struct bma5_dev *dev);

/*!
 * \ingroup bma5ApiRegs
 * \page bma5_api_bma5_get_feature_data_tx bma5_get_feature_data_tx
 * \code
 * int8_t bma5_get_feature_data_tx(uint8_t *feature_data, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to get the feature engine ('feat_eng') feature data
 *
 * @param[out] feature_data : The data port associated with feature_data_addr.
 *                            During burst read/write operations on this address
 *                            the address increment stops and the burst operation can be used to
 *                            read/write multiple feature_data words. See the extendend memory map for details.
 * @param[in]  dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma5_get_feature_data_tx(uint8_t *feature_data, struct bma5_dev *dev);

/*!
 * \ingroup bma5ApiRegs
 * \page bma5_api_bma5_get_feature_data_tx bma5_get_feature_data_tx
 * \code
 * int8_t bma5_set_feature_data_tx(const uint8_t feature_data, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to set the feature engine ('feat_eng') feature data
 *
 * @param[in] feature_data : The data port associated with feature_data_addr.
 *                           During burst read/write operations on this address
 *                           the address increment stops and the burst operation can be used to
 *                           read/write multiple feature_data words. See the extendend memory map for details.
 * @param[in] dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma5_set_feature_data_tx(const uint8_t feature_data, struct bma5_dev *dev);

/*!
 * \ingroup bma5ApiRegs
 * \page bma5_api_bma5_get_acc_self_test bma5_get_acc_self_test
 * \code
 * int8_t bma5_get_acc_self_test(struct bma5_acc_self_test *config, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to get the Select NORMAL/SELF_TEST mode and test data. If you write to this register, the ACC data path is reset.
 *
 * @param[out] config : Structure instance of bma5_acc_self_test
 * @param[in]  dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma5_get_acc_self_test(struct bma5_acc_self_test *config, struct bma5_dev *dev);

/*!
 * \ingroup bma5ApiRegs
 * \page bma5_api_bma5_get_acc_self_test bma5_get_acc_self_test
 * \code
 * int8_t bma5_set_acc_self_test(const struct bma5_acc_self_test *config, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to set the Select NORMAL/SELF_TEST mode and test data. If you write to this register, the ACC data path is reset.
 *
 * @param[in] config : Structure instance of bma5_acc_self_test
 * @param[in] dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma5_set_acc_self_test(const struct bma5_acc_self_test *config, struct bma5_dev *dev);

/*!
 * \ingroup bma5ApiRegs
 * \page bma5_api_bma5_get_cmd bma5_get_cmd
 * \code
 * int8_t bma5_set_cmd(const uint8_t cmd, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to set the Command Register
 *
 * @param[in] cmd : Available commands (Note: Register will always read as 0x00):
 * @param[in] dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma5_set_cmd(const uint8_t cmd, struct bma5_dev *dev);

/*!
 * \ingroup bma5ApiRegs
 * \page bma5_api_bma5_get_acc bma5_get_acc
 * \code
 * int8_t bma5_get_acc(struct bma5_accel *accel, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to get the acceleration data
 *
 * @param[out] acc : acceleration data.
 * @param[in]  dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma5_get_acc(struct bma5_accel *accel, struct bma5_dev *dev);

/*!
 * \ingroup bma5ApiRegs
 * \page bma5_api_bma5_get_sensor_time bma5_get_sensor_time
 * \code
 * int8_t bma5_get_sensor_time(uint32_t *sensor_time, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to get the the sensor time
 *
 * @param[out] sensor_time : the sensor time
 * @param[in]  dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma5_get_sensor_time(uint32_t *sensor_time, struct bma5_dev *dev);

/*!
 * \ingroup bma5ApiRegs
 * \page bma5_api_bma5_get_fifo_fill_level bma5_get_fifo_fill_level
 * \code
 * int8_t bma5_get_fifo_fill_level(uint16_t *fifo_fill_level, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to get the FIFO fill level
 *
 * @param[out] fifo_fill_level : FIFO fill level
 * @param[in]  dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma5_get_fifo_fill_level(uint16_t *fifo_fill_level, struct bma5_dev *dev);

/*!
 * \ingroup bma5ApiRegs
 * \page bma5_api_bma5_get_fifo_wm bma5_get_fifo_wm
 * \code
 * int8_t bma5_get_fifo_wm(uint16_t *fifo_wm, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to get the FIFO watermark level
 *
 * @param[out] fifo_wm : FIFO watermark level
 * @param[in]  dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma5_get_fifo_wm(uint16_t *fifo_wm, struct bma5_dev *dev);

/*!
 * \ingroup bma5ApiRegs
 * \page bma5_api_bma5_set_fifo_wm bma5_set_fifo_wm
 * \code
 * int8_t bma5_set_fifo_wm(uint16_t fifo_wm, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to set the FIFO watermark level
 *
 * @param[in] fifo_wm : FIFO watermark level
 * @param[in]  dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma5_set_fifo_wm(uint16_t fifo_wm, struct bma5_dev *dev);

/*!
 * \ingroup bma5ApiRegs
 * \page bma5_api_bma5_set_acc_doff bma5_set_acc_doff
 * \code
 * int8_t bma5_set_acc_doff(struct bma5_accel_doff *accel_doff, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to get the acceleration doff data
 *
 * @param[out] accel_doff : acceleration doff data.
 * @param[in]  dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma5_set_acc_doff(const struct bma5_accel_doff *accel_doff, struct bma5_dev *dev);

/*!
 * \ingroup bma5ApiRegs
 * \page bma5_api_bma5_get_acc_doff bma5_get_acc_doff
 * \code
 * int8_t bma5_get_acc_doff(struct bma5_accel_doff *accel_doff, struct bma5_dev *dev);
 * \endcode
 * @details This API carries the provision to get the acceleration doff data
 *
 * @param[out] accel_doff : acceleration doff data.
 * @param[in]  dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma5_get_acc_doff(struct bma5_accel_doff *accel_doff, struct bma5_dev *dev);

/**
 * \ingroup bma5
 * \defgroup bma5Apist BMA5 Self-Test
 * @brief Self-Test APIs
 */

/*!
 * \ingroup bma5Apist
 * \page bma5_api_bma5_activate_self_test bma5_activate_self_test
 * \code
 * int8_t bma5_activate_self_test(struct bma5_dev *dev);
 * \endcode
 * @details This API is to activate self-test procedure
 *
 * @param[in,out] dev : Structure instance of bma5_dev
 *
 * @return Result of API execution status.
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma5_activate_self_test(struct bma5_dev *dev);

/*!
 * \ingroup bma5Apist
 * \page bma5_api_bma5_self_test_pos_excitation bma5_self_test_pos_excitation
 * \code
 * int8_t bma5_self_test_pos_excitation(struct bma5_dev *dev);
 * \endcode
 * @details This API is to perform self-test procedure with positive excitation
 *
 * @param[in,out] dev : Structure instance of bma5_dev
 *
 * @return Result of API execution status.
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma5_self_test_pos_excitation(struct bma5_dev *dev);

/*!
 * \ingroup bma5Apist
 * \page bma5_api_bma5_self_test_neg_excitation bma5_self_test_neg_excitation
 * \code
 * int8_t bma5_self_test_neg_excitation(struct bma5_dev *dev);
 * \endcode
 * @details This API is to perform self-test procedure with negative excitation
 *
 * @param[in,out] dev : Structure instance of bma5_dev
 *
 * @return Result of API execution status.
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma5_self_test_neg_excitation(struct bma5_dev *dev);

/**
 * \ingroup bma5
 * \defgroup bma5FIFOApiRegs BMA5 FIFO
 * @brief APIs for FIFO
 */

/*!
 * \ingroup bma5FIFOApiRegs
 * \page bma5_api_bma5_read_fifo_data bma5_read_fifo_data
 * \code
 * int8_t bma5_read_fifo_data(struct bma5_fifo_frame *fifo, const struct bma5_fifo_conf *fifo_conf,
 * struct bma5_dev *dev);
 * \endcode
 * @details This API reads the FIFO data from the sensor
 *
 * @param[in, out] fifo        : Structure instance of bma5_fifo_frame.
 * @param[in, out] fifo_conf   : Structure instance of bma5_fifo_conf.
 * @param[in,out] dev              : Structure instance of bma5_dev.
 *
 *  @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma5_read_fifo_data(struct bma5_fifo_frame *fifo, const struct bma5_fifo_conf *fifo_conf, struct bma5_dev *dev);

/*!
 * \ingroup bma5FIFOApiRegs
 * \page bma5_api_bma5_extract_acc_sens_time_8_bit bma5_extract_acc_sens_time_8_bit
 * \code
 * int8_t bma5_extract_acc_sens_time_8_bit(struct bma5_sens_fifo_axes_data_8_bit *accel_data,
                                        struct bma5_fifo_frame *fifo,
                                        const struct bma5_fifo_conf *fifo_conf,
                                        const struct bma5_dev *dev);
 * \endcode
 *  @details This API parses and extracts the accelerometer and sensortime frames from
 *  FIFO 8 bit data read by the "bma5_read_fifo_data" API and stores it in the
 *  "bma5_sens_fifo_axes_data_8_bit" structure instance.
 *
 * @param[in] accel_data       : Structure instance of bma5_sens_fifo_axes_data_8_bit.
 * @param[in] fifo             : Structure instance of bma5_fifo_frame.
 * @param[in] fifo_conf        : Structure instance of bma5_fifo_conf.
 * @param[in] dev              : Structure instance of bma5_dev.
 *
 *  @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma5_extract_acc_sens_time_8_bit(struct bma5_sens_fifo_axes_data_8_bit *accel_data,
                                        struct bma5_fifo_frame *fifo,
                                        const struct bma5_fifo_conf *fifo_conf,
                                        const struct bma5_dev *dev);

/*!
 * \ingroup bma5FIFOApiRegs
 * \page bma5_api_bma5_extract_acc_sens_time_16_bit bma5_extract_acc_sens_time_16_bit
 * \code
 * int8_t bma5_extract_acc_sens_time_16_bit(struct bma5_sens_fifo_axes_data_16_bit *accel_data,
                                         struct bma5_fifo_frame *fifo,
                                         const struct bma5_fifo_conf *fifo_conf,
                                         const struct bma5_dev *dev);
 * \endcode
 *  @details This API parses and extracts the accelerometer and sensortime frames from
 *  FIFO 16 bit data read by the "bma5_read_fifo_data" API and stores it in the
 *  "bma5_sens_fifo_axes_data_16_bit" structure instance.
 *
 * @param[in] accel_data       : Structure instance of bma5_sens_fifo_axes_data_16_bit.
 * @param[in] fifo             : Structure instance of bma5_fifo_frame.
 * @param[in] fifo_conf        : Structure instance of bma5_fifo_conf.
 * @param[in] dev              : Structure instance of bma5_dev.
 *
 *  @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
int8_t bma5_extract_acc_sens_time_16_bit(struct bma5_sens_fifo_axes_data_16_bit *accel_data,
                                         struct bma5_fifo_frame *fifo,
                                         const struct bma5_fifo_conf *fifo_conf,
                                         const struct bma5_dev *dev);

#ifdef __cplusplus
}
#endif /* End of CPP guard */

#endif /*_BMA5_H */
