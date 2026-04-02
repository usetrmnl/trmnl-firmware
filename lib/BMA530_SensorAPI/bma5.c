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
* @file       bma5.c
* @date       2024-07-29
* @version    v4.2.0
*
*/

/******************************************************************************/
/****************************** Header files **********************************/
/******************************************************************************/
#include "bma5.h"

/******************************************************************************/
/*********************** Static function declarations *************************/
/******************************************************************************/

/*!
 * @brief Internal API to verify the validity of the primary device handle which
 *        is passed as argument.
 *
 * @param[in] dev : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 * @retval BMA5_OK -> device handle is valid
 * @retval BMA5_E_NULL_PTR -> Null pointer error
 */
static int8_t verify_handle(const struct bma5_dev *dev);

/*!
 *  @brief This internal API is used to reset the FIFO related configurations
 *  in the fifo_frame structure.
 *
 * @param[in] fifo : Structure instance of bma5_fifo_frame.
 *
 *  @return Void
 */
static void reset_fifo_data_structure(struct bma5_fifo_frame *fifo);

/*!
 *  @brief This internal API is used to unpack the accelerometer and sensor time data from the
 *  FIFO 8 bit data.
 *  It updates the idx value which is used to store the index of
 *  the current data byte which is parsed.
 *
 * @param[in] frame_header : Stores header byte.
 * @param[in, out] acc     : Structure instance of bma5_sens_fifo_axes_data_8_bit.
 * @param[in, out] idx     : Index value of number of bytes parsed.
 * @param[in, out] acc_idx : Index value of accel data (x,y,z axes)
 *                           frame to be parsed.
 * @param[in] fifo         : Structure instance of bma5_fifo_frame.
 * @param[in] fifo_conf    : Structure instance of bma5_fifo_conf.
 *
 *  @return Result of API execution status
 *  @retval BMA5_OK -> Success
 *  @retval BMA5_E_NULL_PTR -> Null pointer Error
 */
static int8_t unpack_accel_sensor_time_frm_8_bit(uint8_t frame_header,
                                                 struct bma5_sens_fifo_axes_data_8_bit *acc,
                                                 uint16_t *idx,
                                                 uint16_t *acc_idx,
                                                 const struct bma5_fifo_frame *fifo,
                                                 const struct bma5_fifo_conf *fifo_conf);

/*!
 *  @brief This internal API is used to unpack the accelerometer and sensor time data from the
 *  FIFO 16 bit data.
 *  It updates the idx value which is used to store the index of
 *  the current data byte which is parsed.
 *
 * @param[in] frame_header : Stores header byte.
 * @param[in, out] acc     : Structure instance of bma5_sens_fifo_axes_data_16_bit.
 * @param[in, out] idx     : Index value of number of bytes parsed.
 * @param[in, out] acc_idx : Index value of accel data (x,y,z axes)
 *                           frame to be parsed.
 * @param[in] fifo         : Structure instance of bma5_fifo_frame.
 * @param[in] fifo_conf    : Structure instance of bma5_fifo_conf.
 *
 *  @return Result of API execution status
 *  @retval BMA5_OK -> Success
 *  @retval BMA5_E_NULL_PTR -> Null pointer Error
 */
static int8_t unpack_accel_sensor_time_frm_16_bit(uint8_t frame_header,
                                                  struct bma5_sens_fifo_axes_data_16_bit *acc,
                                                  uint16_t *idx,
                                                  uint16_t *acc_idx,
                                                  const struct bma5_fifo_frame *fifo,
                                                  const struct bma5_fifo_conf *fifo_conf);

/*!
 *  @brief This internal API is used to unpack the accelerometer and sensor time data from the
 *  FIFO 8 bit data and store it in the instance of the structure bma5_sens_fifo_axes_data_8_bit.
 *
 * @param[in] frame_header              : Stores header byte.
 * @param[in, out] accel_data           : Structure instance of bma5_sens_fifo_axes_data_8_bit.
 * @param[in, out] data_start_index     : Index value of number of bytes parsed.
 * @param[in] per_frame_data            : Variable to hold total number of bytes in one frame.
 * @param[in] fifo                      : Structure instance of bma5_fifo_frame.
 *
 *  @return Result of API execution status
 *  @retval BMA5_OK -> Success
 *  @retval BMA5_E_NULL_PTR -> Null pointer Error
 */
static int8_t unpack_accel_sensor_time_8_bit_data(uint8_t frame_header,
                                                  struct bma5_sens_fifo_axes_data_8_bit *accel_data,
                                                  uint16_t *data_start_index,
                                                  uint16_t per_frame_data,
                                                  const struct bma5_fifo_frame *fifo);

/*!
 *  @brief This internal API is used to unpack the accelerometer and sensor time data from the
 *  FIFO 16 bit data and store it in the instance of the structure bma5_sens_fifo_axes_data_16_bit.
 *
 * @param[in] frame_header              : Stores header byte.
 * @param[in, out] accel_data           : Structure instance of bma5_sens_fifo_axes_data_16_bit.
 * @param[in, out] data_start_index     : Index value of number of bytes parsed.
 * @param[in] fifo                      : Structure instance of bma5_fifo_frame.
 *
 *  @return Result of API execution status
 *  @retval BMA5_OK -> Success
 *  @retval BMA5_E_NULL_PTR -> Null pointer Error
 */
static int8_t unpack_accel_sensor_time_16_bit_data(uint8_t frame_header,
                                                   struct bma5_sens_fifo_axes_data_16_bit *accel_data,
                                                   uint16_t *data_start_index,
                                                   const struct bma5_fifo_frame *fifo);

/*!
 *  @brief This internal API reads the FIFO fill level which is set
 *  in the sensor.
 *
 * @param[out] fifo_fill_lvl    : Get FIFO fill level.
 * @param[in,out] dev               : Structure instance of bma5_dev.
 *
 * @return Result of API execution status
 *
 * @retval = 0 -> Success
 * @retval > 0 -> Warning
 * @retval < 0 -> Error
 */
static int8_t get_fifo_fill_level(uint16_t *fifo_fill_lvl, struct bma5_dev *dev);

/******************************************************************************/
/*********************** User function definitions ****************************/
/******************************************************************************/
int8_t bma5_get_health_status(uint8_t *sensor_health_status, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    /* Temporary variable to carry the register value */
    uint8_t reg_value;

    if (NULL == sensor_health_status)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        result = bma5_get_regs(BMA5_REG_HEALTH_STATUS, &reg_value, sizeof(reg_value), dev);
        if (BMA5_OK == result)
        {
            /* Parse needed details from received serial data */
            *sensor_health_status = BMA5_GET_BITS_POS_0(reg_value, BMA5_SENSOR_HEALTH_STATUS);
        }
    }

    return result;
}

int8_t bma5_get_cmd_suspend(uint8_t *suspend, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    /* Temporary variable to carry the register value */
    uint8_t reg_value;

    if (NULL == suspend)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        result = bma5_get_regs(BMA5_REG_CMD_SUSPEND, &reg_value, sizeof(reg_value), dev);
        if (BMA5_OK == result)
        {
            /* Parse needed details from received serial data */
            *suspend = BMA5_GET_BITS_POS_0(reg_value, BMA5_SUSPEND);
        }
    }

    return result;
}

int8_t bma5_set_cmd_suspend(const uint8_t suspend, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    /* Temporary variable to store the register value to be set */
    uint8_t reg_value = 0;

    /* Bring up the register value to be set, as per the input details */
    reg_value = BMA5_SET_BITS_POS_0(reg_value, BMA5_SUSPEND, suspend);

    result = bma5_set_regs(BMA5_REG_CMD_SUSPEND, (const uint8_t *)&reg_value, sizeof(reg_value), dev);

    return result;
}

int8_t bma5_get_config_status(struct bma5_config_status *config, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    /* Temporary variable to carry the register value */
    uint8_t reg_value;

    if (NULL == config)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        result = bma5_get_regs(BMA5_REG_CONFIG_STATUS, &reg_value, sizeof(reg_value), dev);
        if (BMA5_OK == result)
        {
            /* Parse needed details from received serial data */
            config->feat_eng_err = BMA5_GET_BITS_POS_0(reg_value, BMA5_FEAT_ENG_ERR);
            config->acc_conf_err = BMA5_GET_BITS(reg_value, BMA5_ACC_CONF_ERR);
        }
    }

    return result;
}

int8_t bma5_set_config_status(const struct bma5_config_status *config, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    /* Temporary variable to store the register value to be set */
    uint8_t reg_value = 0;

    if (NULL == config)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        /* Bring up the register value to be set, as per the input details */
        reg_value = BMA5_SET_BITS_POS_0(reg_value, BMA5_FEAT_ENG_ERR, config->feat_eng_err);

        result = bma5_set_regs(BMA5_REG_CONFIG_STATUS, (const uint8_t *)&reg_value, sizeof(reg_value), dev);
    }

    return result;
}

int8_t bma5_get_sensor_status(struct bma5_sensor_status *config, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    /* Temporary variable to carry the register value */
    uint8_t reg_value;

    if (NULL == config)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        result = bma5_get_regs(BMA5_REG_SENSOR_STATUS, &reg_value, sizeof(reg_value), dev);
        if (BMA5_OK == result)
        {
            /* Parse needed details from received serial data */
            config->acc_data_rdy = BMA5_GET_BITS_POS_0(reg_value, BMA5_ACC_DATA_RDY);
            config->temperature_rdy = BMA5_GET_BITS(reg_value, BMA5_TEMPERATURE_RDY);
            config->sensor_rdy = BMA5_GET_BITS(reg_value, BMA5_SENSOR_RDY);
        }
    }

    return result;
}

int8_t bma5_set_sensor_status(const struct bma5_sensor_status *config, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    /* Temporary variable to store the register value to be set */
    uint8_t reg_value = 0;

    if (NULL == config)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        /* Bring up the register value to be set, as per the input details */
        reg_value = BMA5_SET_BITS_POS_0(reg_value, BMA5_ACC_DATA_RDY, config->acc_data_rdy);
        reg_value = BMA5_SET_BITS(reg_value, BMA5_TEMPERATURE_RDY, config->temperature_rdy);

        result = bma5_set_regs(BMA5_REG_SENSOR_STATUS, (const uint8_t *)&reg_value, sizeof(reg_value), dev);
    }

    return result;
}

int8_t bma5_get_temp_data(uint8_t *temp_data, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    /* Temporary variable to carry the register value */
    uint8_t reg_value;

    if (NULL == temp_data)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        result = bma5_get_regs(BMA5_REG_TEMP_DATA, &reg_value, sizeof(reg_value), dev);
        if (BMA5_OK == result)
        {
            /* Parse needed details from received serial data */
            *temp_data = BMA5_GET_BITS_POS_0(reg_value, BMA5_TEMP_DATA);
        }
    }

    return result;
}

int8_t bma5_get_fifo_data_out(uint8_t *fifo_data_out, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    /* Temporary variable to carry the register value */
    uint8_t reg_value;

    if (NULL == fifo_data_out)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        result = bma5_get_regs(BMA5_REG_FIFO_DATA_OUT, &reg_value, sizeof(reg_value), dev);
        if (BMA5_OK == result)
        {
            /* Parse needed details from received serial data */
            *fifo_data_out = BMA5_GET_BITS_POS_0(reg_value, BMA5_FIFO_DATA_OUT);
        }
    }

    return result;
}

int8_t bma5_get_acc_conf_0(uint8_t *sensor_ctrl, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    /* Temporary variable to carry the register value */
    uint8_t reg_value;

    if (NULL == sensor_ctrl)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        result = bma5_get_regs(BMA5_REG_ACC_CONF_0, &reg_value, sizeof(reg_value), dev);
        if (BMA5_OK == result)
        {
            /* Parse needed details from received serial data */
            *sensor_ctrl = BMA5_GET_BITS_POS_0(reg_value, BMA5_SENSOR_CTRL);
        }
    }

    return result;
}

int8_t bma5_set_acc_conf_0(const uint8_t sensor_ctrl, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    /* Temporary variable to store the register value to be set */
    uint8_t reg_value = 0;

    /* Bring up the register value to be set, as per the input details */
    reg_value = BMA5_SET_BITS_POS_0(reg_value, BMA5_SENSOR_CTRL, sensor_ctrl);

    result = bma5_set_regs(BMA5_REG_ACC_CONF_0, (const uint8_t *)&reg_value, sizeof(reg_value), dev);

    return result;
}

int8_t bma5_get_acc_conf(struct bma5_acc_conf *config, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    /* Temporary variable to carry the register value */
    uint8_t reg_value[2] = { 0 };

    if (NULL == config)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        result = bma5_get_regs(BMA5_REG_ACC_CONF_1, reg_value, 2, dev);
        if (BMA5_OK == result)
        {
            /* Parse needed details from received serial data */
            config->acc_odr = BMA5_GET_BITS_POS_0(reg_value[0], BMA5_ACC_ODR);
            config->acc_bwp = BMA5_GET_BITS(reg_value[0], BMA5_ACC_BWP);
            config->power_mode = BMA5_GET_BITS(reg_value[0], BMA5_POWER_MODE);

            config->acc_range = BMA5_GET_BITS_POS_0(reg_value[1], BMA5_ACC_RANGE);
            config->acc_iir_ro = BMA5_GET_BITS(reg_value[1], BMA5_ACC_IIR_RO);
            config->noise_mode = BMA5_GET_BITS(reg_value[1], BMA5_NOISE_MODE);
            config->acc_drdy_int_auto_clear = BMA5_GET_BITS(reg_value[1], BMA5_ACC_DRDY_INT_AUTO_CLEAR);
        }
    }

    return result;
}

int8_t bma5_set_acc_conf(const struct bma5_acc_conf *config, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    /* Temporary variable to store the register value to be set */
    uint8_t reg_value[2] = { 0 };

    if (NULL == config)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        /* Bring up the register value to be set, as per the input details */
        reg_value[0] = BMA5_SET_BITS_POS_0(reg_value[0], BMA5_ACC_ODR, config->acc_odr);
        reg_value[0] = BMA5_SET_BITS(reg_value[0], BMA5_ACC_BWP, config->acc_bwp);
        reg_value[0] = BMA5_SET_BITS(reg_value[0], BMA5_POWER_MODE, config->power_mode);

        reg_value[1] = BMA5_SET_BITS_POS_0(reg_value[1], BMA5_ACC_RANGE, config->acc_range);
        reg_value[1] = BMA5_SET_BITS(reg_value[1], BMA5_ACC_IIR_RO, config->acc_iir_ro);
        reg_value[1] = BMA5_SET_BITS(reg_value[1], BMA5_NOISE_MODE, config->noise_mode);
        reg_value[1] = BMA5_SET_BITS(reg_value[1], BMA5_ACC_DRDY_INT_AUTO_CLEAR, config->acc_drdy_int_auto_clear);

        result = bma5_set_regs(BMA5_REG_ACC_CONF_1, (const uint8_t *)reg_value, 2, dev);
    }

    return result;
}

int8_t bma5_get_temp_conf(struct bma5_temp_conf *config, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    /* Temporary variable to carry the register value */
    uint8_t reg_value;

    if (NULL == config)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        result = bma5_get_regs(BMA5_REG_TEMP_CONF, &reg_value, sizeof(reg_value), dev);
        if (BMA5_OK == result)
        {
            /* Parse needed details from received serial data */
            config->temp_rate = BMA5_GET_BITS_POS_0(reg_value, BMA5_TEMP_RATE);
            config->temp_meas_src = BMA5_GET_BITS(reg_value, BMA5_TEMP_MEAS_SRC);
            config->temp_ext_sel = BMA5_GET_BITS(reg_value, BMA5_TEMP_EXT_SEL);
        }
    }

    return result;
}

int8_t bma5_set_temp_conf(const struct bma5_temp_conf *config, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    /* Temporary variable to store the register value to be set */
    uint8_t reg_value = 0;

    if (NULL == config)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        /* Bring up the register value to be set, as per the input details */
        reg_value = BMA5_SET_BITS_POS_0(reg_value, BMA5_TEMP_RATE, config->temp_rate);
        reg_value = BMA5_SET_BITS(reg_value, BMA5_TEMP_MEAS_SRC, config->temp_meas_src);
        reg_value = BMA5_SET_BITS(reg_value, BMA5_TEMP_EXT_SEL, config->temp_ext_sel);

        result = bma5_set_regs(BMA5_REG_TEMP_CONF, (const uint8_t *)&reg_value, sizeof(reg_value), dev);
    }

    return result;
}

int8_t bma5_get_int_conf(struct bma5_int_conf_types *config, uint8_t int_configs, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result = BMA5_OK;
    uint8_t loop;

    /* Temporary variable to carry the register value */
    uint8_t reg_value;

    if (NULL == config)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        for (loop = 0; loop < int_configs; loop++)
        {
            if (config[loop].int_src == BMA5_INT_1)
            {
                result = bma5_get_regs(BMA5_REG_INT1_CONF, &reg_value, sizeof(reg_value), dev);
            }
            else if (config[loop].int_src == BMA5_INT_2)
            {
                result = bma5_get_regs(BMA5_REG_INT2_CONF, &reg_value, sizeof(reg_value), dev);
            }
            else
            {
                result = BMA5_E_INVALID_INTERRUPT;
            }

            /* Update the config values. */
            if (BMA5_OK != result)
            {
                break;
            }

            /* Parse needed details from received serial data */
            config[loop].int_conf.int_mode = BMA5_GET_BITS_POS_0(reg_value, BMA5_INT_MODE);
            config[loop].int_conf.int_od = BMA5_GET_BITS(reg_value, BMA5_INT_OD);
            config[loop].int_conf.int_lvl = BMA5_GET_BITS(reg_value, BMA5_INT_LVL);
        }
    }

    return result;
}

int8_t bma5_set_int_conf(const struct bma5_int_conf_types *config, uint8_t int_configs, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result = BMA5_OK;
    uint8_t loop;

    /* Temporary variable to store the register value to be set */
    uint8_t reg_value = 0;

    if (NULL == config)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        for (loop = 0; loop < int_configs; loop++)
        {
            /* Bring up the register value to be set, as per the input details */
            reg_value = BMA5_SET_BITS_POS_0(reg_value, BMA5_INT_MODE, config[loop].int_conf.int_mode);
            reg_value = BMA5_SET_BITS(reg_value, BMA5_INT_OD, config[loop].int_conf.int_od);
            reg_value = BMA5_SET_BITS(reg_value, BMA5_INT_LVL, config[loop].int_conf.int_lvl);

            if (config[loop].int_src == BMA5_INT_1)
            {
                result = bma5_set_regs(BMA5_REG_INT1_CONF, (const uint8_t *)&reg_value, sizeof(reg_value), dev);
            }
            else if (config[loop].int_src == BMA5_INT_2)
            {
                result = bma5_set_regs(BMA5_REG_INT2_CONF, (const uint8_t *)&reg_value, sizeof(reg_value), dev);
            }
            else
            {
                result = BMA5_E_INVALID_INTERRUPT;
            }

            if (result != BMA5_OK)
            {
                break;
            }
        }
    }

    return result;
}

int8_t bma5_get_if_conf_0(uint8_t *if_i2c_slv_addr, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    /* Temporary variable to carry the register value */
    uint8_t reg_value;

    if (NULL == if_i2c_slv_addr)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        result = bma5_get_regs(BMA5_REG_IF_CONF_0, &reg_value, sizeof(reg_value), dev);
        if (BMA5_OK == result)
        {
            /* Parse needed details from received serial data */
            *if_i2c_slv_addr = BMA5_GET_BITS_POS_0(reg_value, BMA5_IF_I2C_SLV_ADDR);
        }
    }

    return result;
}

int8_t bma5_get_if_conf(struct bma5_if_conf *config, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    /* Temporary variable to carry the register value */
    uint8_t reg_value;

    if (NULL == config)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        result = bma5_get_regs(BMA5_REG_IF_CONF_1, &reg_value, sizeof(reg_value), dev);
        if (BMA5_OK == result)
        {
            /* Parse needed details from received serial data */
            config->if_i3c_cfg = BMA5_GET_BITS_POS_0(reg_value, BMA5_IF_I3C_CFG);
            config->if_spi3_cfg = BMA5_GET_BITS(reg_value, BMA5_IF_SPI3_CFG);
            config->if_csb_pullup = BMA5_GET_BITS(reg_value, BMA5_IF_CSB_PULLUP);
            config->if_pad_drv = BMA5_GET_BITS(reg_value, BMA5_IF_PAD_DRV);
            config->if_i2c_drv_sel = BMA5_GET_BITS(reg_value, BMA5_IF_I2C_DRV_SEL);
        }
    }

    return result;
}

int8_t bma5_set_if_conf(const struct bma5_if_conf *config, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    /* Temporary variable to store the register value to be set */
    uint8_t reg_value = 0;

    if (NULL == config)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        /* Bring up the register value to be set, as per the input details */
        reg_value = BMA5_SET_BITS_POS_0(reg_value, BMA5_IF_I3C_CFG, config->if_i3c_cfg);
        reg_value = BMA5_SET_BITS(reg_value, BMA5_IF_SPI3_CFG, config->if_spi3_cfg);
        reg_value = BMA5_SET_BITS(reg_value, BMA5_IF_CSB_PULLUP, config->if_csb_pullup);
        reg_value = BMA5_SET_BITS(reg_value, BMA5_IF_PAD_DRV, config->if_pad_drv);
        reg_value = BMA5_SET_BITS(reg_value, BMA5_IF_I2C_DRV_SEL, config->if_i2c_drv_sel);

        result = bma5_set_regs(BMA5_REG_IF_CONF_1, (const uint8_t *)&reg_value, sizeof(reg_value), dev);
    }

    return result;
}

int8_t bma5_set_fifo_ctrl(const struct bma5_fifo_ctrl *config, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    /* Temporary variable to store the register value to be set */
    uint8_t reg_value = 0;

    if (NULL == config)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        /* Bring up the register value to be set, as per the input details */
        reg_value = BMA5_SET_BITS_POS_0(reg_value, BMA5_FIFO_RST, config->fifo_rst);
        reg_value = BMA5_SET_BITS(reg_value, BMA5_FIFO_FRAME_SYNC, config->fifo_frame_sync);

        result = bma5_set_regs(BMA5_REG_FIFO_CTRL, (const uint8_t *)&reg_value, sizeof(reg_value), dev);
    }

    return result;
}

int8_t bma5_get_fifo_conf(struct bma5_fifo_conf *config, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    /* Temporary variable to carry the register value */
    uint8_t reg_value[2] = { 0 };

    if (NULL == config)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        result = bma5_get_regs(BMA5_REG_FIFO_CONF_0, reg_value, 2, dev);
        if (BMA5_OK == result)
        {
            /* Parse needed details from received serial data */
            config->fifo_cfg = BMA5_GET_BITS_POS_0(reg_value[0], BMA5_FIFO_CFG);
            config->fifo_acc_x = BMA5_GET_BITS(reg_value[0], BMA5_FIFO_ACC_X);
            config->fifo_acc_y = BMA5_GET_BITS(reg_value[0], BMA5_FIFO_ACC_Y);
            config->fifo_acc_z = BMA5_GET_BITS(reg_value[0], BMA5_FIFO_ACC_Z);
            config->fifo_compression = BMA5_GET_BITS(reg_value[0], BMA5_FIFO_COMPRESSION);

            config->fifo_size = BMA5_GET_BITS_POS_0(reg_value[1], BMA5_FIFO_SIZE);
            config->fifo_sensor_time = BMA5_GET_BITS(reg_value[1], BMA5_FIFO_SENSOR_TIME);
            config->fifo_stop_on_full = BMA5_GET_BITS(reg_value[1], BMA5_FIFO_STOP_ON_FULL);
        }
    }

    return result;
}

int8_t bma5_set_fifo_conf(const struct bma5_fifo_conf *config, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    /* Temporary variable to store the register value to be set */
    uint8_t reg_value[2] = { 0 };

    if (NULL == config)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        reg_value[0] = BMA5_SET_BITS_POS_0(reg_value[0], BMA5_FIFO_CFG, config->fifo_cfg);
        reg_value[0] = BMA5_SET_BITS(reg_value[0], BMA5_FIFO_ACC_X, config->fifo_acc_x);
        reg_value[0] = BMA5_SET_BITS(reg_value[0], BMA5_FIFO_ACC_Y, config->fifo_acc_y);
        reg_value[0] = BMA5_SET_BITS(reg_value[0], BMA5_FIFO_ACC_Z, config->fifo_acc_z);
        reg_value[0] = BMA5_SET_BITS(reg_value[0], BMA5_FIFO_COMPRESSION, config->fifo_compression);

        reg_value[1] = BMA5_SET_BITS_POS_0(reg_value[1], BMA5_FIFO_SIZE, config->fifo_size);
        reg_value[1] = BMA5_SET_BITS(reg_value[1], BMA5_FIFO_SENSOR_TIME, config->fifo_sensor_time);
        reg_value[1] = BMA5_SET_BITS(reg_value[1], BMA5_FIFO_STOP_ON_FULL, config->fifo_stop_on_full);

        result = bma5_set_regs(BMA5_REG_FIFO_CONF_0, (const uint8_t *)reg_value, 2, dev);
    }

    return result;
}

int8_t bma5_get_feat_eng_conf(uint8_t *feat_eng_ctrl, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    /* Temporary variable to carry the register value */
    uint8_t reg_value;

    if (NULL == feat_eng_ctrl)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        result = bma5_get_regs(BMA5_REG_FEAT_ENG_CONF, &reg_value, sizeof(reg_value), dev);
        if (BMA5_OK == result)
        {
            /* Parse needed details from received serial data */
            *feat_eng_ctrl = BMA5_GET_BITS_POS_0(reg_value, BMA5_FEAT_ENG_CTRL);
        }
    }

    return result;
}

int8_t bma5_set_feat_eng_conf(const uint8_t feat_eng_ctrl, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    /* Temporary variable to store the register value to be set */
    uint8_t reg_value = 0;

    /* Bring up the register value to be set, as per the input details */
    reg_value = BMA5_SET_BITS_POS_0(reg_value, BMA5_FEAT_ENG_CTRL, feat_eng_ctrl);

    result = bma5_set_regs(BMA5_REG_FEAT_ENG_CONF, (const uint8_t *)&reg_value, sizeof(reg_value), dev);

    return result;
}

int8_t bma5_get_feat_eng_status(struct bma5_feat_eng_status *config, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    /* Temporary variable to carry the register value */
    uint8_t reg_value;

    if (NULL == config)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        result = bma5_get_regs(BMA5_REG_FEAT_ENG_STATUS, &reg_value, sizeof(reg_value), dev);
        if (BMA5_OK == result)
        {
            /* Parse needed details from received serial data */
            config->feat_eng_halted = BMA5_GET_BITS_POS_0(reg_value, BMA5_FEAT_ENG_HALTED);
            config->feat_eng_running = BMA5_GET_BITS(reg_value, BMA5_FEAT_ENG_RUNNING);
            config->host_gpr_update_pending = BMA5_GET_BITS(reg_value, BMA5_HOST_GPR_UPDATE_PENDING);
            config->feat_eng_gpr_update_pending = BMA5_GET_BITS(reg_value, BMA5_FEAT_ENG_GPR_UPDATE_PENDING);
        }
    }

    return result;
}

int8_t bma5_get_feat_eng_gpr_conf(struct bma5_feat_eng_gpr_conf *config, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    /* Temporary variable to carry the register value */
    uint8_t reg_value;

    if (NULL == config)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        result = bma5_get_regs(BMA5_REG_FEAT_ENG_GPR_CONF, &reg_value, sizeof(reg_value), dev);
        if (BMA5_OK == result)
        {
            /* Parse needed details from received serial data */
            config->feat_eng_gpr_0_dir = BMA5_GET_BITS_POS_0(reg_value, BMA5_FEAT_ENG_GPR_0_DIR);
            config->feat_eng_gpr_1_dir = BMA5_GET_BITS(reg_value, BMA5_FEAT_ENG_GPR_1_DIR);
            config->feat_eng_gpr_2_dir = BMA5_GET_BITS(reg_value, BMA5_FEAT_ENG_GPR_2_DIR);
            config->feat_eng_gpr_3_dir = BMA5_GET_BITS(reg_value, BMA5_FEAT_ENG_GPR_3_DIR);
            config->feat_eng_gpr_4_dir = BMA5_GET_BITS(reg_value, BMA5_FEAT_ENG_GPR_4_DIR);
            config->feat_eng_gpr_5_dir = BMA5_GET_BITS(reg_value, BMA5_FEAT_ENG_GPR_5_DIR);
            config->feat_eng_gpr_6_dir = BMA5_GET_BITS(reg_value, BMA5_FEAT_ENG_GPR_6_DIR);
        }
    }

    return result;
}

int8_t bma5_set_feat_eng_gpr_conf(const struct bma5_feat_eng_gpr_conf *config, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    /* Temporary variable to store the register value to be set */
    uint8_t reg_value = 0;

    if (NULL == config)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        /* Bring up the register value to be set, as per the input details */
        reg_value = BMA5_SET_BITS_POS_0(reg_value, BMA5_FEAT_ENG_GPR_0_DIR, config->feat_eng_gpr_0_dir);
        reg_value = BMA5_SET_BITS(reg_value, BMA5_FEAT_ENG_GPR_1_DIR, config->feat_eng_gpr_1_dir);
        reg_value = BMA5_SET_BITS(reg_value, BMA5_FEAT_ENG_GPR_2_DIR, config->feat_eng_gpr_2_dir);
        reg_value = BMA5_SET_BITS(reg_value, BMA5_FEAT_ENG_GPR_3_DIR, config->feat_eng_gpr_3_dir);
        reg_value = BMA5_SET_BITS(reg_value, BMA5_FEAT_ENG_GPR_4_DIR, config->feat_eng_gpr_4_dir);
        reg_value = BMA5_SET_BITS(reg_value, BMA5_FEAT_ENG_GPR_5_DIR, config->feat_eng_gpr_5_dir);
        reg_value = BMA5_SET_BITS(reg_value, BMA5_FEAT_ENG_GPR_6_DIR, config->feat_eng_gpr_6_dir);

        result = bma5_set_regs(BMA5_REG_FEAT_ENG_GPR_CONF, (const uint8_t *)&reg_value, sizeof(reg_value), dev);
    }

    return result;
}

int8_t bma5_set_feat_eng_gpr_ctrl(const struct bma5_feat_eng_gpr_ctrl *config, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    /* Temporary variable to store the register value to be set */
    uint8_t reg_value = 0;

    if (NULL == config)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        /* Bring up the register value to be set, as per the input details */
        reg_value = BMA5_SET_BITS_POS_0(reg_value, BMA5_UPDATE_GPRS, config->update_gprs);
        reg_value = BMA5_SET_BITS(reg_value, BMA5_UNLOCK_GPRS, config->unlock_gprs);

        result = bma5_set_regs(BMA5_REG_FEAT_ENG_GPR_CTRL, (const uint8_t *)&reg_value, sizeof(reg_value), dev);
    }

    return result;
}

int8_t bma5_get_feature_data_addr(uint8_t *feature_data_addr, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    /* Temporary variable to carry the register value */
    uint8_t reg_value;

    if (NULL == feature_data_addr)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        result = bma5_get_regs(BMA5_REG_FEATURE_DATA_ADDR, &reg_value, sizeof(reg_value), dev);
        if (BMA5_OK == result)
        {
            /* Parse needed details from received serial data */
            *feature_data_addr = BMA5_GET_BITS_POS_0(reg_value, BMA5_FEATURE_DATA_ADDR);
        }
    }

    return result;
}

int8_t bma5_set_feature_data_addr(const uint8_t feature_data_addr, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    /* Temporary variable to store the register value to be set */
    uint8_t reg_value = 0;

    /* Bring up the register value to be set, as per the input details */
    reg_value = BMA5_SET_BITS_POS_0(reg_value, BMA5_FEATURE_DATA_ADDR, feature_data_addr);

    result = bma5_set_regs(BMA5_REG_FEATURE_DATA_ADDR, (const uint8_t *)&reg_value, sizeof(reg_value), dev);

    return result;
}

int8_t bma5_get_feature_data_tx(uint8_t *feature_data, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    /* Temporary variable to carry the register value */
    uint8_t reg_value;

    if (NULL == feature_data)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        result = bma5_get_regs(BMA5_REG_FEATURE_DATA_TX, &reg_value, sizeof(reg_value), dev);
        if (BMA5_OK == result)
        {
            /* Parse needed details from received serial data */
            *feature_data = BMA5_GET_BITS_POS_0(reg_value, BMA5_FEATURE_DATA);
        }
    }

    return result;
}

int8_t bma5_set_feature_data_tx(const uint8_t feature_data, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    /* Temporary variable to store the register value to be set */
    uint8_t reg_value = 0;

    /* Bring up the register value to be set, as per the input details */
    reg_value = BMA5_SET_BITS_POS_0(reg_value, BMA5_FEATURE_DATA, feature_data);

    result = bma5_set_regs(BMA5_REG_FEATURE_DATA_TX, (const uint8_t *)&reg_value, sizeof(reg_value), dev);

    return result;
}

int8_t bma5_get_acc_self_test(struct bma5_acc_self_test *config, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    /* Temporary variable to carry the register value */
    uint8_t reg_value;

    if (NULL == config)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        result = bma5_get_regs(BMA5_REG_ACC_SELF_TEST, &reg_value, sizeof(reg_value), dev);
        if (BMA5_OK == result)
        {
            /* Parse needed details from received serial data */
            config->self_test = BMA5_GET_BITS_POS_0(reg_value, BMA5_SELF_TEST);
            config->self_test_sign = BMA5_GET_BITS(reg_value, BMA5_SELF_TEST_SIGN);
        }
    }

    return result;
}

int8_t bma5_set_acc_self_test(const struct bma5_acc_self_test *config, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    /* Temporary variable to store the register value to be set */
    uint8_t reg_value = 0;

    if (NULL == config)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        /* Bring up the register value to be set, as per the input details */
        reg_value = BMA5_SET_BITS_POS_0(reg_value, BMA5_SELF_TEST, config->self_test);
        reg_value = BMA5_SET_BITS(reg_value, BMA5_SELF_TEST_SIGN, config->self_test_sign);

        result = bma5_set_regs(BMA5_REG_ACC_SELF_TEST, (const uint8_t *)&reg_value, sizeof(reg_value), dev);
    }

    return result;
}

int8_t bma5_set_cmd(const uint8_t cmd, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    /* Temporary variable to store the register value to be set */
    uint8_t reg_value = 0;

    /* Bring up the register value to be set, as per the input details */
    reg_value = BMA5_SET_BITS_POS_0(reg_value, BMA5_CMD, cmd);

    result = bma5_set_regs(BMA5_REG_CMD, (const uint8_t *)&reg_value, sizeof(reg_value), dev);

    return result;
}

int8_t bma5_get_acc(struct bma5_accel *accel, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    /* Temporary buffer to receive the serial data from sensor */
    uint8_t data[6] = { 0 };

    if (NULL == accel)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        result = bma5_get_regs(BMA5_REG_ACC_DATA_0, data, 6, dev);
        if (BMA5_OK == result)
        {
            /* Group the serial data to get needed detail */
            accel->x = (int16_t)(((uint16_t)data[1] << 8) | ((uint16_t)data[0]));
            accel->y = (int16_t)(((uint16_t)data[3] << 8) | ((uint16_t)data[2]));
            accel->z = (int16_t)(((uint16_t)data[5] << 8) | ((uint16_t)data[4]));
        }
    }

    return result;
}

int8_t bma5_get_sensor_time(uint32_t *sensor_time, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    /* Temporary buffer to receive the serial data from sensor */
    uint8_t data[3] = { 0 };

    if (NULL == sensor_time)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        result = bma5_get_regs(BMA5_REG_SENSOR_TIME_0, (uint8_t *)data, sizeof(data), dev);
        if (BMA5_OK == result)
        {
            /* Group the serial data to get needed detail */
            *sensor_time = (uint32_t)(((uint32_t)data[2] << 16) | ((uint32_t)data[1] << 8) | ((uint32_t)data[0]));
        }
    }

    return result;
}

int8_t bma5_get_fifo_fill_level(uint16_t *fifo_fill_level, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    /* Temporary variable to receive the serial data from sensor */
    uint8_t reg_value;

    /* Temporary variables to carry values of components */
    uint8_t fifo_fill_level_7_0;
    uint8_t fifo_fill_level_10_8;

    if (NULL == fifo_fill_level)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        result = bma5_get_regs(BMA5_REG_FIFO_LEVEL_0, &reg_value, sizeof(reg_value), dev);
        if (BMA5_OK == result)
        {
            /* Parse fifo_fill_level_7_0 from received serial data */
            fifo_fill_level_7_0 = BMA5_GET_BITS_POS_0(reg_value, BMA5_FIFO_FILL_LEVEL_7_0);

            result = bma5_get_regs(BMA5_REG_FIFO_LEVEL_1, &reg_value, sizeof(reg_value), dev);
        }

        if (BMA5_OK == result)
        {
            /* Parse fifo_fill_level_10_8 from received serial data */
            fifo_fill_level_10_8 = BMA5_GET_BITS_POS_0(reg_value, BMA5_FIFO_FILL_LEVEL_10_8);

            /* Group the components to get needed detail */
            *fifo_fill_level = (uint16_t)(((uint16_t)fifo_fill_level_10_8 << 8) | ((uint16_t)fifo_fill_level_7_0));
        }
    }

    return result;
}

int8_t bma5_get_fifo_wm(uint16_t *fifo_wm, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    /* Temporary variable to receive the serial data from sensor */
    uint8_t reg_value;

    /* Temporary variables to carry values of components */
    uint8_t fifo_watermark_level_7_0;
    uint8_t fifo_watermark_level_10_8;

    if (NULL == fifo_wm)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        result = bma5_get_regs(BMA5_REG_FIFO_WM_0, &reg_value, sizeof(reg_value), dev);
        if (BMA5_OK == result)
        {
            /* Parse fifo_watermark_level_7_0 from received serial data */
            fifo_watermark_level_7_0 = BMA5_GET_BITS_POS_0(reg_value, BMA5_FIFO_WATERMARK_LEVEL_7_0);

            result = bma5_get_regs(BMA5_REG_FIFO_WM_1, &reg_value, sizeof(reg_value), dev);
        }

        if (BMA5_OK == result)
        {
            /* Parse fifo_watermark_level_10_8 from received serial data */
            fifo_watermark_level_10_8 = BMA5_GET_BITS_POS_0(reg_value, BMA5_FIFO_WATERMARK_LEVEL_10_8);

            /* Group the components to get needed detail */
            *fifo_wm = (uint16_t)(((uint16_t)fifo_watermark_level_10_8 << 8) | ((uint16_t)fifo_watermark_level_7_0));
        }
    }

    return result;
}

/*!
 *  @brief This API sets the FIFO watermark level in the sensor.
 */
int8_t bma5_set_fifo_wm(uint16_t fifo_wm, struct bma5_dev *dev)
{
    /* Variable to define error */
    int8_t result;

    /* Array to store FIFO watermark level */
    uint8_t data[2] = { 0 };

    data[0] = (uint8_t)(fifo_wm & UINT16_C(0x00FF));
    data[1] = (uint8_t)((fifo_wm & UINT16_C(0xFF00)) >> 8);

    /* Set FIFO watermark level */
    result = bma5_set_regs(BMA5_REG_FIFO_WM_0, data, 2, dev);

    return result;
}

int8_t bma5_get_acc_doff(struct bma5_accel_doff *accel_doff, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    /* Temporary variable to receive the serial data from sensor */
    uint8_t reg_value[6] = { 0 };

    /* Temporary variables to carry values of components */
    uint8_t acc_doff_7_0;
    uint8_t acc_doff_8;

    if (NULL == accel_doff)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        result = bma5_get_regs(BMA5_REG_ACC_OFFSET_0, reg_value, 6, dev);

        if (BMA5_OK == result)
        {
            acc_doff_7_0 = BMA5_GET_BITS_POS_0(reg_value[0], BMA5_ACC_DOFF_7_0);
            acc_doff_8 = BMA5_GET_BITS_POS_0(reg_value[1], BMA5_ACC_DOFF_8);

            accel_doff->x_doff = (int16_t)((((uint16_t)acc_doff_8 << 8) | ((uint16_t)acc_doff_7_0)) << 7) >> 7;

            acc_doff_7_0 = BMA5_GET_BITS_POS_0(reg_value[2], BMA5_ACC_DOFF_7_0);
            acc_doff_8 = BMA5_GET_BITS_POS_0(reg_value[3], BMA5_ACC_DOFF_8);

            accel_doff->y_doff = (int16_t)((((uint16_t)acc_doff_8 << 8) | ((uint16_t)acc_doff_7_0)) << 7) >> 7;

            acc_doff_7_0 = BMA5_GET_BITS_POS_0(reg_value[4], BMA5_ACC_DOFF_7_0);
            acc_doff_8 = BMA5_GET_BITS_POS_0(reg_value[5], BMA5_ACC_DOFF_8);

            accel_doff->z_doff = (int16_t)((((uint16_t)acc_doff_8 << 8) | ((uint16_t)acc_doff_7_0)) << 7) >> 7;
        }
    }

    return result;
}

/*!
 * @brief This API sets accel user offset for x, y and z-axis.
 */
int8_t bma5_set_acc_doff(const struct bma5_accel_doff *accel_doff, struct bma5_dev *dev)
{
    /* Variable to define error */
    int8_t result = BMA5_OK;

    /* Array to store accel foc user offset data */
    uint8_t accel_doff_data[6] = { 0 };

    if (accel_doff == NULL)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        accel_doff_data[0] = BMA5_SET_BITS_POS_0(accel_doff_data[0], BMA5_ACC_DOFF_X_7_0, accel_doff->x_doff);
        accel_doff_data[1] = BMA5_SET_BITS_POS_0(accel_doff_data[1], BMA5_ACC_DOFF_X_8, accel_doff->x_doff);
        accel_doff_data[2] = BMA5_SET_BITS_POS_0(accel_doff_data[2], BMA5_ACC_DOFF_Y_7_0, accel_doff->y_doff);
        accel_doff_data[3] = BMA5_SET_BITS_POS_0(accel_doff_data[3], BMA5_ACC_DOFF_Y_8, accel_doff->y_doff);
        accel_doff_data[4] = BMA5_SET_BITS_POS_0(accel_doff_data[4], BMA5_ACC_DOFF_Z_7_0, accel_doff->z_doff);
        accel_doff_data[5] = BMA5_SET_BITS_POS_0(accel_doff_data[5], BMA5_ACC_DOFF_Z_8, accel_doff->z_doff);

        /* Set accel user offset */
        result = bma5_set_regs(BMA5_REG_ACC_OFFSET_0, accel_doff_data, 6, dev);
    }

    return result;
}

/*!
 * @brief This API reads the data from the given register address of bma5
 *        sensor.
 */
int8_t bma5_get_regs(uint8_t addr, uint8_t *data, uint32_t len, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    /* Temporary buffer to receive the serial data from sensor */
    uint8_t temp_buf[BMA5_MAX_BUFFER_SIZE] = { 0 };

    /* Number of bytes to be copied from temp_buf to data */
    uint32_t bytes_to_copy = len;

    result = verify_handle(dev);
    if ((BMA5_OK == result) && (NULL == data))
    {
        result = BMA5_E_NULL_PTR;
    }

    if (BMA5_OK == result)
    {
        /* Configuring register address for SPI Interface */
        if (BMA5_SPI_INTF == dev->intf)
        {
            addr = addr | BMA5_SPI_RD_MSK;
        }

        dev->intf_rslt = dev->bus_read(addr, temp_buf, (len + dev->dummy_byte), dev->intf_ptr);

        if (BMA5_INTF_RET_SUCCESS != dev->intf_rslt)
        {
            result = BMA5_E_COM_FAIL;
        }
    }

    if (BMA5_OK == result)
    {
        while (bytes_to_copy--)
        {
            data[bytes_to_copy] = temp_buf[bytes_to_copy + dev->dummy_byte];
        }
    }

    return result;
}

/*!
 * @brief This API writes data to the given register address of bma5 sensor.
 */
int8_t bma5_set_regs(uint8_t addr, const uint8_t *data, uint32_t len, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    result = verify_handle(dev);
    if ((BMA5_OK == result) && (NULL == data))
    {
        result = BMA5_E_NULL_PTR;
    }

    if (BMA5_OK == result)
    {
        /* Configuring register address for SPI Interface */
        if (BMA5_SPI_INTF == dev->intf)
        {
            addr = (addr & BMA5_SPI_WR_MSK);
        }

        dev->intf_rslt = dev->bus_write(addr, data, len, dev->intf_ptr);

        if (BMA5_INTF_RET_SUCCESS != dev->intf_rslt)
        {
            result = BMA5_E_COM_FAIL;
        }
    }

    return result;
}

/*!
 * @brief This API is to activate self-test procedure
 */
int8_t bma5_activate_self_test(struct bma5_dev *dev)
{
    /* Variable to store the function result */
    int8_t result;

    /* Variables to set the accel configuration for self test */
    /* Setting ODR = 1.6 Khz, ACC_BWP = Normal Mode, Power Mode = High Performance Mode (Continuous Mode */
    uint8_t acc_conf1 = 0xAA;

    /* Setting Acc Range = 8G, ACC IIR RO = db_20 -20dB roll-off */
    uint8_t acc_conf2 = 0x06;
    uint8_t acc_conf0;

    /* Disable accelerometer */
    acc_conf0 = BMA5_SENSOR_CTRL_DISABLE;
    result = bma5_set_regs(BMA5_REG_ACC_CONF_0, &acc_conf0, 1, dev);

    if (result == BMA5_OK)
    {
        /* Set acc_odr as 1.6 kHZ, acc_bwp as normal mode and acc_perf_mode as continuous mode */
        result = bma5_set_regs(BMA5_REG_ACC_CONF_1, &acc_conf1, 1, dev);

        if (result == BMA5_OK)
        {
            /* Set acc_range as 8g, acc_iir_ro as -20dB roll-off */
            result = bma5_set_regs(BMA5_REG_ACC_CONF_2, &acc_conf2, 1, dev);

            if (result == BMA5_OK)
            {
                /* Enable accelerometer */
                acc_conf0 = BMA5_SENSOR_CTRL_ENABLE;
                result = bma5_set_regs(BMA5_REG_ACC_CONF_0, &acc_conf0, 1, dev);

                if (result == BMA5_OK)
                {
                    /* Delay of 10ms required after write of above configurations */
                    dev->delay_us(BMA5_SELF_TEST_DELAY, dev->intf_ptr);
                }
            }
        }
    }

    return result;
}

/*!
 * @brief This API is to perform self-test procedure
 * with positive excitation
 */
int8_t bma5_self_test_pos_excitation(struct bma5_dev *dev)
{
    /* Variable to store the function result */
    int8_t result;

    /* Variables to set the accel configuration for self test */
    uint8_t acc_self_test = 0x03;

    /* Enable self-test with positive excitation */
    result = bma5_set_regs(BMA5_REG_ACC_SELF_TEST, &acc_self_test, 1, dev);

    if (result == BMA5_OK)
    {
        /* Delay of 10ms required after write of above configurations */
        dev->delay_us(BMA5_SELF_TEST_DELAY, dev->intf_ptr);
    }

    return result;
}

/*!
 * @brief This API is to perform self-test procedure
 * with negative excitation
 */
int8_t bma5_self_test_neg_excitation(struct bma5_dev *dev)
{
    /* Variable to store the function result */
    int8_t result;

    /* Variables to set the accel configuration for self test */
    uint8_t acc_self_test = 0x01;

    /* Enable self-test with negative excitation */
    result = bma5_set_regs(BMA5_REG_ACC_SELF_TEST, &acc_self_test, 1, dev);

    if (result == BMA5_OK)
    {
        /* Delay of 10ms required after write of above configurations */
        dev->delay_us(BMA5_SELF_TEST_DELAY, dev->intf_ptr);
    }

    return result;
}

/*!
 *  @brief This API reads the FIFO data from the sensor
 */
int8_t bma5_read_fifo_data(struct bma5_fifo_frame *fifo, const struct bma5_fifo_conf *fifo_conf, struct bma5_dev *dev)
{
    /* Variable to store the function result */
    int8_t result;

    /* Variable to store FIFO data register address */
    uint8_t reg_addr = BMA5_REG_FIFO_DATA_OUT;

    /* Number of bytes to be copied */
    uint32_t bytes_to_copy = 0;

    /* Null-pointer check */
    result = verify_handle(dev);

    if ((result == BMA5_OK) && (fifo != NULL) && (fifo->data != NULL) && (fifo_conf != NULL))
    {
        /* Reset the FIFO data structure */
        reset_fifo_data_structure(fifo);

        /* Get fifo available length in bytes */
        result = get_fifo_fill_level(&fifo->fifo_avail_len, dev);

        if ((fifo->fifo_avail_len != 0) && (result == BMA5_OK))
        {
            /* Configuring reg_addr for SPI Interface */
            if (dev->intf == BMA5_SPI_INTF)
            {
                reg_addr = (reg_addr | BMA5_SPI_RD_MSK);
            }

            if (fifo_conf->fifo_sensor_time == BMA5_FIFO_SENSOR_TIME_DEDICATED_FRAME)
            {
                /* Overhead bytes added to read FIFO for higher ODRs */
                fifo->fifo_avail_len += BMA5_SENSORTIME_OVERHEAD_BYTE;
            }

            dev->intf_rslt =
                dev->bus_read(reg_addr, fifo->data, (fifo->fifo_avail_len + dev->dummy_byte), dev->intf_ptr);

            dev->delay_us(2, dev->intf_ptr);

            if (dev->intf_rslt != BMA5_INTF_RET_SUCCESS)
            {
                result = BMA5_E_COM_FAIL;
            }
            else
            {
                while (bytes_to_copy < fifo->fifo_avail_len)
                {
                    fifo->data[bytes_to_copy] = fifo->data[bytes_to_copy + dev->dummy_byte];

                    bytes_to_copy++;
                }
            }
        }
    }
    else
    {
        result = BMA5_E_NULL_PTR;
    }

    return result;
}

/*!
 *  @brief This API parses and extracts the accelerometer and sensortime frames from
 *  FIFO 8 bit data read by the "bma5_read_fifo_data" API and stores it in the
 *  "bma5_sens_fifo_axes_data_8_bit" structure instance.
 */
int8_t bma5_extract_acc_sens_time_8_bit(struct bma5_sens_fifo_axes_data_8_bit *accel_data,
                                        struct bma5_fifo_frame *fifo,
                                        const struct bma5_fifo_conf *fifo_conf,
                                        const struct bma5_dev *dev)
{
    /* Variable to store the function result */
    int8_t result;

    /* Variable to store frame header byte */
    uint8_t frame_header = 0;

    /* Variable to store index value of number of bytes parsed */
    uint16_t data_index = 0;

    /* Variable to store index value of accel data (x,y,z axes) frame to be parsed. */
    uint16_t accel_index = 0;

    /* Null-pointer check */
    result = verify_handle(dev);

    if ((result == BMA5_OK) && (accel_data != NULL) && (fifo != NULL) && (fifo_conf != NULL))
    {
        for (data_index = fifo->acc_byte_start_idx;
             ((data_index < fifo->fifo_avail_len) && (fifo->data[data_index] != 128));
             )
        {
            /* Header byte is stored in the variable frame_header */
            frame_header = fifo->data[data_index];

            /* Index is moved to next byte where the data is starting */
            data_index++;

            result = unpack_accel_sensor_time_frm_8_bit(frame_header,
                                                        accel_data,
                                                        &data_index,
                                                        &accel_index,
                                                        fifo,
                                                        fifo_conf);
        }

        /* Update number of accel data read */
        fifo->fifo_avail_frames = accel_index;

        /* Update the accelerometer byte index */
        fifo->acc_byte_start_idx = data_index;
    }
    else
    {
        result = BMA5_E_NULL_PTR;
    }

    return result;
}

/*!
 *  @brief This API parses and extracts the accelerometer and sensortime frames from
 *  FIFO 16 bit data read by the "bma5_read_fifo_data" API and stores it in the
 *  "bma5_sens_fifo_axes_data_16_bit" structure instance.
 */
int8_t bma5_extract_acc_sens_time_16_bit(struct bma5_sens_fifo_axes_data_16_bit *accel_data,
                                         struct bma5_fifo_frame *fifo,
                                         const struct bma5_fifo_conf *fifo_conf,
                                         const struct bma5_dev *dev)
{
    /* Variable to store the function result */
    int8_t result;

    /* Variable to store frame header byte */
    uint8_t frame_header = 0;

    /* Variable to store index value of number of bytes parsed */
    uint16_t data_index = 0;

    /* Variable to store index value of accel data (x,y,z axes) frame to be parsed. */
    uint16_t accel_index = 0;

    /* Null-pointer check */
    result = verify_handle(dev);

    if ((result == BMA5_OK) && (accel_data != NULL) && (fifo != NULL) && (fifo_conf != NULL))
    {
        for (data_index = fifo->acc_byte_start_idx;
             ((data_index < fifo->fifo_avail_len) && (fifo->data[data_index] != 128));
             )
        {
            /* Header byte is stored in the variable frame_header */
            frame_header = fifo->data[data_index];

            /* Get the frame details from header */
            frame_header = frame_header & BMA5_FIFO_FRAME_HDR_MSK;

            /* Index is moved to next byte where the data is starting */
            data_index++;

            result = unpack_accel_sensor_time_frm_16_bit(frame_header,
                                                         accel_data,
                                                         &data_index,
                                                         &accel_index,
                                                         fifo,
                                                         fifo_conf);
        }

        /* Update number of accel data read */
        fifo->fifo_avail_frames = accel_index;

        /* Update the accelerometer byte index */
        fifo->acc_byte_start_idx = data_index;
    }
    else
    {
        result = BMA5_E_NULL_PTR;
    }

    return result;
}

/******************************************************************************/
/*********************** Static function definitions **************************/
/******************************************************************************/
static int8_t verify_handle(const struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result = BMA5_E_NULL_PTR;

    if (NULL != dev)
    {
        if ((NULL != dev->bus_read) && (NULL != dev->bus_write) && (NULL != dev->delay_us))
        {
            result = BMA5_OK;
        }
    }

    return result;
}

/*!
 *  @brief This internal API is used to reset the FIFO related configurations
 *  in the fifo_frame structure.
 */
static void reset_fifo_data_structure(struct bma5_fifo_frame *fifo)
{
    /* Prepare for next FIFO read by resetting FIFO's
     * internal data structures
     */
    fifo->fifo_avail_len = 0;
    fifo->acc_byte_start_idx = 0;
    fifo->fifo_avail_frames = 0;
}

/*!
 *  @brief This internal API is used to unpack the accelerometer and sensortime data from the
 *  FIFO 8 bit data.
 *  It updates the idx value which is used to store the index of
 *  the current data byte which is parsed.
 */
static int8_t unpack_accel_sensor_time_frm_8_bit(uint8_t frame_header,
                                                 struct bma5_sens_fifo_axes_data_8_bit *acc,
                                                 uint16_t *idx,
                                                 uint16_t *acc_idx,
                                                 const struct bma5_fifo_frame *fifo,
                                                 const struct bma5_fifo_conf *fifo_conf)
{
    /* Variable to store the function result */
    int8_t result;

    /* Variable to hold total number of bytes in one frame */
    uint16_t per_frame_data = 0;

    if (fifo_conf->fifo_sensor_time & BMA5_FIFO_SENSOR_TIME_EACH_FRAME)
    {
        per_frame_data = BMA5_FIFO_8_BIT_ACCEL_XYZ_SENS_TIME_DATA;
    }
    else if (fifo_conf->fifo_sensor_time & BMA5_FIFO_SENSOR_TIME_DEDICATED_FRAME)
    {
        per_frame_data = BMA5_FIFO_8_BIT_ACCEL_XYZ_DATA;
    }

    /* Partial read, then skip the data */
    if ((*idx + per_frame_data) > fifo->fifo_avail_len)
    {
        /* Update the data index as complete */
        *idx = fifo->fifo_avail_len;
    }

    /* Unpack the data array into the structure instance "acc" */
    result = unpack_accel_sensor_time_8_bit_data(frame_header, &acc[*acc_idx], idx, per_frame_data, fifo);

    if (result == BMA5_OK)
    {
        (*acc_idx)++;
    }
    else if (result == BMA5_W_FIFO_PARTIAL_READ)
    {
        *idx = fifo->fifo_avail_len;
    }

    return result;
}

/*!
 *  @brief This internal API is used to unpack the accelerometer and sensortime data from the
 *  FIFO 16 bit data.
 *  It updates the idx value which is used to store the index of
 *  the current data byte which is parsed.
 */
static int8_t unpack_accel_sensor_time_frm_16_bit(uint8_t frame_header,
                                                  struct bma5_sens_fifo_axes_data_16_bit *acc,
                                                  uint16_t *idx,
                                                  uint16_t *acc_idx,
                                                  const struct bma5_fifo_frame *fifo,
                                                  const struct bma5_fifo_conf *fifo_conf)
{
    /* Variable to store the function result */
    int8_t result;

    /* Variable to hold total number of bytes in one frame */
    uint16_t per_frame_data = 0;

    if (fifo_conf->fifo_sensor_time & BMA5_FIFO_SENSOR_TIME_EACH_FRAME)
    {
        per_frame_data = BMA5_FIFO_16_BIT_ACCEL_XYZ_SENS_TIME_DATA;
    }
    else if (fifo_conf->fifo_sensor_time & BMA5_FIFO_SENSOR_TIME_DEDICATED_FRAME)
    {
        per_frame_data = BMA5_FIFO_16_BIT_ACCEL_XYZ_DATA;
    }

    /* Partial read, then skip the data */
    if ((*idx + per_frame_data) > fifo->fifo_avail_len)
    {
        /* Update the data index as complete */
        *idx = fifo->fifo_avail_len;
    }

    /* Unpack the data array into the structure instance "acc" */
    result = unpack_accel_sensor_time_16_bit_data(frame_header, &acc[*acc_idx], idx, fifo);

    if (result == BMA5_OK)
    {
        (*acc_idx)++;
    }
    else if (result == BMA5_W_FIFO_PARTIAL_READ)
    {
        *idx = fifo->fifo_avail_len;
    }

    return result;
}

/*!
 *  @brief This internal API is used to unpack the accelerometer data from the
 *  FIFO 8 bit data and store it in the instance of the structure bma5_sens_fifo_axes_data_8_bit.
 */
static int8_t unpack_accel_sensor_time_8_bit_data(uint8_t frame_header,
                                                  struct bma5_sens_fifo_axes_data_8_bit *accel_data,
                                                  uint16_t *data_start_index,
                                                  uint16_t per_frame_data,
                                                  const struct bma5_fifo_frame *fifo)
{
    /* Variable to store the function result */
    int8_t result = BMA5_OK;

    /* Variable to store sensor time */
    uint32_t sens_time_data_xlsb, sens_time_data_lsb, sens_time_data_msb;

    if ((((*data_start_index) + per_frame_data) <= fifo->fifo_avail_len) &&
        (frame_header != BMA5_FIFO_SENS_TIME_EN_DEDI_FRM))
    {
        if (frame_header & BMA5_FIFO_ACC_X_MSK)
        {
            /* Accel raw x data */
            accel_data->x = (int8_t)(fifo->data[(*data_start_index)++]);
        }
        else
        {
            accel_data->x = 0;
        }

        if (frame_header & BMA5_FIFO_ACC_Y_MSK)
        {
            /* Accel raw y data */
            accel_data->y = (int8_t)(fifo->data[(*data_start_index)++]);
        }
        else
        {
            accel_data->y = 0;
        }

        if (frame_header & BMA5_FIFO_ACC_Z_MSK)
        {
            /* Accel raw z data */
            accel_data->z = (int8_t)(fifo->data[(*data_start_index)++]);
        }
        else
        {
            accel_data->z = 0;
        }

        if (frame_header & BMA5_FIFO_SENS_TIME_EN_HDR_BYTE_MSK)
        {
            sens_time_data_xlsb = fifo->data[(*data_start_index)++];
            sens_time_data_lsb = fifo->data[(*data_start_index)++];
            sens_time_data_msb = fifo->data[(*data_start_index)++];

            accel_data->sensor_time = (sens_time_data_xlsb | (sens_time_data_lsb << 8) | (sens_time_data_msb << 16));
        }
        else
        {
            accel_data->sensor_time = 0;
        }
    }
    else
    {
        result = BMA5_W_FIFO_PARTIAL_READ;
    }

    if (frame_header == BMA5_FIFO_SENS_TIME_EN_DEDI_FRM)
    {
        sens_time_data_xlsb = fifo->data[(*data_start_index)++];
        sens_time_data_lsb = fifo->data[(*data_start_index)++];
        sens_time_data_msb = fifo->data[(*data_start_index)++];

        accel_data->sensor_time = (sens_time_data_xlsb | (sens_time_data_lsb << 8) | (sens_time_data_msb << 16));
    }

    return result;
}

/*!
 *  @brief This internal API is used to unpack the accelerometer data from the
 *  FIFO 16 bit data and store it in the instance of the structure bma5_sens_fifo_axes_data_16_bit.
 */
static int8_t unpack_accel_sensor_time_16_bit_data(uint8_t frame_header,
                                                   struct bma5_sens_fifo_axes_data_16_bit *accel_data,
                                                   uint16_t *data_start_index,
                                                   const struct bma5_fifo_frame *fifo)
{
    /* Variable to store the function result */
    int8_t result = BMA5_OK;

    /* Variable to store accel data LSB and MSB values */
    uint16_t data_lsb, data_msb;

    /* Variable to store sensor time */
    uint32_t sens_time_data_xlsb, sens_time_data_lsb, sens_time_data_msb;

    if ((((*data_start_index) + BMA5_FIFO_16_BIT_ACCEL_XYZ_DATA) <= fifo->fifo_avail_len) &&
        (frame_header != BMA5_FIFO_SENS_TIME_EN_DEDI_FRM))
    {
        if (frame_header & BMA5_FIFO_ACC_X_MSK)
        {
            /* Accel raw x data */
            data_lsb = fifo->data[(*data_start_index)++];
            data_msb = fifo->data[(*data_start_index)++];
            accel_data->x = (int16_t)((data_msb << 8) | data_lsb);
        }
        else
        {
            accel_data->x = 0;
        }

        if (frame_header & BMA5_FIFO_ACC_Y_MSK)
        {
            /* Accel raw y data */
            data_lsb = fifo->data[(*data_start_index)++];
            data_msb = fifo->data[(*data_start_index)++];
            accel_data->y = (int16_t)((data_msb << 8) | data_lsb);
        }
        else
        {
            accel_data->y = 0;
        }

        if (frame_header & BMA5_FIFO_ACC_Z_MSK)
        {
            /* Accel raw z data */
            data_lsb = fifo->data[(*data_start_index)++];
            data_msb = fifo->data[(*data_start_index)++];
            accel_data->z = (int16_t)((data_msb << 8) | data_lsb);
        }
        else
        {
            accel_data->z = 0;
        }

        if (frame_header & BMA5_FIFO_SENS_TIME_EN_HDR_BYTE_MSK)
        {
            sens_time_data_xlsb = fifo->data[(*data_start_index)++];
            sens_time_data_lsb = fifo->data[(*data_start_index)++];
            sens_time_data_msb = fifo->data[(*data_start_index)++];

            accel_data->sensor_time = (sens_time_data_xlsb | (sens_time_data_lsb << 8) | (sens_time_data_msb << 16));
        }
        else
        {
            accel_data->sensor_time = 0;
        }
    }
    else
    {
        result = BMA5_W_FIFO_PARTIAL_READ;
    }

    if (frame_header == BMA5_FIFO_SENS_TIME_EN_DEDI_FRM)
    {
        sens_time_data_xlsb = fifo->data[(*data_start_index)++];
        sens_time_data_lsb = fifo->data[(*data_start_index)++];
        sens_time_data_msb = fifo->data[(*data_start_index)++];

        accel_data->sensor_time = (sens_time_data_xlsb | (sens_time_data_lsb << 8) | (sens_time_data_msb << 16));
    }

    return result;
}

/*!
 *  @brief This API reads the FIFO fill level which is set
 *  in the sensor.
 */
static int8_t get_fifo_fill_level(uint16_t *fifo_fill_lvl, struct bma5_dev *dev)
{
    /* Variable to define error */
    int8_t result = BMA5_OK;

    /* Array to store FIFO fill level */
    uint8_t data[2] = { 0, 0 };

    if (fifo_fill_lvl == NULL)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        /* Read the FIFO fill level */
        result = bma5_get_regs(BMA5_REG_FIFO_LEVEL_0, data, 2, dev);

        if (result == BMA5_OK)
        {
            *fifo_fill_lvl = ((uint16_t)(data[1] << 8)) | (data[0]);

            if (*fifo_fill_lvl == 0)
            {
                result = BMA5_W_FIFO_EMPTY;
            }
        }
    }

    return result;
}
