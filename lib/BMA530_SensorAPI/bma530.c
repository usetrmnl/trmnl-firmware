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
* @file       bma530.c
* @date       2024-07-29
* @version    v4.2.0
*
*/

/******************************************************************************/
/****************************** Header files **********************************/
/******************************************************************************/
#include "bma530.h"

/******************************************************************************/
/*********************** User function definitions ****************************/
/******************************************************************************/
int8_t bma530_get_chip_id(uint8_t *chip_id, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    /* Temporary variable to carry the register value */
    uint8_t reg_value;

    if (NULL == chip_id)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        result = bma5_get_regs(BMA530_REG_CHIP_ID, &reg_value, sizeof(reg_value), dev);
        if (BMA5_OK == result)
        {
            /* Parse needed details from received serial data */
            *chip_id = BMA5_GET_BITS_POS_0(reg_value, BMA530_CHIP_ID);
        }
    }

    return result;
}

int8_t bma530_get_int_status(struct bma530_int_status_types *config, uint8_t n_status, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result = BMA5_OK;
    uint8_t loop;

    /* Temporary variable to carry the register value */
    uint8_t reg_value[2] = { 0 };

    if (NULL == config)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        for (loop = 0; loop < n_status; loop++)
        {
            switch (config[loop].int_src)
            {
                case BMA530_INT_STATUS_INT1:
                    result = bma5_get_regs(BMA530_REG_INT_STATUS_INT1_0, reg_value, 2, dev);
                    break;

                case BMA530_INT_STATUS_INT2:
                    result = bma5_get_regs(BMA530_REG_INT_STATUS_INT2_0, reg_value, 2, dev);
                    break;

                case BMA530_INT_STATUS_I3C:
                    result = bma5_get_regs(BMA530_REG_INT_STATUS_I3C_0, reg_value, 2, dev);
                    break;

                default:
                    result = BMA5_E_INVALID_INT_STATUS;
                    break;
            }

            if (BMA5_OK != result)
            {
                break;
            }

            /* Parse needed details from received serial data */
            config[loop].int_status.acc_drdy_int_status = BMA5_GET_BITS_POS_0(reg_value[0], BMA530_ACC_DRDY_INT_STATUS);
            config[loop].int_status.fifo_wm_int_status = BMA5_GET_BITS(reg_value[0], BMA530_FIFO_WM_INT_STATUS);
            config[loop].int_status.fifo_full_int_status = BMA5_GET_BITS(reg_value[0], BMA530_FIFO_FULL_INT_STATUS);
            config[loop].int_status.gen_int1_int_status = BMA5_GET_BITS(reg_value[0], BMA530_GEN_INT1_INT_STATUS);
            config[loop].int_status.gen_int2_int_status = BMA5_GET_BITS(reg_value[0], BMA530_GEN_INT2_INT_STATUS);
            config[loop].int_status.gen_int3_int_status = BMA5_GET_BITS(reg_value[0], BMA530_GEN_INT3_INT_STATUS);
            config[loop].int_status.step_det_int_status = BMA5_GET_BITS(reg_value[0], BMA530_STEP_DET_INT_STATUS);
            config[loop].int_status.step_cnt_int_status = BMA5_GET_BITS(reg_value[0], BMA530_STEP_CNT_INT_STATUS);

            config[loop].int_status.sig_mo_int_status = BMA5_GET_BITS_POS_0(reg_value[1], BMA530_SIG_MO_INT_STATUS);
            config[loop].int_status.tilt_int_status = BMA5_GET_BITS(reg_value[1], BMA530_TILT_INT_STATUS);
            config[loop].int_status.orient_int_status = BMA5_GET_BITS(reg_value[1], BMA530_ORIENT_INT_STATUS);
            config[loop].int_status.acc_foc_int_status = BMA5_GET_BITS(reg_value[1], BMA530_ACC_FOC_INT_STATUS);
            config[loop].int_status.feat_eng_err_int_status =
                BMA5_GET_BITS(reg_value[1], BMA530_FEAT_ENG_ERR_INT_STATUS);
        }
    }

    return result;
}

int8_t bma530_set_int_status(const struct bma530_int_status_types *config, uint8_t n_status, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result = BMA5_OK;
    uint8_t loop;

    /* Temporary variable to store the register value to be set */
    uint8_t reg_value[2] = { 0 };

    if (NULL == config)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        for (loop = 0; loop < n_status; loop++)
        {
            /* Bring up the register value to be set, as per the input details */
            reg_value[0] = BMA5_SET_BITS_POS_0(reg_value[0],
                                               BMA530_ACC_DRDY_INT_STATUS,
                                               config[loop].int_status.acc_drdy_int_status);
            reg_value[0] = BMA5_SET_BITS(reg_value[0],
                                         BMA530_FIFO_WM_INT_STATUS,
                                         config[loop].int_status.fifo_wm_int_status);
            reg_value[0] = BMA5_SET_BITS(reg_value[0],
                                         BMA530_FIFO_FULL_INT_STATUS,
                                         config[loop].int_status.fifo_full_int_status);
            reg_value[0] = BMA5_SET_BITS(reg_value[0],
                                         BMA530_GEN_INT1_INT_STATUS,
                                         config[loop].int_status.gen_int1_int_status);
            reg_value[0] = BMA5_SET_BITS(reg_value[0],
                                         BMA530_GEN_INT2_INT_STATUS,
                                         config[loop].int_status.gen_int2_int_status);
            reg_value[0] = BMA5_SET_BITS(reg_value[0],
                                         BMA530_GEN_INT3_INT_STATUS,
                                         config[loop].int_status.gen_int3_int_status);
            reg_value[0] = BMA5_SET_BITS(reg_value[0],
                                         BMA530_STEP_DET_INT_STATUS,
                                         config[loop].int_status.step_det_int_status);
            reg_value[0] = BMA5_SET_BITS(reg_value[0],
                                         BMA530_STEP_CNT_INT_STATUS,
                                         config[loop].int_status.step_cnt_int_status);

            /* Bring up the register value to be set, as per the input details */
            reg_value[1] = BMA5_SET_BITS_POS_0(reg_value[1],
                                               BMA530_SIG_MO_INT_STATUS,
                                               config[loop].int_status.sig_mo_int_status);
            reg_value[1] = BMA5_SET_BITS(reg_value[1], BMA530_TILT_INT_STATUS, config[loop].int_status.tilt_int_status);
            reg_value[1] = BMA5_SET_BITS(reg_value[1],
                                         BMA530_ORIENT_INT_STATUS,
                                         config[loop].int_status.orient_int_status);
            reg_value[1] = BMA5_SET_BITS(reg_value[1],
                                         BMA530_ACC_FOC_INT_STATUS,
                                         config[loop].int_status.acc_foc_int_status);
            reg_value[1] = BMA5_SET_BITS(reg_value[1],
                                         BMA530_FEAT_ENG_ERR_INT_STATUS,
                                         config[loop].int_status.feat_eng_err_int_status);

            switch (config[loop].int_src)
            {
                case BMA530_INT_STATUS_INT1:
                    result = bma5_set_regs(BMA530_REG_INT_STATUS_INT1_0, reg_value, 2, dev);
                    break;

                case BMA530_INT_STATUS_INT2:
                    result = bma5_set_regs(BMA530_REG_INT_STATUS_INT2_0, reg_value, 2, dev);
                    break;

                case BMA530_INT_STATUS_I3C:
                    result = bma5_set_regs(BMA530_REG_INT_STATUS_I3C_0, reg_value, 2, dev);
                    break;
                default:
                    result = BMA5_E_INVALID_INT_STATUS;
            }

            if (BMA5_OK != result)
            {
                break;
            }
        }
    }

    return result;
}

int8_t bma530_get_int_map(struct bma530_int_map *config, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result = BMA5_OK;

    /* Temporary variable to carry the register value */
    uint8_t reg_value[4] = { 0 };

    if (NULL == config)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        result = bma5_get_regs(BMA530_REG_INT_MAP_0, reg_value, 4, dev);

        if (BMA5_OK == result)
        {
            /* Parse needed details from received serial data */
            config->acc_drdy_int_map = BMA5_GET_BITS_POS_0(reg_value[0], BMA530_ACC_DRDY_INT_MAP);
            config->fifo_wm_int_map = BMA5_GET_BITS(reg_value[0], BMA530_FIFO_WM_INT_MAP);
            config->fifo_full_int_map = BMA5_GET_BITS(reg_value[0], BMA530_FIFO_FULL_INT_MAP);
            config->gen_int1_int_map = BMA5_GET_BITS(reg_value[0], BMA530_GEN_INT1_INT_MAP);

            config->gen_int2_int_map = BMA5_GET_BITS_POS_0(reg_value[1], BMA530_GEN_INT2_INT_MAP);
            config->gen_int3_int_map = BMA5_GET_BITS(reg_value[1], BMA530_GEN_INT3_INT_MAP);
            config->step_det_int_map = BMA5_GET_BITS(reg_value[1], BMA530_STEP_DET_INT_MAP);
            config->step_cnt_int_map = BMA5_GET_BITS(reg_value[1], BMA530_STEP_CNT_INT_MAP);

            config->sig_mo_int_map = BMA5_GET_BITS_POS_0(reg_value[2], BMA530_SIG_MO_INT_MAP);
            config->tilt_int_map = BMA5_GET_BITS(reg_value[2], BMA530_TILT_INT_MAP);
            config->orient_int_map = BMA5_GET_BITS(reg_value[2], BMA530_ORIENT_INT_MAP);
            config->acc_foc_int_map = BMA5_GET_BITS(reg_value[2], BMA530_ACC_FOC_INT_MAP);

            config->feat_eng_err_int_map = BMA5_GET_BITS_POS_0(reg_value[3], BMA530_FEAT_ENG_ERR_INT_MAP);
        }
    }

    return result;
}

int8_t bma530_set_int_map(const struct bma530_int_map *config, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    /* Temporary variable to store the register value to be set */
    uint8_t reg_value[4] = { 0 };

    uint8_t acc_drdy_int_map, fifo_wm_int_map, fifo_full_int_map, gen_int1_int_map;
    uint8_t gen_int2_int_map, gen_int3_int_map, step_det_int_map, step_cnt_int_map;
    uint8_t sig_mo_int_map, tilt_int_map, orient_int_map, acc_foc_int_map;

    if (NULL == config)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        result = bma5_get_regs(BMA530_REG_INT_MAP_0, reg_value, 4, dev);

        if (BMA5_OK == result)
        {
            /* Bring up the register value to be set, as per the input details */
            acc_drdy_int_map =
                (BMA5_SET_BITS_POS_0(reg_value[0], BMA530_ACC_DRDY_INT_MAP,
                                     config->acc_drdy_int_map) & BMA530_ACC_DRDY_INT_MAP_MSK);
            fifo_wm_int_map =
                (BMA5_SET_BITS(reg_value[0], BMA530_FIFO_WM_INT_MAP,
                               config->fifo_wm_int_map) & BMA530_FIFO_WM_INT_MAP_MSK);
            fifo_full_int_map =
                (BMA5_SET_BITS(reg_value[0], BMA530_FIFO_FULL_INT_MAP,
                               config->fifo_full_int_map) & BMA530_FIFO_FULL_INT_MAP_MSK);
            gen_int1_int_map =
                (BMA5_SET_BITS(reg_value[0], BMA530_GEN_INT1_INT_MAP,
                               config->gen_int1_int_map) & BMA530_GEN_INT1_INT_MAP_MSK);

            reg_value[0] = (uint8_t)(acc_drdy_int_map | fifo_wm_int_map | fifo_full_int_map | gen_int1_int_map);

            gen_int2_int_map =
                (BMA5_SET_BITS_POS_0(reg_value[1], BMA530_GEN_INT2_INT_MAP,
                                     config->gen_int2_int_map) & BMA530_GEN_INT2_INT_MAP_MSK);
            gen_int3_int_map =
                (BMA5_SET_BITS(reg_value[1], BMA530_GEN_INT3_INT_MAP,
                               config->gen_int3_int_map) & BMA530_GEN_INT3_INT_MAP_MSK);
            step_det_int_map =
                (BMA5_SET_BITS(reg_value[1], BMA530_STEP_DET_INT_MAP,
                               config->step_det_int_map) & BMA530_STEP_DET_INT_MAP_MSK);
            step_cnt_int_map =
                (BMA5_SET_BITS(reg_value[1], BMA530_STEP_CNT_INT_MAP,
                               config->step_cnt_int_map) & BMA530_STEP_CNT_INT_MAP_MSK);

            reg_value[1] = (uint8_t)(gen_int2_int_map | gen_int3_int_map | step_det_int_map | step_cnt_int_map);

            sig_mo_int_map =
                (BMA5_SET_BITS_POS_0(reg_value[2], BMA530_SIG_MO_INT_MAP,
                                     config->sig_mo_int_map) & BMA530_SIG_MO_INT_MAP_MSK);
            tilt_int_map =
                (BMA5_SET_BITS(reg_value[2], BMA530_TILT_INT_MAP, config->tilt_int_map) & BMA530_TILT_INT_MAP_MSK);
            orient_int_map =
                (BMA5_SET_BITS(reg_value[2], BMA530_ORIENT_INT_MAP,
                               config->orient_int_map) & BMA530_ORIENT_INT_MAP_MSK);
            acc_foc_int_map =
                (BMA5_SET_BITS(reg_value[2], BMA530_ACC_FOC_INT_MAP,
                               config->acc_foc_int_map) & BMA530_ACC_FOC_INT_MAP_MSK);

            reg_value[2] = (uint8_t)(sig_mo_int_map | tilt_int_map | orient_int_map | acc_foc_int_map);

            reg_value[3] = BMA5_SET_BITS_POS_0(reg_value[3], BMA530_FEAT_ENG_ERR_INT_MAP, config->feat_eng_err_int_map);

            result = bma5_set_regs(BMA530_REG_INT_MAP_0, reg_value, 4, dev);
        }
    }

    return result;
}

int8_t bma530_get_feat_eng_gp_flags(struct bma530_feat_eng_gp_flags *config, struct bma5_dev *dev)
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
        result = bma5_get_regs(BMA530_REG_FEAT_ENG_GP_FLAGS, &reg_value, sizeof(reg_value), dev);
        if (BMA5_OK == result)
        {
            /* Parse needed details from received serial data */
            config->feat_init_stat = BMA5_GET_BITS_POS_0(reg_value, BMA530_FEAT_INIT_STAT);
            config->foc_running = BMA5_GET_BITS(reg_value, BMA530_FOC_RUNNING);
            config->fifo_size_changed = BMA5_GET_BITS(reg_value, BMA530_FIFO_SIZE_CHANGED);
        }
    }

    return result;
}

int8_t bma530_get_feat_eng_gpr_0(struct bma530_feat_eng_gpr_0 *config, struct bma5_dev *dev)
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
        result = bma5_get_regs(BMA530_REG_FEAT_ENG_GPR_0, &reg_value, sizeof(reg_value), dev);
        if (BMA5_OK == result)
        {
            /* Parse needed details from received serial data */
            config->gen_int1_en = BMA5_GET_BITS_POS_0(reg_value, BMA530_GEN_INT1_EN);
            config->gen_int2_en = BMA5_GET_BITS(reg_value, BMA530_GEN_INT2_EN);
            config->gen_int3_en = BMA5_GET_BITS(reg_value, BMA530_GEN_INT3_EN);
            config->step_en = BMA5_GET_BITS(reg_value, BMA530_STEP_EN);
            config->sig_mo_en = BMA5_GET_BITS(reg_value, BMA530_SIG_MO_EN);
            config->tilt_en = BMA5_GET_BITS(reg_value, BMA530_TILT_EN);
            config->orient_en = BMA5_GET_BITS(reg_value, BMA530_ORIENT_EN);
            config->acc_foc_en = BMA5_GET_BITS(reg_value, BMA530_ACC_FOC_EN);
        }
    }

    return result;
}

int8_t bma530_set_feat_eng_gpr_0(const struct bma530_feat_eng_gpr_0 *config, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    /* Temporary variable to store the register value to be set */
    uint8_t reg_value = 0;

    uint8_t gen_int1_en, gen_int2_en, gen_int3_en, step_en, sig_mo_en, tilt_en, orient_en, acc_foc_en;

    if (NULL == config)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        result = bma5_get_regs(BMA530_REG_FEAT_ENG_GPR_0, &reg_value, sizeof(reg_value), dev);

        if (BMA5_OK == result)
        {
            /* Bring up the register value to be set, as per the input details */
            gen_int1_en =
                (BMA5_SET_BITS_POS_0(reg_value, BMA530_GEN_INT1_EN, config->gen_int1_en) & BMA530_GEN_INT1_EN_MSK);
            gen_int2_en = (BMA5_SET_BITS(reg_value, BMA530_GEN_INT2_EN, config->gen_int2_en) & BMA530_GEN_INT2_EN_MSK);
            gen_int3_en = (BMA5_SET_BITS(reg_value, BMA530_GEN_INT3_EN, config->gen_int3_en) & BMA530_GEN_INT3_EN_MSK);
            step_en = (BMA5_SET_BITS(reg_value, BMA530_STEP_EN, config->step_en) & BMA530_STEP_EN_MSK);
            sig_mo_en = (BMA5_SET_BITS(reg_value, BMA530_SIG_MO_EN, config->sig_mo_en) & BMA530_SIG_MO_EN_MSK);
            tilt_en = (BMA5_SET_BITS(reg_value, BMA530_TILT_EN, config->tilt_en) & BMA530_TILT_EN_MSK);
            orient_en = (BMA5_SET_BITS(reg_value, BMA530_ORIENT_EN, config->orient_en) & BMA530_ORIENT_EN_MSK);
            acc_foc_en = (BMA5_SET_BITS(reg_value, BMA530_ACC_FOC_EN, config->acc_foc_en) & BMA530_ACC_FOC_EN_MSK);

            reg_value =
                (uint8_t)(gen_int1_en | gen_int2_en | gen_int3_en | step_en | sig_mo_en | tilt_en | orient_en |
                          acc_foc_en);

            result = bma5_set_regs(BMA530_REG_FEAT_ENG_GPR_0, (const uint8_t *)&reg_value, sizeof(reg_value), dev);
        }
    }

    return result;
}

int8_t bma530_get_feat_eng_gpr_1(struct bma530_feat_eng_gpr_1 *config, struct bma5_dev *dev)
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
        result = bma5_get_regs(BMA530_REG_FEAT_ENG_GPR_1, &reg_value, sizeof(reg_value), dev);
        if (BMA5_OK == result)
        {
            /* Parse needed details from received serial data */
            config->gen_int1_data_src = BMA5_GET_BITS_POS_0(reg_value, BMA530_GEN_INT1_DATA_SRC);
            config->gen_int2_data_src = BMA5_GET_BITS(reg_value, BMA530_GEN_INT2_DATA_SRC);
            config->gen_int3_data_src = BMA5_GET_BITS(reg_value, BMA530_GEN_INT3_DATA_SRC);
        }
    }

    return result;
}

int8_t bma530_set_feat_eng_gpr_1(const struct bma530_feat_eng_gpr_1 *config, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    /* Temporary variable to store the register value to be set */
    uint8_t reg_value = 0;

    uint8_t gen_int1_data_src, gen_int2_data_src, gen_int3_data_src;

    if (NULL == config)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        result = bma5_get_regs(BMA530_REG_FEAT_ENG_GPR_1, &reg_value, sizeof(reg_value), dev);

        if (BMA5_OK == result)
        {
            /* Bring up the register value to be set, as per the input details */
            gen_int1_data_src =
                (BMA5_SET_BITS_POS_0(reg_value, BMA530_GEN_INT1_DATA_SRC,
                                     config->gen_int1_data_src) & BMA530_GEN_INT1_DATA_SRC_MSK);
            gen_int2_data_src =
                (BMA5_SET_BITS(reg_value, BMA530_GEN_INT2_DATA_SRC,
                               config->gen_int2_data_src) & BMA530_GEN_INT2_DATA_SRC_MSK);
            gen_int3_data_src =
                (BMA5_SET_BITS(reg_value, BMA530_GEN_INT3_DATA_SRC,
                               config->gen_int3_data_src) & BMA530_GEN_INT3_DATA_SRC_MSK);

            reg_value = (uint8_t)(gen_int1_data_src | gen_int2_data_src | gen_int3_data_src);

            result = bma5_set_regs(BMA530_REG_FEAT_ENG_GPR_1, (const uint8_t *)&reg_value, sizeof(reg_value), dev);
        }
    }

    return result;
}

int8_t bma530_get_feat_eng_gpr_2(uint8_t *step_cnt_out_0, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    /* Temporary variable to carry the register value */
    uint8_t reg_value;

    if (NULL == step_cnt_out_0)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        result = bma5_get_regs(BMA530_REG_FEAT_ENG_GPR_2, &reg_value, sizeof(reg_value), dev);
        if (BMA5_OK == result)
        {
            /* Parse needed details from received serial data */
            *step_cnt_out_0 = BMA5_GET_BITS_POS_0(reg_value, BMA530_STEP_CNT_OUT_0);
        }
    }

    return result;
}

int8_t bma530_get_feat_eng_gpr_3(uint8_t *step_cnt_out_1, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    /* Temporary variable to carry the register value */
    uint8_t reg_value;

    if (NULL == step_cnt_out_1)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        result = bma5_get_regs(BMA530_REG_FEAT_ENG_GPR_3, &reg_value, sizeof(reg_value), dev);
        if (BMA5_OK == result)
        {
            /* Parse needed details from received serial data */
            *step_cnt_out_1 = BMA5_GET_BITS_POS_0(reg_value, BMA530_STEP_CNT_OUT_1);
        }
    }

    return result;
}

int8_t bma530_get_feat_eng_gpr_4(uint8_t *step_cnt_out_2, struct bma5_dev *dev)
{
    /* Function execution status */
    int8_t result;

    /* Temporary variable to carry the register value */
    uint8_t reg_value;

    if (NULL == step_cnt_out_2)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        result = bma5_get_regs(BMA530_REG_FEAT_ENG_GPR_4, &reg_value, sizeof(reg_value), dev);
        if (BMA5_OK == result)
        {
            /* Parse needed details from received serial data */
            *step_cnt_out_2 = BMA5_GET_BITS_POS_0(reg_value, BMA530_STEP_CNT_OUT_2);
        }
    }

    return result;
}

int8_t bma530_get_feat_eng_gpr_5(struct bma530_feat_eng_gpr_5 *config, struct bma5_dev *dev)
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
        result = bma5_get_regs(BMA530_REG_FEAT_ENG_GPR_5, &reg_value, sizeof(reg_value), dev);
        if (BMA5_OK == result)
        {
            /* Parse needed details from received serial data */
            config->orientation_portrait_landscape = BMA5_GET_BITS_POS_0(reg_value,
                                                                         BMA530_ORIENTATION_PORTRAIT_LANDSCAPE);
            config->orientation_face_up_down = BMA5_GET_BITS(reg_value, BMA530_ORIENTATION_FACE_UP_DOWN);
            config->activ_stat = BMA5_GET_BITS(reg_value, BMA530_ACTIV_STAT);
            config->gen_int1_stat = BMA5_GET_BITS(reg_value, BMA530_GEN_INT1_STAT);
            config->gen_int2_stat = BMA5_GET_BITS(reg_value, BMA530_GEN_INT2_STAT);
            config->gen_int3_stat = BMA5_GET_BITS(reg_value, BMA530_GEN_INT3_STAT);
        }
    }

    return result;
}
