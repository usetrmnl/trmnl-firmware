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
* @file       bma530_features.c
* @date       2024-07-29
* @version    v4.2.0
*
*/

/******************************************************************************/
/****************************** Header files **********************************/
/******************************************************************************/

#include "bma530_features.h"

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

/******************************************************************************/
/*********************** User function definitions ****************************/
/******************************************************************************/

/*!
 *  @brief This API is the entry point.
 *  Call this API before using all other APIs.
 *  This API reads the chip-id of the sensor which is the first step to
 *  verify the sensor and also it configures the read mechanism of SPI and
 *  I2C interface.
 */
int8_t bma530_init(struct bma5_dev *dev)
{
    /* Variable to store the function result */
    int8_t result;

    /* Variable to get chip id */
    uint8_t data = 0;

    /* Null-pointer check */
    result = verify_handle(dev);

    if (result == BMA5_OK)
    {
        dev->chip_id = 0;

        if (dev->intf == BMA5_SPI_INTF)
        {
            dev->dummy_byte = 1;
        }
        else
        {
            dev->dummy_byte = 0;
        }

        /* Dummy read results in NACK. Hence result is not checked */
        (void) bma5_get_regs(BMA530_REG_CHIP_ID, &data, 1, dev);

        result = bma5_get_regs(BMA530_REG_CHIP_ID, &data, 1, dev);

        if (result == BMA5_OK)
        {
            /* Assign Chip Id */
            dev->chip_id = data;
        }

        if (dev->chip_id != BMA530_CHIP_ID)
        {
            result = BMA5_E_DEV_NOT_FOUND;
        }
    }
    else
    {
        result = BMA5_E_NULL_PTR;
    }

    return result;
}

/*!
 * @brief This API gets feature output status from the register.
 */
int8_t bma530_get_feat_eng_feature_out(struct bma530_feat_eng_feat_out *feat_out, struct bma5_dev *dev)
{
    /* Variable to store the function result */
    int8_t result;

    /* Array to store feature engine GPR feature outputs */
    uint8_t reg_data[5];

    /* Array to store step counter data */
    uint8_t step_cntr[3];

    if (feat_out == NULL)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        /* Get the feature engine GPR feature output data */
        result = bma5_get_regs(BMA530_REG_FEAT_ENG_GPR_1, reg_data, 5, dev);

        if (result == BMA5_OK)
        {
            step_cntr[0] = BMA5_GET_BITS_POS_0(reg_data[1], BMA530_STEP_CNT_OUT_0);
            step_cntr[1] = BMA5_GET_BITS_POS_0(reg_data[2], BMA530_STEP_CNT_OUT_1);
            step_cntr[2] = BMA5_GET_BITS_POS_0(reg_data[3], BMA530_STEP_CNT_OUT_2);

            feat_out->step_cntr_out =
                (uint32_t)(step_cntr[0] | ((uint16_t)step_cntr[1] << 8) | ((uint32_t)step_cntr[2] << 16));

            feat_out->orientation_portrait_landscape = BMA5_GET_BITS_POS_0(reg_data[4],
                                                                           BMA530_ORIENTATION_PORTRAIT_LANDSCAPE);
            feat_out->activ_stat = BMA5_GET_BITS(reg_data[4], BMA530_ACTIV_STAT);

            feat_out->orientation_face_up_down = BMA5_GET_BITS(reg_data[4], BMA530_ORIENTATION_FACE_UP_DOWN);

            feat_out->gen_int1_stat = BMA5_GET_BITS(reg_data[4], BMA530_GEN_INT1_STAT);

            feat_out->gen_int2_stat = BMA5_GET_BITS(reg_data[4], BMA530_GEN_INT2_STAT);

            feat_out->gen_int3_stat = BMA5_GET_BITS(reg_data[4], BMA530_GEN_INT3_STAT);
        }
    }

    return result;
}

/*!
 * @brief This API sets feature configuration error status
 */
int8_t bma530_set_feat_conf_err(const struct bma530_feat_conf_err *feat_conf_err, struct bma5_dev *dev)
{
    /* Variable to define error */
    int8_t result = BMA5_OK;

    /* Variable to store base address of feature config error */
    uint8_t data = BMA530_BASE_ADDR_FEAT_CONF_ERR;

    /* Array to store feature config error data */
    uint8_t feat_conf_err_data[4] = { 0 };

    if (feat_conf_err == NULL)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        /* Set the feature config error base address to feature engine transmission address to start DMA transaction */
        result = bma5_set_regs(BMA5_REG_FEATURE_DATA_ADDR, &data, 1, dev);

        if (result == BMA5_OK)
        {
            /* Get the configuration from the feature engine register */
            result = bma5_get_regs(BMA5_REG_FEATURE_DATA_TX, feat_conf_err_data, 4, dev);

            if (result == BMA5_OK)
            {
                feat_conf_err_data[2] = BMA5_SET_BITS(feat_conf_err_data[2],
                                                      BMA530_GEN_INT1_CONF_ERR,
                                                      feat_conf_err->gen_int1_conf_err);

                feat_conf_err_data[2] = BMA5_SET_BITS(feat_conf_err_data[2],
                                                      BMA530_GEN_INT2_CONF_ERR,
                                                      feat_conf_err->gen_int2_conf_err);

                feat_conf_err_data[2] = BMA5_SET_BITS(feat_conf_err_data[2],
                                                      BMA530_GEN_INT3_CONF_ERR,
                                                      feat_conf_err->gen_int3_conf_err);

                feat_conf_err_data[2] = BMA5_SET_BITS(feat_conf_err_data[2],
                                                      BMA530_STEP_CONF_ERR,
                                                      feat_conf_err->step_conf_err);

                feat_conf_err_data[2] = BMA5_SET_BITS(feat_conf_err_data[2],
                                                      BMA530_SIG_MO_CONF_ERR,
                                                      feat_conf_err->sig_mo_conf_err);

                feat_conf_err_data[2] = BMA5_SET_BITS(feat_conf_err_data[2],
                                                      BMA530_TILT_CONF_ERR,
                                                      feat_conf_err->tilt_conf_err);

                feat_conf_err_data[2] = BMA5_SET_BITS(feat_conf_err_data[2],
                                                      BMA530_ORIENT_CONF_ERR,
                                                      feat_conf_err->orient_conf_err);

                feat_conf_err_data[2] = BMA5_SET_BITS(feat_conf_err_data[2],
                                                      BMA530_ACC_FOC_CONF_ERR,
                                                      feat_conf_err->acc_foc_conf_err);

                feat_conf_err_data[0] = feat_conf_err_data[2];
                feat_conf_err_data[1] = feat_conf_err_data[3];

                /* Set the configuration from the feature engine register */
                result = bma5_set_regs(BMA5_REG_FEATURE_DATA_TX, feat_conf_err_data, 2, dev);
            }
        }
    }

    return result;
}

/*!
 * @brief This API gets feature configuration error status
 */
int8_t bma530_get_feat_conf_err(struct bma530_feat_conf_err *feat_conf_err, struct bma5_dev *dev)
{
    /* Variable to define error */
    int8_t result = BMA5_OK;

    /* Variable to store base address of feature config error */
    uint8_t data = BMA530_BASE_ADDR_FEAT_CONF_ERR;

    /* Array to store feature config error data */
    uint8_t feat_conf_err_data[4] = { 0 };

    if (feat_conf_err == NULL)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        /* Set the feature config error base address to feature engine transmission address to start DMA transaction */
        result = bma5_set_regs(BMA5_REG_FEATURE_DATA_ADDR, &data, 1, dev);

        if (result == BMA5_OK)
        {
            /* Get the configuration from the feature engine register */
            result = bma5_get_regs(BMA5_REG_FEATURE_DATA_TX, feat_conf_err_data, 4, dev);

            if (result == BMA5_OK)
            {
                feat_conf_err->gen_int1_conf_err = BMA5_GET_BITS(feat_conf_err_data[2], BMA530_GEN_INT1_CONF_ERR);
                feat_conf_err->gen_int2_conf_err = BMA5_GET_BITS(feat_conf_err_data[2], BMA530_GEN_INT2_CONF_ERR);
                feat_conf_err->gen_int3_conf_err = BMA5_GET_BITS(feat_conf_err_data[2], BMA530_GEN_INT3_CONF_ERR);
                feat_conf_err->step_conf_err = BMA5_GET_BITS(feat_conf_err_data[2], BMA530_STEP_CONF_ERR);
                feat_conf_err->sig_mo_conf_err = BMA5_GET_BITS(feat_conf_err_data[2], BMA530_SIG_MO_CONF_ERR);
                feat_conf_err->tilt_conf_err = BMA5_GET_BITS(feat_conf_err_data[2], BMA530_TILT_CONF_ERR);
                feat_conf_err->orient_conf_err = BMA5_GET_BITS(feat_conf_err_data[2], BMA530_ORIENT_CONF_ERR);
                feat_conf_err->acc_foc_conf_err = BMA5_GET_BITS(feat_conf_err_data[2], BMA530_ACC_FOC_CONF_ERR);

            }
        }
    }

    return result;
}

/*!
 * @brief This API sets android compatibility mode
 */
int8_t bma530_set_android_comp_mode(const uint8_t *android_comp, struct bma5_dev *dev)
{
    /* Variable to define error */
    int8_t result = BMA5_OK;

    /* Variable to store base address of android compatibility mode */
    uint8_t data = BMA530_BASE_ADDR_GENERAL_SETTINGS;

    /* Array to store android compatibility mode data */
    uint8_t android_comp_data[4] = { 0 };

    if (android_comp == NULL)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        /* Set the feature axis base address to feature engine transmission address to start DMA transaction */
        result = bma5_set_regs(BMA5_REG_FEATURE_DATA_ADDR, &data, 1, dev);

        if (result == BMA5_OK)
        {
            /* Get the configuration from the feature engine register */
            result = bma5_get_regs(BMA5_REG_FEATURE_DATA_TX, android_comp_data, 4, dev);

            if (result == BMA5_OK)
            {
                android_comp_data[2] = BMA5_SET_BITS_POS_0(android_comp_data[2], BMA530_ANDROID_COMP, (*android_comp));

                android_comp_data[0] = android_comp_data[2];
                android_comp_data[1] = android_comp_data[3];

                /* Set the configuration from the feature engine register */
                result = bma5_set_regs(BMA5_REG_FEATURE_DATA_TX, android_comp_data, 2, dev);
            }
        }
    }

    return result;
}

/*!
 * @brief This API gets android compatibility mode
 */
int8_t bma530_get_android_comp_mode(uint8_t *android_comp, struct bma5_dev *dev)
{
    /* Variable to define error */
    int8_t result = BMA5_OK;

    /* Variable to store base address of android compatibility mode */
    uint8_t data = BMA530_BASE_ADDR_GENERAL_SETTINGS;

    /* Array to store android compatibility mode data */
    uint8_t android_comp_data[4] = { 0 };

    if (android_comp == NULL)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        /* Set the feature axis base address to feature engine transmission address to start DMA transaction */
        result = bma5_set_regs(BMA5_REG_FEATURE_DATA_ADDR, &data, 1, dev);

        if (result == BMA5_OK)
        {
            /* Get the configuration from the feature engine register */
            result = bma5_get_regs(BMA5_REG_FEATURE_DATA_TX, android_comp_data, 4, dev);

            if (result == BMA5_OK)
            {
                (*android_comp) = BMA5_GET_BITS_POS_0(android_comp_data[2], BMA530_ANDROID_COMP);
            }
        }
    }

    return result;
}

/*!
 * @brief This API sets step counter configuration
 */
int8_t bma530_set_step_counter_config(const struct bma530_step_cntr *step_cntr, struct bma5_dev *dev)
{
    /* Variable to define error */
    int8_t result = BMA5_OK;

    /* Variable to store base address of step counter */
    uint8_t data = BMA530_BASE_ADDR_STEP_COUNTER;

    /* Array to store step counter data */
    uint8_t sc_data[38] = { 0 };

    /* Variable to store the data to be written in register */
    uint16_t watermark_level_1, watermark_level_2, reset_counter, sd_en, sc_en;
    uint16_t envelope_up_thres_1, envelope_up_thres_2, envelope_up_decay_coeff_1, envelope_up_decay_coeff_2;
    uint16_t envelope_down_thres_1, envelope_down_thres_2, envelope_down_decay_coeff_1, envelope_down_decay_coeff_2;
    uint16_t acc_mean_decay_coeff_1, acc_mean_decay_coeff_2, step_dur_mean_decay_coeff_1, step_dur_mean_decay_coeff_2;
    uint16_t step_buffer_size, filter_cascade_enabled, step_counter_increment_1, step_counter_increment_2;
    uint16_t en_half_step, peak_duration_min_walking, peak_duration_min_running, activity_detection_factor;
    uint16_t activity_detection_thres_1, activity_detection_thres_2, step_duration_max, step_duration_window,
             en_step_dur_pp, step_dur_thres;
    uint16_t en_mcr_pp, mcr_thres_1, mcr_thres_2, filter_coeff_b_2_1, filter_coeff_b_2_2, filter_coeff_b_1_1,
             filter_coeff_b_1_2;
    uint16_t filter_coeff_b_0_1, filter_coeff_b_0_2, filter_coeff_a_2_1, filter_coeff_a_2_2;
    uint16_t filter_coeff_a_1_1, filter_coeff_a_1_2, filter_coeff_scale_a, filter_coeff_scale_b;

    if (step_cntr == NULL)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        /* Set the step counter base address to feature engine transmission address to start DMA transaction */
        result = bma5_set_regs(BMA5_REG_FEATURE_DATA_ADDR, &data, 1, dev);

        if (result == BMA5_OK)
        {
            /* Get the configuration from the feature engine register */
            result = bma5_get_regs(BMA5_REG_FEATURE_DATA_TX, sc_data, 38, dev);

            if (result == BMA5_OK)
            {
                /* Settings 1 */
                watermark_level_1 =
                    (BMA5_SET_BITS_POS_0(sc_data[2], BMA530_SC_WATERMARK_LVL,
                                         step_cntr->watermark_level) & BMA530_SC_WATERMARK_LVL_MSK);

                watermark_level_2 = (uint16_t)(sc_data[3] << 8);

                watermark_level_2 =
                    (BMA5_SET_BITS_POS_0(watermark_level_2, BMA530_SC_WATERMARK_LVL,
                                         step_cntr->watermark_level) & BMA530_SC_WATERMARK_LVL_MSK);

                reset_counter =
                    (BMA5_SET_BITS(sc_data[3], BMA530_SC_RESET_COUNTER,
                                   step_cntr->reset_counter) & BMA530_SC_RESET_COUNTER_MSK);

                sd_en = (BMA5_SET_BITS(sc_data[3], BMA530_SC_SD_EN, step_cntr->sd_en) & BMA530_SC_SD_EN_MSK);

                sc_en = (BMA5_SET_BITS(sc_data[3], BMA530_SC_SC_EN, step_cntr->sc_en) & BMA530_SC_SC_EN_MSK);

                /* Settings 2 */
                envelope_up_thres_1 =
                    (BMA5_SET_BITS_POS_0(sc_data[4], BMA530_SC_ENVELOPE_UP_THRES,
                                         step_cntr->envelope_up_thres) & BMA530_SC_ENVELOPE_UP_THRES_MSK);

                envelope_up_thres_2 = (uint16_t)(sc_data[5] << 8);

                envelope_up_thres_2 =
                    (BMA5_SET_BITS_POS_0(envelope_up_thres_2, BMA530_SC_ENVELOPE_UP_THRES,
                                         step_cntr->envelope_up_thres) & BMA530_SC_ENVELOPE_UP_THRES_MSK);

                /* Settings 3 */
                envelope_up_decay_coeff_1 =
                    (BMA5_SET_BITS_POS_0(sc_data[6], BMA530_SC_ENVELOPE_UP_DECAY_COEFF,
                                         step_cntr->envelope_up_decay_coeff) & BMA530_SC_ENVELOPE_UP_DECAY_COEFF_MSK);

                envelope_up_decay_coeff_2 = (uint16_t)(sc_data[7] << 8);

                envelope_up_decay_coeff_2 =
                    (BMA5_SET_BITS_POS_0(envelope_up_decay_coeff_2, BMA530_SC_ENVELOPE_UP_DECAY_COEFF,
                                         step_cntr->envelope_up_decay_coeff) & BMA530_SC_ENVELOPE_UP_DECAY_COEFF_MSK);

                /* Settings 4 */
                envelope_down_thres_1 =
                    (BMA5_SET_BITS_POS_0(sc_data[8], BMA530_SC_ENVELOPE_DOWN_THRES,
                                         step_cntr->envelope_down_thres) & BMA530_SC_ENVELOPE_DOWN_THRES_MSK);

                envelope_down_thres_2 = (uint16_t)(sc_data[9] << 8);

                envelope_down_thres_2 =
                    (BMA5_SET_BITS_POS_0(envelope_down_thres_2, BMA530_SC_ENVELOPE_DOWN_THRES,
                                         step_cntr->envelope_down_thres) & BMA530_SC_ENVELOPE_DOWN_THRES_MSK);

                /* Settings 5 */
                envelope_down_decay_coeff_1 =
                    (BMA5_SET_BITS_POS_0(sc_data[10], BMA530_SC_ENVELOPE_DOWN_DECAY_COEFF,
                                         step_cntr->envelope_down_decay_coeff) &
                     BMA530_SC_ENVELOPE_DOWN_DECAY_COEFF_MSK);

                envelope_down_decay_coeff_2 = (uint16_t)(sc_data[11] << 8);

                envelope_down_decay_coeff_2 =
                    (BMA5_SET_BITS_POS_0(envelope_down_decay_coeff_2, BMA530_SC_ENVELOPE_DOWN_DECAY_COEFF,
                                         step_cntr->envelope_down_decay_coeff) &
                     BMA530_SC_ENVELOPE_DOWN_DECAY_COEFF_MSK);

                /* Settings 6 */
                acc_mean_decay_coeff_1 =
                    (BMA5_SET_BITS_POS_0(sc_data[12], BMA530_SC_ACC_MEAN_DECAY_COEFF,
                                         step_cntr->acc_mean_decay_coeff) & BMA530_SC_ACC_MEAN_DECAY_COEFF_MSK);

                acc_mean_decay_coeff_2 = (uint16_t)(sc_data[13] << 8);

                acc_mean_decay_coeff_2 =
                    (BMA5_SET_BITS_POS_0(acc_mean_decay_coeff_2, BMA530_SC_ACC_MEAN_DECAY_COEFF,
                                         step_cntr->acc_mean_decay_coeff) & BMA530_SC_ACC_MEAN_DECAY_COEFF_MSK);

                /* Settings 7 */
                step_dur_mean_decay_coeff_1 =
                    (BMA5_SET_BITS_POS_0(sc_data[14], BMA530_SC_STEP_DUR_MEAN_DECAY_COEFF,
                                         step_cntr->step_dur_mean_decay_coeff) &
                     BMA530_SC_STEP_DUR_MEAN_DECAY_COEFF_MSK);

                step_dur_mean_decay_coeff_2 = (uint16_t)(sc_data[15] << 8);

                step_dur_mean_decay_coeff_2 =
                    (BMA5_SET_BITS_POS_0(step_dur_mean_decay_coeff_2, BMA530_SC_STEP_DUR_MEAN_DECAY_COEFF,
                                         step_cntr->step_dur_mean_decay_coeff) &
                     BMA530_SC_STEP_DUR_MEAN_DECAY_COEFF_MSK);

                /* Settings 8 */
                step_buffer_size =
                    (BMA5_SET_BITS_POS_0(sc_data[16], BMA530_SC_STEP_BUFFER_SIZE,
                                         step_cntr->step_buffer_size) & BMA530_SC_STEP_BUFFER_SIZE_MSK);

                filter_cascade_enabled =
                    (BMA5_SET_BITS(sc_data[16], BMA530_SC_FILTER_CASCADE_ENABLED,
                                   step_cntr->filter_cascade_enabled) & BMA530_SC_FILTER_CASCADE_ENABLED_MSK);

                step_counter_increment_1 =
                    (BMA5_SET_BITS(sc_data[16], BMA530_SC_STEP_COUNTER_INCREAMENT,
                                   step_cntr->step_counter_increment) & BMA530_SC_STEP_COUNTER_INCREAMENT_MSK);

                step_counter_increment_2 = (uint16_t)(sc_data[17] << 8);

                step_counter_increment_2 =
                    (BMA5_SET_BITS(step_counter_increment_2, BMA530_SC_STEP_COUNTER_INCREAMENT,
                                   step_cntr->step_counter_increment) & BMA530_SC_STEP_COUNTER_INCREAMENT_MSK);

                en_half_step =
                    (BMA5_SET_BITS(sc_data[17], BMA530_SC_EN_HALF_STEP,
                                   step_cntr->en_half_step) & BMA530_SC_EN_HALF_STEP_MSK);

                /* Settings 9 */
                peak_duration_min_walking =
                    (BMA5_SET_BITS_POS_0(sc_data[18], BMA530_SC_PEAK_DUR_MIN_WALKING,
                                         step_cntr->peak_duration_min_walking) & BMA530_SC_PEAK_DUR_MIN_WALKING_MSK);

                peak_duration_min_running =
                    (BMA5_SET_BITS(sc_data[19], BMA530_SC_PEAK_DUR_MIN_RUNNING,
                                   step_cntr->peak_duration_min_running) & BMA530_SC_PEAK_DUR_MIN_RUNNING_MSK);

                /* Settings 10 */
                activity_detection_factor =
                    (BMA5_SET_BITS_POS_0(sc_data[20], BMA530_SC_ACTIVITY_DET_FACT,
                                         step_cntr->activity_detection_factor) & BMA530_SC_ACTIVITY_DET_FACT_MSK);

                activity_detection_thres_1 =
                    (BMA5_SET_BITS(sc_data[20], BMA530_SC_ACTIVITY_DET_THRES,
                                   step_cntr->activity_detection_thres) & BMA530_SC_ACTIVITY_DET_THRES_MSK);

                activity_detection_thres_2 = (uint16_t)(sc_data[21] << 8);

                activity_detection_thres_2 =
                    (BMA5_SET_BITS(activity_detection_thres_2, BMA530_SC_ACTIVITY_DET_THRES,
                                   step_cntr->activity_detection_thres) & BMA530_SC_ACTIVITY_DET_THRES_MSK);

                /* Settings 11 */
                step_duration_max =
                    (BMA5_SET_BITS_POS_0(sc_data[22], BMA530_SC_STEP_DUR_MAX,
                                         step_cntr->step_duration_max) & BMA530_SC_STEP_DUR_MAX_MSK);

                step_duration_window =
                    (BMA5_SET_BITS(sc_data[21], BMA530_SC_STEP_DUR_WINDOW,
                                   step_cntr->step_duration_window) & BMA530_SC_STEP_DUR_WINDOW_MSK);

                /* Settings 12 */
                en_step_dur_pp =
                    (BMA5_SET_BITS_POS_0(sc_data[23], BMA530_SC_EN_STEP_DUR_PP,
                                         step_cntr->en_step_dur_pp) & BMA530_SC_EN_STEP_DUR_PP_MSK);

                step_dur_thres =
                    (BMA5_SET_BITS(sc_data[24], BMA530_SC_STEP_DUR_THRES,
                                   step_cntr->step_dur_thres) & BMA530_SC_STEP_DUR_THRES_MSK);

                en_mcr_pp =
                    (BMA5_SET_BITS(sc_data[24], BMA530_SC_EN_MCR_PP, step_cntr->en_mcr_pp) & BMA530_SC_EN_MCR_PP_MSK);

                mcr_thres_1 =
                    (BMA5_SET_BITS(sc_data[24], BMA530_SC_MCR_THRES, step_cntr->mcr_thres) & BMA530_SC_MCR_THRES_MSK);

                mcr_thres_2 = (uint16_t)(sc_data[25] << 8);

                mcr_thres_2 =
                    (BMA5_SET_BITS(mcr_thres_2, BMA530_SC_MCR_THRES, step_cntr->mcr_thres) & BMA530_SC_MCR_THRES_MSK);

                /* Settings 13 */
                filter_coeff_b_2_1 =
                    (BMA5_SET_BITS_POS_0(sc_data[26], BMA530_SC_FILTER_COEFF_B_2,
                                         step_cntr->filter_coeff_b_2) & BMA530_SC_FILTER_COEFF_B_2_MSK);

                filter_coeff_b_2_2 = (uint16_t)(sc_data[27] << 8);

                filter_coeff_b_2_2 =
                    (BMA5_SET_BITS_POS_0(filter_coeff_b_2_2, BMA530_SC_FILTER_COEFF_B_2,
                                         step_cntr->filter_coeff_b_2) & BMA530_SC_FILTER_COEFF_B_2_MSK);

                /* Settings 14 */
                filter_coeff_b_1_1 =
                    (BMA5_SET_BITS_POS_0(sc_data[28], BMA530_SC_FILTER_COEFF_B_1,
                                         step_cntr->filter_coeff_b_1) & BMA530_SC_FILTER_COEFF_B_1_MSK);

                filter_coeff_b_1_2 = (uint16_t)(sc_data[29] << 8);

                filter_coeff_b_1_2 =
                    (BMA5_SET_BITS_POS_0(filter_coeff_b_1_2, BMA530_SC_FILTER_COEFF_B_1,
                                         step_cntr->filter_coeff_b_1) & BMA530_SC_FILTER_COEFF_B_1_MSK);

                /* Settings 15 */
                filter_coeff_b_0_1 =
                    (BMA5_SET_BITS_POS_0(sc_data[30], BMA530_SC_FILTER_COEFF_B_0,
                                         step_cntr->filter_coeff_b_0) & BMA530_SC_FILTER_COEFF_B_0_MSK);

                filter_coeff_b_0_2 = (uint16_t)(sc_data[31] << 8);

                filter_coeff_b_0_2 =
                    (BMA5_SET_BITS_POS_0(filter_coeff_b_0_2, BMA530_SC_FILTER_COEFF_B_0,
                                         step_cntr->filter_coeff_b_0) & BMA530_SC_FILTER_COEFF_B_0_MSK);

                /* Settings 16 */
                filter_coeff_a_2_1 =
                    (BMA5_SET_BITS_POS_0(sc_data[32], BMA530_SC_FILTER_COEFF_A_2,
                                         step_cntr->filter_coeff_a_2) & BMA530_SC_FILTER_COEFF_A_2_MSK);

                filter_coeff_a_2_2 = (uint16_t)(sc_data[33] << 8);

                filter_coeff_a_2_2 =
                    (BMA5_SET_BITS_POS_0(filter_coeff_a_2_2, BMA530_SC_FILTER_COEFF_A_2,
                                         step_cntr->filter_coeff_a_2) & BMA530_SC_FILTER_COEFF_A_2_MSK);

                /* Settings 17 */
                filter_coeff_a_1_1 =
                    (BMA5_SET_BITS_POS_0(sc_data[34], BMA530_SC_FILTER_COEFF_A_1,
                                         step_cntr->filter_coeff_a_1) & BMA530_SC_FILTER_COEFF_A_1_MSK);

                filter_coeff_a_1_2 = (uint16_t)(sc_data[35] << 8);

                filter_coeff_a_1_2 =
                    (BMA5_SET_BITS_POS_0(filter_coeff_a_1_2, BMA530_SC_FILTER_COEFF_A_1,
                                         step_cntr->filter_coeff_a_1) & BMA530_SC_FILTER_COEFF_A_1_MSK);

                /* Settings 18 */
                filter_coeff_scale_a =
                    (BMA5_SET_BITS_POS_0(sc_data[36], BMA530_SC_FILTER_COEFF_SCALE_A,
                                         step_cntr->filter_coeff_scale_a) & BMA530_SC_FILTER_COEFF_SCALE_A_MSK);

                filter_coeff_scale_b =
                    (BMA5_SET_BITS(sc_data[37], BMA530_SC_FILTER_COEFF_SCALE_B,
                                   step_cntr->filter_coeff_scale_b) & BMA530_SC_FILTER_COEFF_SCALE_B_MSK);

                sc_data[0] = (uint8_t)watermark_level_1;
                sc_data[1] = (uint8_t)((watermark_level_2 | reset_counter | sd_en | sc_en) >> 8);
                sc_data[2] = (uint8_t)envelope_up_thres_1;
                sc_data[3] = (uint8_t)(envelope_up_thres_2 >> 8);
                sc_data[4] = (uint8_t)envelope_up_decay_coeff_1;
                sc_data[5] = (uint8_t)(envelope_up_decay_coeff_2 >> 8);
                sc_data[6] = (uint8_t)envelope_down_thres_1;
                sc_data[7] = (uint8_t)(envelope_down_thres_2 >> 8);
                sc_data[8] = (uint8_t)envelope_down_decay_coeff_1;
                sc_data[9] = (uint8_t)(envelope_down_decay_coeff_2 >> 8);
                sc_data[10] = (uint8_t)acc_mean_decay_coeff_1;
                sc_data[11] = (uint8_t)(acc_mean_decay_coeff_2 >> 8);
                sc_data[12] = (uint8_t)step_dur_mean_decay_coeff_1;
                sc_data[13] = (uint8_t)(step_dur_mean_decay_coeff_2 >> 8);
                sc_data[14] = (uint8_t)(step_buffer_size | filter_cascade_enabled | step_counter_increment_1);
                sc_data[15] = (uint8_t)((step_counter_increment_2 | en_half_step) >> 8);
                sc_data[16] = (uint8_t)peak_duration_min_walking;
                sc_data[17] = (uint8_t)(peak_duration_min_running >> 8);
                sc_data[18] = (uint8_t)(activity_detection_factor | activity_detection_thres_1);
                sc_data[19] = (uint8_t)(activity_detection_thres_2 >> 8);
                sc_data[20] = (uint8_t)step_duration_max;
                sc_data[21] = (uint8_t)(step_duration_window >> 8);
                sc_data[22] = (uint8_t)(en_step_dur_pp | step_dur_thres | en_mcr_pp | mcr_thres_1);
                sc_data[23] = (uint8_t)(mcr_thres_2 >> 8);
                sc_data[24] = (uint8_t)filter_coeff_b_2_1;
                sc_data[25] = (uint8_t)(filter_coeff_b_2_2 >> 8);
                sc_data[26] = (uint8_t)filter_coeff_b_1_1;
                sc_data[27] = (uint8_t)(filter_coeff_b_1_2 >> 8);
                sc_data[28] = (uint8_t)filter_coeff_b_0_1;
                sc_data[29] = (uint8_t)(filter_coeff_b_0_2 >> 8);
                sc_data[30] = (uint8_t)filter_coeff_a_2_1;
                sc_data[31] = (uint8_t)(filter_coeff_a_2_2 >> 8);
                sc_data[32] = (uint8_t)filter_coeff_a_1_1;
                sc_data[33] = (uint8_t)(filter_coeff_a_1_2 >> 8);
                sc_data[34] = (uint8_t)filter_coeff_scale_a;
                sc_data[35] = (uint8_t)(filter_coeff_scale_b >> 8);

                /* Set the configuration from the feature engine register */
                result = bma5_set_regs(BMA5_REG_FEATURE_DATA_TX, sc_data, 36, dev);
            }
        }
    }

    return result;
}

/*!
 * @brief This API gets step counter configuration
 */
int8_t bma530_get_step_counter_config(struct bma530_step_cntr *step_cntr, struct bma5_dev *dev)
{
    /* Variable to define error */
    int8_t result = BMA5_OK;

    /* Variable to store base address of step counter */
    uint8_t data = BMA530_BASE_ADDR_STEP_COUNTER;

    /* Array to store step counter data */
    uint8_t sc_data[38] = { 0 };

    /* Variable to define array offset */
    uint8_t idx;

    /* Variable to define LSB */
    uint16_t lsb;

    /* Variable to define MSB */
    uint16_t msb;

    /* Variable to define a word */
    uint16_t lsb_msb;

    if (step_cntr == NULL)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        /* Set the step counter base address to feature engine transmission address to start DMA transaction */
        result = bma5_set_regs(BMA5_REG_FEATURE_DATA_ADDR, &data, 1, dev);

        if (result == BMA5_OK)
        {
            /* Get the configuration from the feature engine register */
            result = bma5_get_regs(BMA5_REG_FEATURE_DATA_TX, sc_data, 38, dev);

            if (result == BMA5_OK)
            {
                /* First two bytes are dummy bytes */
                idx = 2;

                /* Settings 1 */
                /* Get word to calculate watermark_level, reset_counter, sd_en, and sc_en select from same word */
                lsb = (uint16_t) sc_data[idx++];
                msb = ((uint16_t) sc_data[idx++] << 8);
                lsb_msb = (uint16_t)(lsb | msb);

                step_cntr->watermark_level = lsb_msb & BMA530_SC_WATERMARK_LVL_MSK;

                step_cntr->reset_counter = (lsb_msb & BMA530_SC_RESET_COUNTER_MSK) >> BMA530_SC_RESET_COUNTER_POS;

                step_cntr->sd_en = (lsb_msb & BMA530_SC_SD_EN_MSK) >> BMA530_SC_SD_EN_POS;

                step_cntr->sc_en = (lsb_msb & BMA530_SC_SC_EN_MSK) >> BMA530_SC_SC_EN_POS;

                /* Settings 2 */
                /* Get word to calculate envelope_up_thres */
                lsb = (uint16_t) sc_data[idx++];
                msb = ((uint16_t) sc_data[idx++] << 8);
                lsb_msb = (uint16_t)(lsb | msb);

                step_cntr->envelope_up_thres = lsb_msb & BMA530_SC_ENVELOPE_UP_THRES_MSK;

                /* Settings 3 */
                /* Get word to calculate envelope_up_decay_coeff */
                lsb = (uint16_t) sc_data[idx++];
                msb = ((uint16_t) sc_data[idx++] << 8);
                lsb_msb = (uint16_t)(lsb | msb);

                step_cntr->envelope_up_decay_coeff = lsb_msb & BMA530_SC_ENVELOPE_UP_DECAY_COEFF_MSK;

                /* Settings 4 */
                /* Get word to calculate envelope_down_thres */
                lsb = (uint16_t) sc_data[idx++];
                msb = ((uint16_t) sc_data[idx++] << 8);
                lsb_msb = (uint16_t)(lsb | msb);

                step_cntr->envelope_down_thres = lsb_msb & BMA530_SC_ENVELOPE_DOWN_THRES_MSK;

                /* Settings 5 */
                /* Get word to calculate envelope_down_decay_coeff */
                lsb = (uint16_t) sc_data[idx++];
                msb = ((uint16_t) sc_data[idx++] << 8);
                lsb_msb = (uint16_t)(lsb | msb);

                step_cntr->envelope_down_decay_coeff = lsb_msb & BMA530_SC_ENVELOPE_DOWN_DECAY_COEFF_MSK;

                /* Settings 6 */
                /* Get word to calculate acc_mean_decay_coeff */
                lsb = (uint16_t) sc_data[idx++];
                msb = ((uint16_t) sc_data[idx++] << 8);
                lsb_msb = (uint16_t)(lsb | msb);

                step_cntr->acc_mean_decay_coeff = lsb_msb & BMA530_SC_ACC_MEAN_DECAY_COEFF_MSK;

                /* Settings 7 */
                /* Get word to calculate step_dur_mean_decay_coeff */
                lsb = (uint16_t) sc_data[idx++];
                msb = ((uint16_t) sc_data[idx++] << 8);
                lsb_msb = (uint16_t)(lsb | msb);

                step_cntr->step_dur_mean_decay_coeff = lsb_msb & BMA530_SC_STEP_DUR_MEAN_DECAY_COEFF_MSK;

                /* Settings 8 */

                /* Get word to calculate step_buffer_size, filter_cascade_enabled, step_counter_increment and
                 * en_half_step from same word */
                lsb = (uint16_t) sc_data[idx++];
                msb = ((uint16_t) sc_data[idx++] << 8);
                lsb_msb = (uint16_t)(lsb | msb);

                step_cntr->step_buffer_size = lsb_msb & BMA530_SC_STEP_BUFFER_SIZE_MSK;

                step_cntr->filter_cascade_enabled = (lsb_msb & BMA530_SC_FILTER_CASCADE_ENABLED_MSK) >>
                                                    BMA530_SC_FILTER_CASCADE_ENABLED_POS;

                step_cntr->step_counter_increment = (lsb_msb & BMA530_SC_STEP_COUNTER_INCREAMENT_MSK) >>
                                                    BMA530_SC_STEP_COUNTER_INCREAMENT_POS;

                step_cntr->en_half_step = (lsb_msb & BMA530_SC_EN_HALF_STEP_MSK) >> BMA530_SC_EN_HALF_STEP_POS;

                /* Settings 9 */
                /* Get word to calculate peak_duration_min_walking and peak_duration_min_running from same word */
                lsb = (uint16_t) sc_data[idx++];
                msb = ((uint16_t) sc_data[idx++] << 8);
                lsb_msb = (uint16_t)(lsb | msb);

                step_cntr->peak_duration_min_walking = lsb_msb & BMA530_SC_PEAK_DUR_MIN_WALKING_MSK;

                step_cntr->peak_duration_min_running = (lsb_msb & BMA530_SC_PEAK_DUR_MIN_RUNNING_MSK) >>
                                                       BMA530_SC_PEAK_DUR_MIN_RUNNING_POS;

                /* Settings 10 */
                /* Get word to calculate activity_detection_factor and activity_detection_thres from same word */
                lsb = (uint16_t) sc_data[idx++];
                msb = ((uint16_t) sc_data[idx++] << 8);
                lsb_msb = (uint16_t)(lsb | msb);

                step_cntr->activity_detection_factor = lsb_msb & BMA530_SC_ACTIVITY_DET_FACT_MSK;

                step_cntr->activity_detection_thres = (lsb_msb & BMA530_SC_ACTIVITY_DET_THRES_MSK) >>
                                                      BMA530_SC_ACTIVITY_DET_THRES_POS;

                /* Settings 11 */
                /* Get word to calculate step_duration_max and step_duration_window from same word */
                lsb = (uint16_t) sc_data[idx++];
                msb = ((uint16_t) sc_data[idx++] << 8);
                lsb_msb = (uint16_t)(lsb | msb);

                step_cntr->step_duration_max = lsb_msb & BMA530_SC_STEP_DUR_MAX_MSK;

                step_cntr->step_duration_window = (lsb_msb & BMA530_SC_STEP_DUR_WINDOW_MSK) >>
                                                  BMA530_SC_STEP_DUR_WINDOW_POS;

                /* Settings 12 */
                /* Get word to calculate en_step_dur_pp, step_dur_thres, en_mcr_pp and mcr_thres from same word */
                lsb = (uint16_t) sc_data[idx++];
                msb = ((uint16_t) sc_data[idx++] << 8);
                lsb_msb = (uint16_t)(lsb | msb);

                step_cntr->en_step_dur_pp = lsb_msb & BMA530_SC_EN_STEP_DUR_PP_MSK;

                step_cntr->step_dur_thres = (lsb_msb & BMA530_SC_STEP_DUR_THRES_MSK) >> BMA530_SC_STEP_DUR_THRES_POS;

                step_cntr->en_mcr_pp = (lsb_msb & BMA530_SC_EN_MCR_PP_MSK) >> BMA530_SC_EN_MCR_PP_POS;

                step_cntr->mcr_thres = (lsb_msb & BMA530_SC_MCR_THRES_MSK) >> BMA530_SC_MCR_THRES_POS;

                /* Settings 13 */
                /* Get word to calculate filter_coeff_b_2 */
                lsb = (uint16_t) sc_data[idx++];
                msb = ((uint16_t) sc_data[idx++] << 8);
                lsb_msb = (uint16_t)(lsb | msb);

                step_cntr->filter_coeff_b_2 = lsb_msb & BMA530_SC_FILTER_COEFF_B_2_MSK;

                /* Settings 14 */
                /* Get word to calculate filter_coeff_b_1 */
                lsb = (uint16_t) sc_data[idx++];
                msb = ((uint16_t) sc_data[idx++] << 8);
                lsb_msb = (uint16_t)(lsb | msb);

                step_cntr->filter_coeff_b_1 = lsb_msb & BMA530_SC_FILTER_COEFF_B_1_MSK;

                /* Settings 15 */
                /* Get word to calculate filter_coeff_b_0 */
                lsb = (uint16_t) sc_data[idx++];
                msb = ((uint16_t) sc_data[idx++] << 8);
                lsb_msb = (uint16_t)(lsb | msb);

                step_cntr->filter_coeff_b_0 = lsb_msb & BMA530_SC_FILTER_COEFF_B_0_MSK;

                /* Settings 16 */
                /* Get word to calculate filter_coeff_a_2 */
                lsb = (uint16_t) sc_data[idx++];
                msb = ((uint16_t) sc_data[idx++] << 8);
                lsb_msb = (uint16_t)(lsb | msb);

                step_cntr->filter_coeff_a_2 = lsb_msb & BMA530_SC_FILTER_COEFF_A_2_MSK;

                /* Settings 17 */
                /* Get word to calculate filter_coeff_a_1 */
                lsb = (uint16_t) sc_data[idx++];
                msb = ((uint16_t) sc_data[idx++] << 8);
                lsb_msb = (uint16_t)(lsb | msb);

                step_cntr->filter_coeff_a_1 = lsb_msb & BMA530_SC_FILTER_COEFF_A_1_MSK;

                /* Settings 18 */
                /* Get word to calculate filter_coeff_scale_a and filter_coeff_scale_b from same word */
                lsb = (uint16_t) sc_data[idx++];
                msb = ((uint16_t) sc_data[idx++] << 8);
                lsb_msb = (uint16_t)(lsb | msb);

                step_cntr->filter_coeff_scale_a = lsb_msb & BMA530_SC_FILTER_COEFF_SCALE_A_MSK;

                step_cntr->filter_coeff_scale_b = (lsb_msb & BMA530_SC_FILTER_COEFF_SCALE_B_MSK) >>
                                                  BMA530_SC_FILTER_COEFF_SCALE_B_POS;
            }
        }
    }

    return result;
}

/*!
 * @brief This API gets default step counter configuration
 */
int8_t bma530_get_default_step_counter_config(struct bma530_step_cntr *step_cntr, struct bma5_dev *dev)
{
    /* Variable to define error */
    int8_t result = BMA5_OK;

    /* Variable to store base address of step counter */
    uint8_t data = BMA530_BASE_ADDR_STEP_COUNTER;

    if (step_cntr == NULL)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        /* Set the step counter base address to feature engine transmission address to start DMA transaction */
        result = bma5_set_regs(BMA5_REG_FEATURE_DATA_ADDR, &data, 1, dev);

        if (result == BMA5_OK)
        {
            /* Settings 1 */
            step_cntr->watermark_level = BMA530_SC_DEFAULT_WATERMARK_LEVEL;

            step_cntr->reset_counter = BMA530_SC_DEFAULT_RESET_COUNTER;

            step_cntr->sd_en = BMA530_SC_DEFAULT_SD_EN;

            step_cntr->sc_en = BMA530_SC_DEFAULT_SC_EN;

            if (dev->context == BMA5_HEARABLE)
            {
                /* Settings 2 */

                step_cntr->envelope_up_thres = BMA530_STEP_COUNTER_2_ENVELOPE_UP_THRES_H;

                /* Settings 3 */

                step_cntr->envelope_up_decay_coeff = BMA530_STEP_COUNTER_3_ENVELOPE_UP_DECAY_COEFF_H;

                /* Settings 4 */

                step_cntr->envelope_down_thres = BMA530_STEP_COUNTER_4_ENVELOPE_DOWN_THRES_H;

                /* Settings 5 */

                step_cntr->envelope_down_decay_coeff = BMA530_STEP_COUNTER_5_ENVELOPE_DOWN_DECAY_COEFF_H;

                /* Settings 6 */

                step_cntr->acc_mean_decay_coeff = BMA530_STEP_COUNTER_6_ACC_MEAN_DECAY_COEFF_H;

                /* Settings 7 */

                step_cntr->step_dur_mean_decay_coeff = BMA530_STEP_COUNTER_7_STEP_DUR_MEAN_DECAY_COEFF_H;

                /* Settings 8 */

                step_cntr->step_buffer_size = BMA530_STEP_COUNTER_8_STEP_BUFFER_SIZE_H;

                step_cntr->filter_cascade_enabled = BMA530_STEP_COUNTER_8_FILTER_CASCADE_ENABLED_H;

                step_cntr->step_counter_increment = BMA530_STEP_COUNTER_8_STEP_COUNTER_INCREMENT_H;

                step_cntr->en_half_step = BMA530_STEP_COUNTER_8_EN_HALF_STEP_H;

                /* Settings 9 */

                step_cntr->peak_duration_min_walking = BMA530_STEP_COUNTER_9_PEAK_DURATION_MIN_WALKING_H;

                step_cntr->peak_duration_min_running = BMA530_STEP_COUNTER_9_PEAK_DURATION_MIN_RUNNING_H;

                /* Settings 10 */

                step_cntr->activity_detection_factor = BMA530_STEP_COUNTER_10_ACTIVITY_DETECTION_FACTOR_H;

                step_cntr->activity_detection_thres = BMA530_STEP_COUNTER_10_ACTIVITY_DETECTION_THRES_H;

                /* Settings 11 */

                step_cntr->step_duration_max = BMA530_STEP_COUNTER_11_STEP_DURATION_MAX_H;

                step_cntr->step_duration_window = BMA530_STEP_COUNTER_11_STEP_DURATION_WINDOW_H;

                /* Settings 12 */

                step_cntr->en_step_dur_pp = BMA530_STEP_COUNTER_12_EN_STEP_DUR_PP_H;

                step_cntr->step_dur_thres = BMA530_STEP_COUNTER_12_STEP_DUR_THRES_H;

                step_cntr->en_mcr_pp = BMA530_STEP_COUNTER_12_EN_MCR_PP_H;

                step_cntr->mcr_thres = BMA530_STEP_COUNTER_12_MCR_THRES_H;
            }
            else if (dev->context == BMA5_WEARABLE)
            {
                /* Settings 2 */

                step_cntr->envelope_up_thres = BMA530_STEP_COUNTER_2_ENVELOPE_UP_THRES_W;

                /* Settings 3 */

                step_cntr->envelope_up_decay_coeff = BMA530_STEP_COUNTER_3_ENVELOPE_UP_DECAY_COEFF_W;

                /* Settings 4 */

                step_cntr->envelope_down_thres = BMA530_STEP_COUNTER_4_ENVELOPE_DOWN_THRES_W;

                /* Settings 5 */

                step_cntr->envelope_down_decay_coeff = BMA530_STEP_COUNTER_5_ENVELOPE_DOWN_DECAY_COEFF_W;

                /* Settings 6 */

                step_cntr->acc_mean_decay_coeff = BMA530_STEP_COUNTER_6_ACC_MEAN_DECAY_COEFF_W;

                /* Settings 7 */

                step_cntr->step_dur_mean_decay_coeff = BMA530_STEP_COUNTER_7_STEP_DUR_MEAN_DECAY_COEFF_W;

                /* Settings 8 */

                step_cntr->step_buffer_size = BMA530_STEP_COUNTER_8_STEP_BUFFER_SIZE_W;

                step_cntr->filter_cascade_enabled = BMA530_STEP_COUNTER_8_FILTER_CASCADE_ENABLED_W;

                step_cntr->step_counter_increment = BMA530_STEP_COUNTER_8_STEP_COUNTER_INCREMENT_W;

                step_cntr->en_half_step = BMA530_STEP_COUNTER_8_EN_HALF_STEP_W;

                /* Settings 9 */

                step_cntr->peak_duration_min_walking = BMA530_STEP_COUNTER_9_PEAK_DURATION_MIN_WALKING_W;

                step_cntr->peak_duration_min_running = BMA530_STEP_COUNTER_9_PEAK_DURATION_MIN_RUNNING_W;

                /* Settings 10 */

                step_cntr->activity_detection_factor = BMA530_STEP_COUNTER_10_ACTIVITY_DETECTION_FACTOR_W;

                step_cntr->activity_detection_thres = BMA530_STEP_COUNTER_10_ACTIVITY_DETECTION_THRES_W;

                /* Settings 11 */

                step_cntr->step_duration_max = BMA530_STEP_COUNTER_11_STEP_DURATION_MAX_W;

                step_cntr->step_duration_window = BMA530_STEP_COUNTER_11_STEP_DURATION_WINDOW_W;

                /* Settings 12 */

                step_cntr->en_step_dur_pp = BMA530_STEP_COUNTER_12_EN_STEP_DUR_PP_W;

                step_cntr->step_dur_thres = BMA530_STEP_COUNTER_12_STEP_DUR_THRES_W;

                step_cntr->en_mcr_pp = BMA530_STEP_COUNTER_12_EN_MCR_PP_W;

                step_cntr->mcr_thres = BMA530_STEP_COUNTER_12_MCR_THRES_W;
            }
            else if (dev->context == BMA5_SMARTPHONE)
            {
                /* Settings 2 */

                step_cntr->envelope_up_thres = BMA530_STEP_COUNTER_2_ENVELOPE_UP_THRES_S;

                /* Settings 3 */

                step_cntr->envelope_up_decay_coeff = BMA530_STEP_COUNTER_3_ENVELOPE_UP_DECAY_COEFF_S;

                /* Settings 4 */

                step_cntr->envelope_down_thres = BMA530_STEP_COUNTER_4_ENVELOPE_DOWN_THRES_S;

                /* Settings 5 */

                step_cntr->envelope_down_decay_coeff = BMA530_STEP_COUNTER_5_ENVELOPE_DOWN_DECAY_COEFF_S;

                /* Settings 6 */

                step_cntr->acc_mean_decay_coeff = BMA530_STEP_COUNTER_6_ACC_MEAN_DECAY_COEFF_S;

                /* Settings 7 */

                step_cntr->step_dur_mean_decay_coeff = BMA530_STEP_COUNTER_7_STEP_DUR_MEAN_DECAY_COEFF_S;

                /* Settings 8 */

                step_cntr->step_buffer_size = BMA530_STEP_COUNTER_8_STEP_BUFFER_SIZE_S;

                step_cntr->filter_cascade_enabled = BMA530_STEP_COUNTER_8_FILTER_CASCADE_ENABLED_S;

                step_cntr->step_counter_increment = BMA530_STEP_COUNTER_8_STEP_COUNTER_INCREMENT_S;

                step_cntr->en_half_step = BMA530_STEP_COUNTER_8_EN_HALF_STEP_S;

                /* Settings 9 */

                step_cntr->peak_duration_min_walking = BMA530_STEP_COUNTER_9_PEAK_DURATION_MIN_WALKING_S;

                step_cntr->peak_duration_min_running = BMA530_STEP_COUNTER_9_PEAK_DURATION_MIN_RUNNING_S;

                /* Settings 10 */

                step_cntr->activity_detection_factor = BMA530_STEP_COUNTER_10_ACTIVITY_DETECTION_FACTOR_S;

                step_cntr->activity_detection_thres = BMA530_STEP_COUNTER_10_ACTIVITY_DETECTION_THRES_S;

                /* Settings 11 */

                step_cntr->step_duration_max = BMA530_STEP_COUNTER_11_STEP_DURATION_MAX_S;

                step_cntr->step_duration_window = BMA530_STEP_COUNTER_11_STEP_DURATION_WINDOW_S;

                /* Settings 12 */

                step_cntr->en_step_dur_pp = BMA530_STEP_COUNTER_12_EN_STEP_DUR_PP_S;

                step_cntr->step_dur_thres = BMA530_STEP_COUNTER_12_STEP_DUR_THRES_S;

                step_cntr->en_mcr_pp = BMA530_STEP_COUNTER_12_EN_MCR_PP_S;

                step_cntr->mcr_thres = BMA530_STEP_COUNTER_12_MCR_THRES_S;
            }
            else
            {
                result = BMA5_E_INVALID_CONTEXT_PARAM;
            }

            /* Settings 13 */

            step_cntr->filter_coeff_b_2 = BMA530_SC_DEFAULT_FILTER_COEFF_B_2;

            /* Settings 14 */

            step_cntr->filter_coeff_b_1 = BMA530_SC_DEFAULT_FILTER_COEFF_B_1;

            /* Settings 15 */

            step_cntr->filter_coeff_b_0 = BMA530_SC_DEFAULT_FILTER_COEFF_B_0;

            /* Settings 16 */

            step_cntr->filter_coeff_a_2 = BMA530_SC_DEFAULT_FILTER_COEFF_A_2;

            /* Settings 17 */

            step_cntr->filter_coeff_a_1 = BMA530_SC_DEFAULT_FILTER_COEFF_A_1;

            /* Settings 18 */

            step_cntr->filter_coeff_scale_a = BMA530_SC_DEFAULT_FILTER_COEFF_SCALE_A;

            step_cntr->filter_coeff_scale_b = BMA530_SC_DEFAULT_FILTER_COEFF_SCALE_B;
        }
    }

    return result;
}

/*!
 * @brief This API sets sig-motion configuration
 */
int8_t bma530_set_sig_motion_config(const struct bma530_sig_motion *sig_mot, struct bma5_dev *dev)
{
    /* Variable to define error */
    int8_t result = BMA5_OK;

    /* Variable to store base address of sig-motion */
    uint8_t data = BMA530_BASE_ADDR_SIG_MOTION;

    /* Array to store sig-motion data */
    uint8_t sig_data[8] = { 0 };

    /* Variables to store the data to be written in register */
    uint16_t block_size_1, block_size_2, p2p_min_1, p2p_min_2, mcr_min, p2p_max_1, p2p_max_2, mcr_max;

    if (sig_mot == NULL)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        /* Set the sig-motion base address to feature engine transmission address to start DMA transaction */
        result = bma5_set_regs(BMA5_REG_FEATURE_DATA_ADDR, &data, 1, dev);

        if (result == BMA5_OK)
        {
            /* Get the configuration from the feature engine register */
            result = bma5_get_regs(BMA5_REG_FEATURE_DATA_TX, sig_data, 8, dev);

            if (result == BMA5_OK)
            {
                /* Settings 1 */
                block_size_1 =
                    (BMA5_SET_BITS_POS_0(sig_data[2], BMA530_SIG_MOT_BLOCK_SIZE,
                                         sig_mot->block_size) & BMA530_SIG_MOT_BLOCK_SIZE_MSK);

                block_size_2 = (uint16_t)(sig_data[3] << 8);

                block_size_2 =
                    (BMA5_SET_BITS_POS_0(block_size_2, BMA530_SIG_MOT_BLOCK_SIZE,
                                         sig_mot->block_size) & BMA530_SIG_MOT_BLOCK_SIZE_MSK);

                /* Settings 2 */
                p2p_min_1 =
                    (BMA5_SET_BITS_POS_0(sig_data[4], BMA530_SIG_MOT_P2P_MIN,
                                         sig_mot->p2p_min) & BMA530_SIG_MOT_P2P_MIN_MSK);

                p2p_min_2 = (uint16_t)(sig_data[5] << 8);

                p2p_min_2 =
                    (BMA5_SET_BITS_POS_0(p2p_min_2, BMA530_SIG_MOT_P2P_MIN,
                                         sig_mot->p2p_min) & BMA530_SIG_MOT_P2P_MIN_MSK);

                mcr_min =
                    (BMA5_SET_BITS(sig_data[5], BMA530_SIG_MOT_MCR_MIN, sig_mot->mcr_min) & BMA530_SIG_MOT_MCR_MIN_MSK);

                /* Settings 3 */
                p2p_max_1 =
                    (BMA5_SET_BITS_POS_0(sig_data[6], BMA530_SIG_MOT_P2P_MAX,
                                         sig_mot->p2p_max) & BMA530_SIG_MOT_P2P_MAX_MSK);

                p2p_max_2 = (uint16_t)(sig_data[7] << 8);

                p2p_max_2 =
                    (BMA5_SET_BITS_POS_0(p2p_max_2, BMA530_SIG_MOT_P2P_MAX,
                                         sig_mot->p2p_max) & BMA530_SIG_MOT_P2P_MAX_MSK);

                mcr_max =
                    (BMA5_SET_BITS(sig_data[7], BMA530_SIG_MOT_MCR_MAX, sig_mot->mcr_max) & BMA530_SIG_MOT_MCR_MAX_MSK);

                sig_data[0] = (uint8_t)block_size_1;
                sig_data[1] = (uint8_t)(block_size_2 >> 8);
                sig_data[2] = (uint8_t)p2p_min_1;
                sig_data[3] = (uint8_t)((p2p_min_2 | mcr_min) >> 8);
                sig_data[4] = (uint8_t)p2p_max_1;
                sig_data[5] = (uint8_t)((p2p_max_2 | mcr_max) >> 8);

                /* Set the configuration from the feature engine register */
                result = bma5_set_regs(BMA5_REG_FEATURE_DATA_TX, sig_data, 6, dev);
            }
        }
    }

    return result;
}

/*!
 * @brief This API gets sig-motion configuration
 */
int8_t bma530_get_sig_motion_config(struct bma530_sig_motion *sig_mot, struct bma5_dev *dev)
{
    /* Variable to define error */
    int8_t result = BMA5_OK;

    /* Variable to store base address of sig-motion */
    uint8_t data = BMA530_BASE_ADDR_SIG_MOTION;

    /* Array to store sig-motion data */
    uint8_t sig_data[8] = { 0 };

    /* Variable to define array offset */
    uint8_t idx;

    /* Variable to define LSB */
    uint16_t lsb;

    /* Variable to define MSB */
    uint16_t msb;

    /* Variable to define a word */
    uint16_t lsb_msb;

    if (sig_mot == NULL)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        /* Set the sig-motion base address to feature engine transmission address to start DMA transaction */
        result = bma5_set_regs(BMA5_REG_FEATURE_DATA_ADDR, &data, 1, dev);

        if (result == BMA5_OK)
        {
            /* Get the configuration from the feature engine register */
            result = bma5_get_regs(BMA5_REG_FEATURE_DATA_TX, sig_data, 8, dev);

            if (result == BMA5_OK)
            {
                /* First two bytes are dummy bytes */
                idx = 2;

                /* Settings 1 */
                /* Get word to calculate block_size */
                lsb = (uint16_t) sig_data[idx++];
                msb = ((uint16_t) sig_data[idx++] << 8);
                lsb_msb = (uint16_t)(lsb | msb);

                sig_mot->block_size = lsb_msb & BMA530_SIG_MOT_BLOCK_SIZE_MSK;

                /* Settings 2 */
                /* Get word to calculate p2p_min and mcr_min from same word */
                lsb = (uint16_t) sig_data[idx++];
                msb = ((uint16_t) sig_data[idx++] << 8);
                lsb_msb = (uint16_t)(lsb | msb);

                sig_mot->p2p_min = lsb_msb & BMA530_SIG_MOT_P2P_MIN_MSK;

                sig_mot->mcr_min = (lsb_msb & BMA530_SIG_MOT_MCR_MIN_MSK) >> BMA530_SIG_MOT_MCR_MIN_POS;

                /* Settings 3 */
                /* Get word to calculate p2p_max and mcr_max from same word */
                lsb = (uint16_t) sig_data[idx++];
                msb = ((uint16_t) sig_data[idx++] << 8);
                lsb_msb = (uint16_t)(lsb | msb);

                sig_mot->p2p_max = lsb_msb & BMA530_SIG_MOT_P2P_MAX_MSK;

                sig_mot->mcr_max = (lsb_msb & BMA530_SIG_MOT_MCR_MAX_MSK) >> BMA530_SIG_MOT_MCR_MAX_POS;
            }
        }
    }

    return result;
}

/*!
 * @brief This API gets default sig-motion configuration
 */
int8_t bma530_get_default_sig_motion_config(struct bma530_sig_motion *sig_mot, struct bma5_dev *dev)
{
    /* Variable to define error */
    int8_t result = BMA5_OK;

    /* Variable to store base address of sig-motion */
    uint8_t data = BMA530_BASE_ADDR_SIG_MOTION;

    if (sig_mot == NULL)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        /* Set the sig-motion base address to feature engine transmission address to start DMA transaction */
        result = bma5_set_regs(BMA5_REG_FEATURE_DATA_ADDR, &data, 1, dev);

        if (result == BMA5_OK)
        {
            if (dev->context == BMA5_HEARABLE)
            {
                /* Settings 1 */

                sig_mot->block_size = BMA530_SIG_MOTION_1_BLOCK_SIZE_H;

                /* Settings 2 */

                sig_mot->p2p_min = BMA530_SIG_MOTION_2_P2P_MIN_H;

                sig_mot->mcr_min = BMA530_SIG_MOTION_2_MCR_MIN_H;

                /* Settings 3 */

                sig_mot->p2p_max = BMA530_SIG_MOTION_3_P2P_MAX_H;

                sig_mot->mcr_max = BMA530_SIG_MOTION_3_MCR_MAX_H;
            }
            else if (dev->context == BMA5_WEARABLE)
            {
                /* Settings 1 */

                sig_mot->block_size = BMA530_SIG_MOTION_1_BLOCK_SIZE_W;

                /* Settings 2 */

                sig_mot->p2p_min = BMA530_SIG_MOTION_2_P2P_MIN_W;

                sig_mot->mcr_min = BMA530_SIG_MOTION_2_MCR_MIN_W;

                /* Settings 3 */

                sig_mot->p2p_max = BMA530_SIG_MOTION_3_P2P_MAX_W;

                sig_mot->mcr_max = BMA530_SIG_MOTION_3_MCR_MAX_W;
            }
            else if (dev->context == BMA5_SMARTPHONE)
            {
                /* Settings 1 */

                sig_mot->block_size = BMA530_SIG_MOTION_1_BLOCK_SIZE_S;

                /* Settings 2 */

                sig_mot->p2p_min = BMA530_SIG_MOTION_2_P2P_MIN_S;

                sig_mot->mcr_min = BMA530_SIG_MOTION_2_MCR_MIN_S;

                /* Settings 3 */

                sig_mot->p2p_max = BMA530_SIG_MOTION_3_P2P_MAX_S;

                sig_mot->mcr_max = BMA530_SIG_MOTION_3_MCR_MAX_S;
            }
            else
            {
                result = BMA5_E_INVALID_CONTEXT_PARAM;
            }
        }
    }

    return result;
}

/*!
 * @brief This API sets tilt configuration
 */
int8_t bma530_set_tilt_config(const struct bma530_tilt *tilt, struct bma5_dev *dev)
{
    /* Variable to define error */
    int8_t result = BMA5_OK;

    /* Variable to store base address of tilt */
    uint8_t data = BMA530_BASE_ADDR_TILT;

    /* Array to store tilt config data */
    uint8_t tilt_data[6] = { 0 };

    /* Variables to store the data to be written in register */
    uint16_t segment_size, min_tilt_angle, beta_acc_mean_1, beta_acc_mean_2;

    if (tilt == NULL)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        /* Set the tilt base address to feature engine transmission address to start DMA transaction */
        result = bma5_set_regs(BMA5_REG_FEATURE_DATA_ADDR, &data, 1, dev);

        if (result == BMA5_OK)
        {
            /* Get the configuration from the feature engine register */
            result = bma5_get_regs(BMA5_REG_FEATURE_DATA_TX, tilt_data, 6, dev);

            if (result == BMA5_OK)
            {
                /* Settings 1 */
                segment_size =
                    (BMA5_SET_BITS_POS_0(tilt_data[2], BMA530_TILT_SEGMENT_SIZE,
                                         tilt->segment_size) & BMA530_TILT_SEGMENT_SIZE_MSK);

                min_tilt_angle =
                    (BMA5_SET_BITS(tilt_data[3], BMA530_TILT_MIN_TILT_ANGLE,
                                   tilt->min_tilt_angle) & BMA530_TILT_MIN_TILT_ANGLE_MSK);

                /* Settings 2 */
                beta_acc_mean_1 =
                    (BMA5_SET_BITS_POS_0(tilt_data[4], BMA530_TILT_BETA_ACC_MEAN,
                                         tilt->beta_acc_mean) & BMA530_TILT_BETA_ACC_MEAN_MSK);

                beta_acc_mean_2 = (uint16_t)(tilt_data[5] << 8);

                beta_acc_mean_2 =
                    (BMA5_SET_BITS_POS_0(beta_acc_mean_2, BMA530_TILT_BETA_ACC_MEAN,
                                         tilt->beta_acc_mean) & BMA530_TILT_BETA_ACC_MEAN_MSK);

                tilt_data[0] = (uint8_t)segment_size;
                tilt_data[1] = (uint8_t)(min_tilt_angle >> 8);
                tilt_data[2] = (uint8_t)beta_acc_mean_1;
                tilt_data[3] = (uint8_t)(beta_acc_mean_2 >> 8);

                /* Set the configuration from the feature engine register */
                result = bma5_set_regs(BMA5_REG_FEATURE_DATA_TX, tilt_data, 4, dev);
            }
        }
    }

    return result;
}

/*!
 * @brief This API gets tilt configuration
 */
int8_t bma530_get_tilt_config(struct bma530_tilt *tilt, struct bma5_dev *dev)
{
    /* Variable to define error */
    int8_t result = BMA5_OK;

    /* Variable to store base address of tilt */
    uint8_t data = BMA530_BASE_ADDR_TILT;

    /* Array to store tilt config data */
    uint8_t tilt_data[6] = { 0 };

    /* Variable to define array offset */
    uint8_t idx;

    /* Variable to define LSB */
    uint16_t lsb;

    /* Variable to define MSB */
    uint16_t msb;

    /* Variable to define a word */
    uint16_t lsb_msb;

    if (tilt == NULL)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        /* Set the tilt base address to feature engine transmission address to start DMA transaction */
        result = bma5_set_regs(BMA5_REG_FEATURE_DATA_ADDR, &data, 1, dev);

        if (result == BMA5_OK)
        {
            /* Get the configuration from the feature engine register */
            result = bma5_get_regs(BMA5_REG_FEATURE_DATA_TX, tilt_data, 6, dev);

            if (result == BMA5_OK)
            {
                /* First two bytes are dummy bytes */
                idx = 2;

                /* Settings 1 */
                /* Get word to calculate segment_size and min_tilt_angle from same word */
                lsb = (uint16_t) tilt_data[idx++];
                msb = ((uint16_t) tilt_data[idx++] << 8);
                lsb_msb = (uint16_t)(lsb | msb);

                tilt->segment_size = lsb_msb & BMA530_TILT_SEGMENT_SIZE_MSK;

                tilt->min_tilt_angle = (lsb_msb & BMA530_TILT_MIN_TILT_ANGLE_MSK) >> BMA530_TILT_MIN_TILT_ANGLE_POS;

                /* Settings 2 */
                /* Get word to calculate beta_acc_mean */
                lsb = (uint16_t) tilt_data[idx++];
                msb = ((uint16_t) tilt_data[idx++] << 8);
                lsb_msb = (uint16_t)(lsb | msb);

                tilt->beta_acc_mean = lsb_msb & BMA530_TILT_BETA_ACC_MEAN_MSK;
            }
        }
    }

    return result;
}

/*!
 * @brief This API sets orientation configuration
 */
int8_t bma530_set_orient_config(const struct bma530_orient *orient, struct bma5_dev *dev)
{
    /* Variable to define error */
    int8_t result = BMA5_OK;

    /* Variable to store base address of orientation */
    uint8_t data = BMA530_BASE_ADDR_ORIENTATION;

    /* Array to store orientation config data */
    uint8_t orient_data[6] = { 0 };

    /* Variables to store the data to be written in register */
    uint16_t ud_en, mode, blocking, slope_thres, hysteresis;
    uint16_t theta_1, theta_2, hold_time;

    if (orient == NULL)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        /* Set the orientation base address to feature engine transmission address to start DMA transaction */
        result = bma5_set_regs(BMA5_REG_FEATURE_DATA_ADDR, &data, 1, dev);

        if (result == BMA5_OK)
        {
            /* Get the configuration from the feature engine register */
            result = bma5_get_regs(BMA5_REG_FEATURE_DATA_TX, orient_data, 6, dev);

            if (result == BMA5_OK)
            {
                /* Settings 1 */
                ud_en =
                    (BMA5_SET_BITS_POS_0(orient_data[0], BMA530_ORIENT_UD_EN, orient->ud_en) & BMA530_ORIENT_UD_EN_MSK);

                mode = (BMA5_SET_BITS(orient_data[0], BMA530_ORIENT_MODE, orient->mode) & BMA530_ORIENT_MODE_MSK);

                blocking =
                    (BMA5_SET_BITS(orient_data[0], BMA530_ORIENT_BLOCKING,
                                   orient->blocking) & BMA530_ORIENT_BLOCKING_MSK);

                theta_1 = (BMA5_SET_BITS(orient_data[0], BMA530_ORIENT_THETA, orient->theta) & BMA530_ORIENT_THETA_MSK);

                theta_2 = (uint16_t)(orient_data[1] << 8);

                theta_2 = (BMA5_SET_BITS(theta_2, BMA530_ORIENT_THETA, orient->theta) & BMA530_ORIENT_THETA_MSK);

                hold_time =
                    (BMA5_SET_BITS(orient_data[1], BMA530_ORIENT_HOLD_TIME,
                                   orient->hold_time) & BMA530_ORIENT_HOLD_TIME_MSK);

                /* Settings 2 */
                slope_thres =
                    (BMA5_SET_BITS_POS_0(orient_data[2], BMA530_ORIENT_SLOPE_THRES,
                                         orient->slope_thres) & BMA530_ORIENT_SLOPE_THRES_MSK);

                hysteresis =
                    (BMA5_SET_BITS(orient_data[3], BMA530_ORIENT_HYSTERESIS,
                                   orient->hysteresis) & BMA530_ORIENT_HYSTERESIS_MSK);

                orient_data[0] = (uint8_t)(ud_en | mode | blocking | theta_1);
                orient_data[1] = (uint8_t)((theta_2 | hold_time) >> 8);
                orient_data[2] = (uint8_t)slope_thres;
                orient_data[3] = (uint8_t)(hysteresis >> 8);

                /* Set the configuration from the feature engine register */
                result = bma5_set_regs(BMA5_REG_FEATURE_DATA_TX, orient_data, 4, dev);
            }
        }
    }

    return result;
}

/*!
 * @brief This API gets orientation configuration
 */
int8_t bma530_get_orient_config(struct bma530_orient *orient, struct bma5_dev *dev)
{
    /* Variable to define error */
    int8_t result = BMA5_OK;

    /* Variable to store base address of orientation */
    uint8_t data = BMA530_BASE_ADDR_ORIENTATION;

    /* Array to store orientation config data */
    uint8_t orient_data[6] = { 0 };

    /* Variable to define array offset */
    uint8_t idx = 0;

    /* Variable to define LSB */
    uint16_t lsb;

    /* Variable to define MSB */
    uint16_t msb;

    /* Variable to define a word */
    uint16_t lsb_msb;

    if (orient == NULL)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        /* Set the orientation base address to feature engine transmission address to start DMA transaction */
        result = bma5_set_regs(BMA5_REG_FEATURE_DATA_ADDR, &data, 1, dev);

        if (result == BMA5_OK)
        {
            /* Get the configuration from the feature engine register */
            result = bma5_get_regs(BMA5_REG_FEATURE_DATA_TX, orient_data, 6, dev);

            if (result == BMA5_OK)
            {
                /* First two bytes are dummy bytes */
                idx = 2;

                /* Settings 1 */
                /* Get word to calculate ud en, mode, blocking, theta and hold time from same word */
                lsb = (uint16_t) orient_data[idx++];
                msb = ((uint16_t) orient_data[idx++] << 8);
                lsb_msb = (uint16_t)(lsb | msb);

                orient->ud_en = lsb_msb & BMA530_ORIENT_UD_EN_MSK;

                orient->mode = (lsb_msb & BMA530_ORIENT_MODE_MSK) >> BMA530_ORIENT_MODE_POS;

                orient->blocking = (lsb_msb & BMA530_ORIENT_BLOCKING_MSK) >> BMA530_ORIENT_BLOCKING_POS;

                orient->theta = (lsb_msb & BMA530_ORIENT_THETA_MSK) >> BMA530_ORIENT_THETA_POS;

                orient->hold_time = (lsb_msb & BMA530_ORIENT_HOLD_TIME_MSK) >> BMA530_ORIENT_HOLD_TIME_POS;

                /* Settings 2 */
                /* Get word to calculate slope threshold and hysteresis from same word */
                lsb = (uint16_t) orient_data[idx++];
                msb = ((uint16_t) orient_data[idx++] << 8);
                lsb_msb = (uint16_t)(lsb | msb);

                orient->slope_thres = lsb_msb & BMA530_ORIENT_SLOPE_THRES_MSK;

                orient->hysteresis = (lsb_msb & BMA530_ORIENT_HYSTERESIS_MSK) >> BMA530_ORIENT_HYSTERESIS_POS;
            }
        }
    }

    return result;
}

/*!
 * @brief This API sets generic interrupt configurations
 */
int8_t bma530_set_generic_int_config(const struct bma530_generic_interrupt_types *gen_int,
                                     uint8_t n_ints,
                                     struct bma5_dev *dev)
{
    /* Variable to define error */
    int8_t result = BMA5_OK;
    uint8_t loop;

    /* Variable to store base address of generic interrupt 1 */
    uint8_t data;

    /* Array to store generic interrupt 1 data */
    uint8_t int_1_data[16] = { 0 };

    uint16_t slope_thres_1, slope_thres_2, comb_sel, axis_sel, hysteresis_1, hysteresis_2, criterion_sel;
    uint16_t acc_ref_up, duration_1, duration_2, wait_time, quiet_time_1, quiet_time_2;
    uint16_t ref_acc_x_1, ref_acc_x_2, ref_acc_y_1, ref_acc_y_2, ref_acc_z_1, ref_acc_z_2;

    if (gen_int == NULL)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        for (loop = 0; loop < n_ints; loop++)
        {
            switch (gen_int[loop].generic_interrupt)
            {
                case BMA530_GEN_INT_1:
                    data = BMA530_BASE_ADDR_GENERIC_INT1;
                    break;

                case BMA530_GEN_INT_2:
                    data = BMA530_BASE_ADDR_GENERIC_INT2;
                    break;

                case BMA530_GEN_INT_3:
                    data = BMA530_BASE_ADDR_GENERIC_INT3;
                    break;

                default:
                    result = BMA5_E_INVALID_GEN_INT;

                    return result;
            }

            if (result == BMA5_OK)
            {
                /* Set the generic interrupt base address to feature engine transmission address to start DMA
                 * transaction */
                result = bma5_set_regs(BMA5_REG_FEATURE_DATA_ADDR, &data, 1, dev);
            }

            if (result == BMA5_OK)
            {
                /* Get the configuration from the feature engine register */
                result = bma5_get_regs(BMA5_REG_FEATURE_DATA_TX, int_1_data, 16, dev);

                if (result == BMA5_OK)
                {
                    /* Settings 1 */
                    slope_thres_1 =
                        (BMA5_SET_BITS_POS_0(int_1_data[2], BMA530_GEN_INT_SLOPE_THRES,
                                             gen_int[loop].gen_int.slope_thres) & BMA530_GEN_INT_SLOPE_THRES_MSK);

                    slope_thres_2 = (uint16_t)(int_1_data[3] << 8);

                    slope_thres_2 =
                        (BMA5_SET_BITS_POS_0(slope_thres_2, BMA530_GEN_INT_SLOPE_THRES,
                                             gen_int[loop].gen_int.slope_thres) & BMA530_GEN_INT_SLOPE_THRES_MSK);

                    comb_sel =
                        (BMA5_SET_BITS(int_1_data[3], BMA530_GEN_INT_COMB_SEL,
                                       gen_int[loop].gen_int.comb_sel) & BMA530_GEN_INT_COMB_SEL_MSK);

                    axis_sel =
                        (BMA5_SET_BITS(int_1_data[3], BMA530_GEN_INT_AXIS_SEL,
                                       gen_int[loop].gen_int.axis_sel) & BMA530_GEN_INT_AXIS_SEL_MSK);

                    /* Settings 2 */
                    hysteresis_1 =
                        (BMA5_SET_BITS_POS_0(int_1_data[4], BMA530_GEN_INT_HYST,
                                             gen_int[loop].gen_int.hysteresis) & BMA530_GEN_INT_HYST_MSK);

                    hysteresis_2 = (uint16_t)(int_1_data[5] << 8);

                    hysteresis_2 =
                        (BMA5_SET_BITS_POS_0(hysteresis_2, BMA530_GEN_INT_HYST,
                                             gen_int[loop].gen_int.hysteresis) & BMA530_GEN_INT_HYST_MSK);

                    criterion_sel =
                        (BMA5_SET_BITS(int_1_data[5], BMA530_GEN_INT_CRIT_SEL,
                                       gen_int[loop].gen_int.criterion_sel) & BMA530_GEN_INT_CRIT_SEL_MSK);

                    acc_ref_up =
                        (BMA5_SET_BITS(int_1_data[5], BMA530_GEN_INT_ACC_REF_UP,
                                       gen_int[loop].gen_int.acc_ref_up) & BMA530_GEN_INT_ACC_REF_UP_MSK);

                    /* Settings 3 */
                    duration_1 =
                        (BMA5_SET_BITS_POS_0(int_1_data[6], BMA530_GEN_INT_DURATION,
                                             gen_int[loop].gen_int.duration) & BMA530_GEN_INT_DURATION_MSK);

                    duration_2 = (uint16_t)(int_1_data[7] << 8);

                    duration_2 =
                        (BMA5_SET_BITS_POS_0(duration_2, BMA530_GEN_INT_DURATION,
                                             gen_int[loop].gen_int.duration) & BMA530_GEN_INT_DURATION_MSK);

                    wait_time =
                        (BMA5_SET_BITS(int_1_data[7], BMA530_GEN_INT_WAIT_TIME,
                                       gen_int[loop].gen_int.wait_time) & BMA530_GEN_INT_WAIT_TIME_MSK);

                    /* Settings 4 */
                    quiet_time_1 =
                        (BMA5_SET_BITS_POS_0(int_1_data[8], BMA530_GEN_INT_QUIET_TIME,
                                             gen_int[loop].gen_int.quiet_time) & BMA530_GEN_INT_QUIET_TIME_MSK);

                    quiet_time_2 = (uint16_t)(int_1_data[9] << 8);

                    quiet_time_2 =
                        (BMA5_SET_BITS_POS_0(quiet_time_2, BMA530_GEN_INT_QUIET_TIME,
                                             gen_int[loop].gen_int.quiet_time) & BMA530_GEN_INT_QUIET_TIME_MSK);

                    /* Settings 5 */
                    ref_acc_x_1 =
                        BMA5_SET_BITS_POS_0(int_1_data[10], BMA530_GEN_INT_REF_ACC_X,
                                            (uint16_t)(gen_int[loop].gen_int.ref_acc_x & BMA530_GEN_INT_REF_ACC_X_MSK));

                    ref_acc_x_2 = (uint16_t)(int_1_data[11] << 8);

                    ref_acc_x_2 =
                        BMA5_SET_BITS_POS_0(ref_acc_x_2, BMA530_GEN_INT_REF_ACC_X,
                                            (uint16_t)(gen_int[loop].gen_int.ref_acc_x & BMA530_GEN_INT_REF_ACC_X_MSK));

                    /* Settings 6 */
                    ref_acc_y_1 =
                        BMA5_SET_BITS_POS_0(int_1_data[12], BMA530_GEN_INT_REF_ACC_Y,
                                            (uint16_t)(gen_int[loop].gen_int.ref_acc_y & BMA530_GEN_INT_REF_ACC_Y_MSK));

                    ref_acc_y_2 = (uint16_t)(int_1_data[13] << 8);

                    ref_acc_y_2 =
                        BMA5_SET_BITS_POS_0(ref_acc_y_2, BMA530_GEN_INT_REF_ACC_Y,
                                            (uint16_t)(gen_int[loop].gen_int.ref_acc_y & BMA530_GEN_INT_REF_ACC_Y_MSK));

                    /* Settings 7 */
                    ref_acc_z_1 =
                        BMA5_SET_BITS_POS_0(int_1_data[14], BMA530_GEN_INT_REF_ACC_Z,
                                            (uint16_t)(gen_int[loop].gen_int.ref_acc_z & BMA530_GEN_INT_REF_ACC_Z_MSK));

                    ref_acc_z_2 = (uint16_t)(int_1_data[15] << 8);

                    ref_acc_z_2 =
                        BMA5_SET_BITS_POS_0(ref_acc_z_2, BMA530_GEN_INT_REF_ACC_Z,
                                            (uint16_t)(gen_int[loop].gen_int.ref_acc_z & BMA530_GEN_INT_REF_ACC_Z_MSK));

                    int_1_data[0] = (uint8_t)slope_thres_1;
                    int_1_data[1] = (uint8_t)((slope_thres_2 | comb_sel | axis_sel) >> 8);
                    int_1_data[2] = (uint8_t)hysteresis_1;
                    int_1_data[3] = (uint8_t)((hysteresis_2 | criterion_sel | acc_ref_up) >> 8);
                    int_1_data[4] = (uint8_t)duration_1;
                    int_1_data[5] = (uint8_t)((duration_2 | wait_time) >> 8);
                    int_1_data[6] = (uint8_t)quiet_time_1;
                    int_1_data[7] = (uint8_t)(quiet_time_2 >> 8);
                    int_1_data[8] = (uint8_t)ref_acc_x_1;
                    int_1_data[9] = (uint8_t)(ref_acc_x_2 >> 8);
                    int_1_data[10] = (uint8_t)ref_acc_y_1;
                    int_1_data[11] = (uint8_t)(ref_acc_y_2 >> 8);
                    int_1_data[12] = (uint8_t)ref_acc_z_1;
                    int_1_data[13] = (uint8_t)(ref_acc_z_2 >> 8);

                    /* Set the configuration from the feature engine register */
                    result = bma5_set_regs(BMA5_REG_FEATURE_DATA_TX, int_1_data, 14, dev);
                }
            }
        }
    }

    return result;
}

/*!
 * @brief This API gets default generic interrupt 1 configurations
 */
int8_t bma530_get_default_generic_int_config(struct bma530_generic_interrupt_types *gen_int,
                                             uint8_t n_ints,
                                             struct bma5_dev *dev)
{
    /* Variable to define error */
    int8_t result = BMA5_OK;
    uint8_t loop;

    /* Variable to store base address of generic interrupt 1 */
    uint8_t data;

    if (gen_int == NULL)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        for (loop = 0; loop < n_ints; loop++)
        {
            switch (gen_int[loop].generic_interrupt)
            {
                case BMA530_GEN_INT_1:
                    data = BMA530_BASE_ADDR_GENERIC_INT1;
                    break;

                case BMA530_GEN_INT_2:
                    data = BMA530_BASE_ADDR_GENERIC_INT2;
                    break;

                default:
                    result = BMA5_E_INVALID_GEN_INT;

                    return result;
            }

            if (result == BMA5_OK)
            {
                /* Set the generic interrupt base address to feature engine transmission address to start DMA
                 * transaction */
                result = bma5_set_regs(BMA5_REG_FEATURE_DATA_ADDR, &data, 1, dev);
            }

            if (result == BMA5_OK)
            {
                gen_int[loop].gen_int.comb_sel = BMA530_GEN_INT_COMB_SEL_LOGICAL_OR;

                gen_int[loop].gen_int.axis_sel = BMA530_AXIS_SEL_DEFAULT;

                gen_int[loop].gen_int.criterion_sel = BMA530_GEN_INT_CRI_SEL_ACT;

                gen_int[loop].gen_int.acc_ref_up = BMA530_GEN_INT_ACC_REF_UP_ON_ALWAYS;

                gen_int[loop].gen_int.duration = BMA530_DURATION_DEFAULT;

                gen_int[loop].gen_int.wait_time = BMA530_WAIT_TIME_DEFAULT;

                gen_int[loop].gen_int.quiet_time = BMA530_QUIET_TIME_DEFAULT;

                gen_int[loop].gen_int.ref_acc_x = BMA530_ACC_REF_X_DEFAULT;

                gen_int[loop].gen_int.ref_acc_y = BMA530_ACC_REF_Y_DEFAULT;

                gen_int[loop].gen_int.ref_acc_z = BMA530_ACC_REF_Z_DEFAULT;

                if (dev->context == BMA5_HEARABLE && gen_int[loop].generic_interrupt == BMA530_GEN_INT_1)
                {
                    gen_int[loop].gen_int.slope_thres = BMA530_GENERIC_INTERRUPT1_1_GI1_SLOPE_THRES_H;
                    gen_int[loop].gen_int.hysteresis = BMA530_GENERIC_INTERRUPT1_2_GI1_HYSTERESIS_H;
                }
                else if (dev->context == BMA5_WEARABLE && gen_int[loop].generic_interrupt == BMA530_GEN_INT_1)
                {
                    gen_int[loop].gen_int.slope_thres = BMA530_GENERIC_INTERRUPT1_1_GI1_SLOPE_THRES_W;
                    gen_int[loop].gen_int.hysteresis = BMA530_GENERIC_INTERRUPT1_2_GI1_HYSTERESIS_W;
                }
                else if (dev->context == BMA5_SMARTPHONE && gen_int[loop].generic_interrupt == BMA530_GEN_INT_1)
                {
                    gen_int[loop].gen_int.slope_thres = BMA530_GENERIC_INTERRUPT1_1_GI1_SLOPE_THRES_S;
                    gen_int[loop].gen_int.hysteresis = BMA530_GENERIC_INTERRUPT1_2_GI1_HYSTERESIS_S;
                }
                else if (dev->context == BMA5_HEARABLE && gen_int[loop].generic_interrupt == BMA530_GEN_INT_2)
                {
                    gen_int[loop].gen_int.slope_thres = BMA530_GENERIC_INTERRUPT2_1_GI2_SLOPE_THRES_H;
                    gen_int[loop].gen_int.hysteresis = BMA530_GENERIC_INTERRUPT2_2_GI2_HYSTERESIS_H;
                }
                else if (dev->context == BMA5_WEARABLE && gen_int[loop].generic_interrupt == BMA530_GEN_INT_2)
                {
                    gen_int[loop].gen_int.slope_thres = BMA530_GENERIC_INTERRUPT2_1_GI2_SLOPE_THRES_W;
                    gen_int[loop].gen_int.hysteresis = BMA530_GENERIC_INTERRUPT2_2_GI2_HYSTERESIS_W;
                }
                else if (dev->context == BMA5_SMARTPHONE && gen_int[loop].generic_interrupt == BMA530_GEN_INT_2)
                {
                    gen_int[loop].gen_int.slope_thres = BMA530_GENERIC_INTERRUPT2_1_GI2_SLOPE_THRES_S;
                    gen_int[loop].gen_int.hysteresis = BMA530_GENERIC_INTERRUPT2_2_GI2_HYSTERESIS_S;
                }
                else
                {
                    result = BMA5_E_INVALID_CONTEXT_PARAM;
                }
            }
        }
    }

    return result;
}

/*!
 * @brief This API gets generic interrupt 1 configurations
 */
int8_t bma530_get_generic_int_config(struct bma530_generic_interrupt_types *gen_int,
                                     uint8_t n_ints,
                                     struct bma5_dev *dev)
{
    /* Variable to define error */
    int8_t result = BMA5_OK;
    uint8_t loop;

    /* Variable to store base address of generic interrupt 1 */
    uint8_t data;

    /* Array to store generic interrupt 1 data */
    uint8_t int_1_data[16] = { 0 };

    /* Variable to define array offset */
    uint8_t idx = 0;

    /* Variable to define LSB */
    uint16_t lsb;

    /* Variable to define MSB */
    uint16_t msb;

    /* Variable to define a word */
    uint16_t lsb_msb;

    if (gen_int == NULL)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        for (loop = 0; loop < n_ints; loop++)
        {
            switch (gen_int[loop].generic_interrupt)
            {
                case BMA530_GEN_INT_1:
                    data = BMA530_BASE_ADDR_GENERIC_INT1;
                    break;

                case BMA530_GEN_INT_2:
                    data = BMA530_BASE_ADDR_GENERIC_INT2;
                    break;

                case BMA530_GEN_INT_3:
                    data = BMA530_BASE_ADDR_GENERIC_INT3;
                    break;

                default:
                    result = BMA5_E_INVALID_GEN_INT;

                    return result;
            }

            if (result == BMA5_OK)
            {
                /* Set the generic interrupt base address to feature engine transmission address to start DMA
                 * transaction */
                result = bma5_set_regs(BMA5_REG_FEATURE_DATA_ADDR, &data, 1, dev);
            }

            if (result == BMA5_OK)
            {
                /* Get the configuration from the feature engine register */
                result = bma5_get_regs(BMA5_REG_FEATURE_DATA_TX, int_1_data, 16, dev);

                if (result == BMA5_OK)
                {
                    /* First two bytes are dummy bytes */
                    idx = 2;

                    /* Settings 1 */
                    /* Get word to calculate slope threshold, comb_sel and axis select from same word */
                    lsb = (uint16_t) int_1_data[idx++];
                    msb = ((uint16_t) int_1_data[idx++] << 8);
                    lsb_msb = (uint16_t)(lsb | msb);

                    gen_int[loop].gen_int.slope_thres = lsb_msb & BMA530_GEN_INT_SLOPE_THRES_MSK;

                    gen_int[loop].gen_int.comb_sel = (lsb_msb & BMA530_GEN_INT_COMB_SEL_MSK) >>
                                                     BMA530_GEN_INT_COMB_SEL_POS;

                    gen_int[loop].gen_int.axis_sel = (lsb_msb & BMA530_GEN_INT_AXIS_SEL_MSK) >>
                                                     BMA530_GEN_INT_AXIS_SEL_POS;

                    /* Settings 2 */
                    /* Get word to calculate hysteresis, criterion_sel and acc_ref_up from same word */
                    lsb = (uint16_t) int_1_data[idx++];
                    msb = ((uint16_t) int_1_data[idx++] << 8);
                    lsb_msb = (uint16_t)(lsb | msb);

                    gen_int[loop].gen_int.hysteresis = lsb_msb & BMA530_GEN_INT_HYST_MSK;

                    gen_int[loop].gen_int.criterion_sel = (lsb_msb & BMA530_GEN_INT_CRIT_SEL_MSK) >>
                                                          BMA530_GEN_INT_CRIT_SEL_POS;

                    gen_int[loop].gen_int.acc_ref_up = (lsb_msb & BMA530_GEN_INT_ACC_REF_UP_MSK) >>
                                                       BMA530_GEN_INT_ACC_REF_UP_POS;

                    /* Settings 3 */
                    /* Get word to calculate duration and wait time from same word */
                    lsb = (uint16_t) int_1_data[idx++];
                    msb = ((uint16_t) int_1_data[idx++] << 8);
                    lsb_msb = (uint16_t)(lsb | msb);

                    gen_int[loop].gen_int.duration = lsb_msb & BMA530_GEN_INT_DURATION_MSK;

                    gen_int[loop].gen_int.wait_time = (lsb_msb & BMA530_GEN_INT_WAIT_TIME_MSK) >>
                                                      BMA530_GEN_INT_WAIT_TIME_POS;

                    /* Settings 4 */
                    /* Get word to calculate quiet time */
                    lsb = (uint16_t) int_1_data[idx++];
                    msb = ((uint16_t) int_1_data[idx++] << 8);
                    lsb_msb = (uint16_t)(lsb | msb);

                    gen_int[loop].gen_int.quiet_time = lsb_msb & BMA530_GEN_INT_QUIET_TIME_MSK;

                    /* Settings 5 */
                    /* Get word to calculate ref_acc_x */
                    lsb = (uint16_t) int_1_data[idx++];
                    msb = ((uint16_t) int_1_data[idx++] << 8);
                    lsb_msb = (uint16_t)(lsb | msb);

                    gen_int[loop].gen_int.ref_acc_x = (int16_t)(lsb_msb & BMA530_GEN_INT_REF_ACC_X_MSK);

                    /* Settings 6 */
                    /* Get word to calculate ref_acc_y */
                    lsb = (uint16_t) int_1_data[idx++];
                    msb = ((uint16_t) int_1_data[idx++] << 8);
                    lsb_msb = (uint16_t)(lsb | msb);

                    gen_int[loop].gen_int.ref_acc_y = (int16_t)(lsb_msb & BMA530_GEN_INT_REF_ACC_Y_MSK);

                    /* Settings 7 */
                    /* Get word to calculate ref_acc_z */
                    lsb = (uint16_t) int_1_data[idx++];
                    msb = ((uint16_t) int_1_data[idx++] << 8);
                    lsb_msb = (uint16_t)(lsb | msb);

                    gen_int[loop].gen_int.ref_acc_z = (int16_t)(lsb_msb & BMA530_GEN_INT_REF_ACC_Z_MSK);
                }
            }
        }
    }

    return result;
}

/*!
 * @brief This API sets accel foc configuration
 */
int8_t bma530_set_accel_foc_config(const struct bma530_accel_foc_config *acc_foc, struct bma5_dev *dev)
{
    /* Variable to define error */
    int8_t result = BMA5_OK;

    /* Variable to store base address of accel foc */
    uint8_t data = BMA530_BASE_ADDR_ACC_FOC;

    /* Array to store accel foc config data */
    uint8_t acc_foc_data[10] = { 0 };

    uint16_t reg_data;

    uint16_t foc_off_x, foc_off_y, foc_off_z;
    uint8_t foc_apply_corr, foc_filter_coeff, foc_axis_1g;

    if (acc_foc == NULL)
    {
        result = BMA5_E_NULL_PTR;

    }
    else
    {
        /* Set the accel foc base address to feature engine transmission address to start DMA transaction */
        result = bma5_set_regs(BMA5_REG_FEATURE_DATA_ADDR, &data, 1, dev);

        if (result == BMA5_OK)
        {
            /* Get the configuration from the feature engine register */
            result = bma5_get_regs(BMA5_REG_FEATURE_DATA_TX, acc_foc_data, 10, dev);

            if (result == BMA5_OK)
            {
                /* Settings 1 */

                reg_data = (uint16_t)(((uint16_t)acc_foc_data[2] | ((uint16_t)acc_foc_data[3] << 8)));
                foc_off_x = BMA5_SET_BITS_POS_0(reg_data, BMA530_ACC_FOC_OFF_X, acc_foc->foc_off_x);

                /* Settings 2 */

                reg_data = (uint16_t)(((uint16_t)acc_foc_data[4] | ((uint16_t)acc_foc_data[5] << 8)));
                foc_off_y = BMA5_SET_BITS_POS_0(reg_data, BMA530_ACC_FOC_OFF_Y, acc_foc->foc_off_y);

                /* Settings 3 */

                reg_data = (uint16_t)(((uint16_t)acc_foc_data[6] | ((uint16_t)acc_foc_data[7] << 8)));
                foc_off_z = BMA5_SET_BITS_POS_0(reg_data, BMA530_ACC_FOC_OFF_Z, acc_foc->foc_off_z);

                /* Settings 4 */

                reg_data = (uint16_t)(((uint16_t)acc_foc_data[8] | ((uint16_t)acc_foc_data[9] << 8)));
                foc_apply_corr = (uint8_t)BMA5_SET_BITS_POS_0(reg_data,
                                                              BMA530_ACC_FOC_APPLY_CORR,
                                                              acc_foc->foc_apply_corr);

                foc_filter_coeff = (uint8_t)BMA5_SET_BITS(reg_data,
                                                          BMA530_ACC_FOC_FILTER_COEFF,
                                                          acc_foc->foc_filter_coeff);

                foc_axis_1g = (uint8_t)BMA5_SET_BITS(reg_data, BMA530_ACC_FOC_AXIS_1G, acc_foc->foc_axis_1g);

                acc_foc_data[0] = (uint8_t)(foc_off_x & 0x00FF);
                acc_foc_data[1] = (uint8_t)((foc_off_x & 0xFF00) >> 8);
                acc_foc_data[2] = (uint8_t)(foc_off_y & 0x00FF);
                acc_foc_data[3] = (uint8_t)((foc_off_y & 0xFF00) >> 8);
                acc_foc_data[4] = (uint8_t)(foc_off_z & 0x00FF);
                acc_foc_data[5] = (uint8_t)((foc_off_z & 0xFF00) >> 8);
                acc_foc_data[6] = (uint8_t)(foc_apply_corr | foc_filter_coeff | foc_axis_1g);

                /* Set the configuration from the feature engine register */
                result = bma5_set_regs(BMA5_REG_FEATURE_DATA_TX, acc_foc_data, 8, dev);
            }
        }
    }

    return result;
}

/*!
 * @brief This API gets accel foc configuration
 */
int8_t bma530_get_accel_foc_config(struct bma530_accel_foc_config *acc_foc, struct bma5_dev *dev)
{
    /* Variable to define error */
    int8_t result = BMA5_OK;

    /* Variable to store base address of accel foc */
    uint8_t data = BMA530_BASE_ADDR_ACC_FOC;

    /* Array to store accel foc config data */
    uint8_t acc_foc_data[10] = { 0 };

    /* Variable to define array offset */
    uint8_t idx = 0;

    /* Variable to define LSB */
    uint16_t lsb;

    /* Variable to define MSB */
    uint16_t msb;

    /* Variable to define a word */
    uint16_t lsb_msb;

    if (acc_foc == NULL)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        /* Set the accel foc base address to feature engine transmission address to start DMA transaction */
        result = bma5_set_regs(BMA5_REG_FEATURE_DATA_ADDR, &data, 1, dev);

        if (result == BMA5_OK)
        {
            /* Get the configuration from the feature engine register */
            result = bma5_get_regs(BMA5_REG_FEATURE_DATA_TX, acc_foc_data, 10, dev);

            if (result == BMA5_OK)
            {
                /* First two bytes are dummy bytes */
                idx = 2;

                /* Settings 1 */
                /* Get word to calculate foc_off_x */
                lsb = (uint16_t) acc_foc_data[idx++];
                msb = ((uint16_t) acc_foc_data[idx++] << 8);
                lsb_msb = (uint16_t)(lsb | msb);

                acc_foc->foc_off_x = lsb_msb & BMA530_ACC_FOC_OFF_X_MSK;

                /* Settings 2 */
                /* Get word to calculate foc_off_y */
                lsb = (uint16_t) acc_foc_data[idx++];
                msb = ((uint16_t) acc_foc_data[idx++] << 8);
                lsb_msb = (uint16_t)(lsb | msb);

                acc_foc->foc_off_y = lsb_msb & BMA530_ACC_FOC_OFF_Y_MSK;

                /* Settings 3 */
                /* Get word to calculate foc_off_z */
                lsb = (uint16_t) acc_foc_data[idx++];
                msb = ((uint16_t) acc_foc_data[idx++] << 8);
                lsb_msb = (uint16_t)(lsb | msb);

                acc_foc->foc_off_z = lsb_msb & BMA530_ACC_FOC_OFF_Z_MSK;

                /* Settings 4 */
                /* Get word to calculate foc_apply_corr, foc_filter_coeff and foc_axis_1g from same word  */
                lsb = (uint16_t) acc_foc_data[idx++];
                msb = ((uint16_t) acc_foc_data[idx++] << 8);
                lsb_msb = (uint16_t)(lsb | msb);

                acc_foc->foc_apply_corr = lsb_msb & BMA530_ACC_FOC_APPLY_CORR_MSK;

                acc_foc->foc_filter_coeff = (lsb_msb & BMA530_ACC_FOC_FILTER_COEFF_MSK) >>
                                            BMA530_ACC_FOC_FILTER_COEFF_POS;

                acc_foc->foc_axis_1g = (lsb_msb & BMA530_ACC_FOC_AXIS_1G_MSK) >> BMA530_ACC_FOC_AXIS_1G_POS;
            }
        }
    }

    return result;
}

/*!
 * @brief This API sets feature axis configurations
 */
int8_t bma530_set_feature_axis_config(const struct bma530_feat_axis *feat_axis, struct bma5_dev *dev)
{
    /* Variable to define error */
    int8_t result = BMA5_OK;

    /* Variable to store base address of feature axis */
    uint8_t data = BMA530_BASE_ADDR_GENERAL_SETTINGS;

    /* Array to store feature axis data */
    uint8_t feat_axis_data[4] = { 0 };

    if (feat_axis == NULL)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        /* Set the feature axis base address to feature engine transmission address to start DMA transaction */
        result = bma5_set_regs(BMA5_REG_FEATURE_DATA_ADDR, &data, 1, dev);

        if (result == BMA5_OK)
        {
            /* Get the configuration from the feature engine register */
            result = bma5_get_regs(BMA5_REG_FEATURE_DATA_TX, feat_axis_data, 4, dev);

            if (result == BMA5_OK)
            {
                feat_axis_data[2] = BMA5_SET_BITS(feat_axis_data[2], BMA530_FEAT_AXIS_EX, feat_axis->feat_axis_ex);

                feat_axis_data[2] = BMA5_SET_BITS(feat_axis_data[2], BMA530_FEAT_X_INV, feat_axis->feat_x_inv);

                feat_axis_data[2] = BMA5_SET_BITS(feat_axis_data[2], BMA530_FEAT_Y_INV, feat_axis->feat_y_inv);

                feat_axis_data[2] = BMA5_SET_BITS(feat_axis_data[2], BMA530_FEAT_Z_INV, feat_axis->feat_z_inv);

                feat_axis_data[0] = feat_axis_data[2];
                feat_axis_data[1] = feat_axis_data[3];

                /* Set the configuration from the feature engine register */
                result = bma5_set_regs(BMA5_REG_FEATURE_DATA_TX, feat_axis_data, 2, dev);
            }
        }
    }

    return result;
}

/*!
 * @brief This API gets feature axis configurations
 */
int8_t bma530_get_feature_axis_config(struct bma530_feat_axis *feat_axis, struct bma5_dev *dev)
{
    /* Variable to define error */
    int8_t result = BMA5_OK;

    /* Variable to store base address of feature axis */
    uint8_t data = BMA530_BASE_ADDR_GENERAL_SETTINGS;

    /* Array to store feature axis data */
    uint8_t feat_axis_data[4] = { 0 };

    if (feat_axis == NULL)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        /* Set the feature axis base address to feature engine transmission address to start DMA transaction */
        result = bma5_set_regs(BMA5_REG_FEATURE_DATA_ADDR, &data, 1, dev);

        if (result == BMA5_OK)
        {
            /* Get the configuration from the feature engine register */
            result = bma5_get_regs(BMA5_REG_FEATURE_DATA_TX, feat_axis_data, 4, dev);

            if (result == BMA5_OK)
            {
                feat_axis->feat_axis_ex = BMA5_GET_BITS(feat_axis_data[2], BMA530_FEAT_AXIS_EX);

                feat_axis->feat_x_inv = BMA5_GET_BITS(feat_axis_data[2], BMA530_FEAT_X_INV);

                feat_axis->feat_y_inv = BMA5_GET_BITS(feat_axis_data[2], BMA530_FEAT_Y_INV);

                feat_axis->feat_z_inv = BMA5_GET_BITS(feat_axis_data[2], BMA530_FEAT_Z_INV);
            }
        }
    }

    return result;
}

/*!
 * @brief This API resets sensor. All registers are overwritten with
 * their default values.
 */
int8_t bma530_soft_reset(struct bma5_dev *dev)
{
    /* Variable to store the function result */
    int8_t result;

    /* Variable to perform dummy read */
    uint8_t dummy_read;

    /* Assign soft-reset command */
    uint8_t cmd = BMA5_CMD_SOFTRESET;

    /* Null-pointer check */
    result = verify_handle(dev);

    if (result != BMA5_OK)
    {
        result = BMA5_E_NULL_PTR;
    }
    else
    {
        if (dev->intf == BMA5_SPI_INTF)
        {
            dev->dummy_byte = 1;
        }
        else
        {
            dev->dummy_byte = 0;
        }

        /* Set soft-reset command */
        result = bma5_set_regs(BMA5_REG_CMD, &cmd, 1, dev);

        if (result == BMA5_OK)
        {
            /* Provide 2ms delay after soft-reset */
            dev->delay_us(BMA530_SOFT_RESET_DELAY, dev->intf_ptr);

            /* Dummy read is performed after soft-reset */
            result = bma5_get_regs(BMA530_REG_CHIP_ID, &dummy_read, 1, dev);
        }
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
