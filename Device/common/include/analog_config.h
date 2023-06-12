/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

#ifndef ANALOG_CONFIG_H_
#define ANALOG_CONFIG_H_

#include "peripheral_types.h"

#define VBAT_ANA_REG1_VAL          0x02441A80
#define VBAT_ANA_REG2_VAL          0x00CC0630
#define VBAT_ANA_REG3_VAL          0xF8448004
#define DCDC_ANA_REG1_VAL          0xA14DE693
#define DCDC_ANA_REG2_VAL          0x0B014404
#define COMP_REG2_VAL              0x0C18C210
#define ADC_REG1_VAL               0x001D0410

/**
 @fn          void analog_config_vbat_reg1(void)
 @brief       Assigning Vbat registers values to the Vbat register1 base address
 @param[in]   none
 @return      none
 */
static inline void analog_config_vbat_reg1(void)
{
    /* Analog configuration Vbat register1 */
    ANA_REG->VBAT_ANA_REG1 = VBAT_ANA_REG1_VAL;
}

/* Analog configuration:
   Vbat register2 contains below parameters:
   -pmubg_vref_cont,
   -dig_ldo_18_en,
   -dig_ldo_cont,
   -phy_ldo_en,
   -phy_ldo_cont,
   -osc_38Mrc_cont,
   -ana_periph_bg_en,
   -ana_periph_bg_cont,
   -ana_periph_ldo_cont,
   -ana_periph_ldo_en,
*/

/**
 @fn          void analog_config_vbat_reg2(void)
 @brief       Assigning Vbat registers values to the Vbat register2 base address
 @param[in]   none
 @return      none
 */
static inline void analog_config_vbat_reg2(void)
{
    /* Analog configuration Vbat register2 */
    ANA_REG->VBAT_ANA_REG2 = VBAT_ANA_REG2_VAL;
}

/**
 @fn          void analog_config_vbat_reg3(void)
 @brief       Assigning Vbat registers values to the Vbat register3 base address
 @param[in]   none
 @return      none
 */
static inline void analog_config_vbat_reg3(void)
{
    /* Analog configuration Vbat register3 */
    ANA_REG->VBAT_ANA_REG3 = VBAT_ANA_REG3_VAL;
}

/**
 @fn          void analog_config_dcdc_reg1(void)
 @brief       Assigning DCDC registers values to the DCDC register1 base address
 @param[in]   none
 @return      none
 */
static inline void analog_config_dcdc_reg1(void)
{
    /* Analog configuration DCDC register1 */
    ANA_REG->DCDC_REG1 = DCDC_ANA_REG1_VAL;
}

/**
 @fn          void analog_config_dcdc_reg2(void)
 @brief       Assigning DCDC registers values to the DCDC register2 base address
 @param[in]   none
 @return      none
 */
static inline void analog_config_dcdc_reg2(void)
{
    /* Analog configuration DCDC register2 */
    ANA_REG->DCDC_REG2 = DCDC_ANA_REG2_VAL;
}

/* Analog configuration:
   comparator register2 contains below parameters:
   -comp_lpo_in_p_sel,
   -comp_lpo_in_m_sel,
   -comp_lpo_hyst,
   -comp_lp_en,
   -dac6_en,
   -dac6_vref_scale,
   -dac6_cont,
   -adc_vref_cont,
   -adc_vref_buf_rdiv_en,
   -adc_vref_buf_en
*/

/**
 @fn          void analog_config_comp_reg2(void)
 @brief       Assigning comparator register2 values to the comparator
              register2 base address
 @param[in]   none
 @return      none
 */
static inline void analog_config_comp_reg2(void)
{
    /* Analog configuration comparator register2 */
    CMP_REG->CMP_COMP_REG2 = COMP_REG2_VAL;
}

/**
 @fn          void analog_config_adc_reg1(void)
 @brief       Assigning ADC registers values to the ADC register1 base address
 @param[in]   none
 @return      none
 */
static inline void analog_config_adc_reg1(void)
{
    /* Analog configuration ADC register1 */
    //ADC_REG->ADC_REG1 = ADC_REG1_VAL;
}

#endif /* ANALOG_CONFIG_H_ */
