/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

#ifndef DAC_H_
#define DAC_H_

#ifdef  __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/**
 @brief struct DAC_Type:- Register map for DAC
 */
typedef struct
{
    volatile uint32_t REG1;   /* DAC REGISTER1 */
    volatile uint32_t INPUT;  /* DAC INPUT register */
}DAC_Type;

/* DAC  Control register */
#define DAC_EN                (1U << 0)   /* Enable DAC */
#define DAC_RESET             (1U << 27)  /* 0=Reset,this will reset the DAC */
#define DAC_HP_MODE_EN        (1U << 18)  /* To enable the dac output buffer */
#define DAC_MAX_INPUT         (0xFFFU)    /* Maximum input for the DAC is 4095(DAC 12 bit resolution) */

/**
 @fn           void dac_enable(DAC_Type *dac)
 @brief        Enable the DAC.
 @param[in]    dac : Pointer to dac Type
 @return       none
 */
static inline void dac_enable(DAC_Type *dac)
{
    /* Enable the DAC */
    dac->REG1 |= (DAC_EN);
}

/**
 @fn           void dac_disable(DAC_Type *dac)
 @brief        Disable the DAC.
 @param[in]    dac : Pointer to dac Type
 @return       none
 */
static inline void dac_disable(DAC_Type *dac)
{
    /* Disable the DAC */
    dac->REG1 &= ~(DAC_EN);
}

/**
 @fn           void dac_set_config(DAC_Type *dac, uint32_t value)
 @brief        Configure the DAC
 @param[in]    value : to set Bypass mode, capacitor and resistor control, ibias
                       value,unsigned input data and to provide the input data in
                       dac_in register.
 @param[in]    dac   : Pointer to dac Type
 @return       none
 */
static inline void dac_set_config(DAC_Type *dac ,uint32_t value)
{
    /* Adding value to the register */
    dac->REG1 |= value;
}

/**
 @fn           void dac_clear_config (DAC_Type *dac)
 @brief        Clear the DAC configuration.
 @param[in]    dac : Pointer to dac Type
 @return       none
 */
static inline void dac_clear_config(DAC_Type *dac)
{
    /* Clear the DAC configuration */
    dac->REG1 = 0U;
}

/**
 @fn           DAC_HP_MODE(DAC_Type *dac)
 @brief        Enable HP mode of DAC.
 @param[in]    dac : Pointer to dac Type
 @return       none
 */
static inline void DAC_HP_MODE(DAC_Type *dac)
{
    /* To enable the output buffer of DAC */
    dac->REG1 |= DAC_HP_MODE_EN;
}

/**
 @fn           void reset_dac_blocks(DAC_Type *dac)
 @brief        Reset the DAC.
 @param[in]    dac : Pointer to dac Type
 @return       none
 */
static inline void dac_reset(DAC_Type *dac)
{
    /* Reset the DAC */
    dac->REG1 &= ~(DAC_RESET);
}

/**
 @fn           void dac_input(DAC_Type *dac, uint32_t value)
 @brief        Set the DAC input.
 @param[in]    value : DAC input
 @param[in]    dac   : Pointer to dac Type
 @return       none
 */
static inline void dac_input(DAC_Type *dac, uint32_t value)
{
    /* set the DAC input */
    dac->INPUT = value;
}

#endif /* DAC_H */
