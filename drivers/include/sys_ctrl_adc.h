/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

#ifndef SYS_CTRL_ADC_H_
#define SYS_CTRL_ADC_H_

#include "peripheral_types.h"

/* ADC120 */
static inline void enable_adc0_periph_clk(void)
{
    CLKCTL_PER_SLV->ADC_CTRL |= ADC_CTRL_ADC0_CKEN;
}

static inline void disable_adc0_periph_clk(void)
{
    CLKCTL_PER_SLV->ADC_CTRL &= ~ADC_CTRL_ADC0_CKEN;
}

/* ADC121 */
static inline void enable_adc1_periph_clk(void)
{
    CLKCTL_PER_SLV->ADC_CTRL |= ADC_CTRL_ADC1_CKEN;
}

static inline void disable_adc1_periph_clk(void)
{
    CLKCTL_PER_SLV->ADC_CTRL &= ~ADC_CTRL_ADC1_CKEN;
}

/* ADC122 */
static inline void enable_adc2_periph_clk(void)
{
    CLKCTL_PER_SLV->ADC_CTRL |= ADC_CTRL_ADC2_CKEN;
}

static inline void disable_adc2_periph_clk(void)
{
    CLKCTL_PER_SLV->ADC_CTRL &= ~ADC_CTRL_ADC2_CKEN;
}

#endif /* SYS_CTRL_ADC_H_ */
