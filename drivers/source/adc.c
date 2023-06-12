/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/* Includes */
#include "adc.h"

/**
 * \fn          void adc_irq_handler(ADC120_Type *adc, conv_info_t *conversion)
 * \brief       Handle interrupts for the ADC instance.
 * \param[in]   adc        : Pointer to the ADC register map
 * \param[in]   conversion : The conversion structure for the ADC instance
 * \return      none
*/
void adc_irq_handler(ADC120_Type *adc, conv_info_t *conversion)
{
    volatile uint32_t *sample_reg_ptr = &adc->ADC_SAMPLE_REG_0;

    /* Clearing the done IRQ*/
    adc->ADC_INTERRUPT = ADC_INTR_DONE_CLEAR;

    /* Single scan mode */
    if ((conversion->sequencer_ctrl_status) == ADC_SINGLE_SCAN_MODE)
    {
        /* read sample and store to user memory */
        *(conversion->conv_buff + conversion->curr_cnt) = *(sample_reg_ptr+ conversion->user_input);
    }

    /* Continuous scan mode */
    if ((conversion->sequencer_ctrl_status) == ADC_CONTINUOUS_SCAN_MODE)
    {
        /* validation for last read channel */
        if(conversion->last_read_channel > ADC_LAST_AVAILABLE_CHANNEL)
        {
           conversion->last_read_channel = 0;
        }

        while((adc->ADC_SEQUENCER_CTRL >> conversion->last_read_channel) & ADC_SEQUENCER_MSK_BIT)
        {
            if(conversion->last_read_channel >= ADC_LAST_AVAILABLE_CHANNEL)
            {
                conversion->last_read_channel = 0;
            }
            else
            {
                conversion->last_read_channel ++;
            }
         }

        *(conversion->conv_buff + conversion->curr_cnt) = *(sample_reg_ptr + conversion->last_read_channel);

        conversion->last_read_channel++;
    }

    conversion->curr_cnt += 1;

    if(conversion->curr_cnt >= conversion->total_cnt)
    {
        /* disable irq */
        adc_mask_interrupt(adc);

        conversion->status |= ADC_CONV_STAT_COMPLETE;
    }

    /* only one interrupt is working so comparator interrupt we are checking in this
     * handler based on the interrupt status */

    /* below is for comparator0 interrupt  */
    if((conversion->comp_ctrl_status) == 0x01 && (adc->ADC_INTERRUPT & ADC_INTR_COMP0_MSK))
    {
        switch ((adc->ADC_CONTROL >> ADC_SHIFT_BIT) & ADC_THRSHLD_COMP_MASK_BIT)
        {
        case ADC_COMP_THRHLD_ABOVE_A:
             conversion->status |= ADC_CONV_STAT_CMP_THLD_ABOVE_A;
             break;
        case ADC_COMP_THRHLD_BELOW_A:
             conversion->status |= ADC_CONV_STAT_CMP_THLD_BELOW_A;
             break;
        case ADC_COMP_THRHLD_BETWEEN_A_B:
             conversion->status |= ADC_CONV_STAT_CMP_THLD_BETWEEN_A_B;
             break;
        }
    adc->ADC_INTERRUPT = ADC_INTR_COMP0_CLEAR;
    }

    /* below is for comparator1 */
    if((conversion->comp_ctrl_status) == 0x01 && (adc->ADC_INTERRUPT & ADC_INTR_COMP1_MSK))
    {
        switch ((adc->ADC_CONTROL >> ADC_SHIFT_BIT) & ADC_THRSHLD_COMP_MASK_BIT)
        {
        case ADC_COMP_THRHLD_ABOVE_B:
             conversion->status |= ADC_CONV_STAT_CMP_THLD_ABOVE_B;
             break;
        case ADC_COMP_THRHLD_BELOW_B:
             conversion->status |= ADC_CONV_STAT_CMP_THLD_BELOW_B;
             break;
        case ADC_COMP_THRHLD_OUTSIDE_A_B:
             conversion->status |= ADC_CONV_STAT_CMP_THLD_OUTSIDE_A_B;
             break;
        }
    adc->ADC_INTERRUPT = ADC_INTR_COMP1_CLEAR;
    }
}

/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
