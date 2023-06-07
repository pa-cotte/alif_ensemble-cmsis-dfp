/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

#include "pdm.h"

/**
  @fn          void pdm_error_detect_irq_handler(PDM_Type *pdm);
  @brief       IRQ handler for the error interrupt
  @param[in]   pdm      : Pointer to the PDM register map
  @return      none
*/
void pdm_error_detect_irq_handler(PDM_Type *pdm)
{
    pdm->PDM_IRQ_ENABLE &= ~(PDM_FIFO_OVERFLOW_IRQ);
}

/**
  @fn          void pdm_audio_detect_irq_handler(PDM_Type *pdm, pdm_transfer_t *transfer )
  @brief       IRQ handler for the audio detect interrupt
  @param[in]   pdm      : Pointer to the PDM register map
  @param[in]   transfer : The transfer structure of the PDM instance
  @return      none
*/
void pdm_audio_detect_irq_handler(PDM_Type *pdm, pdm_transfer_t *transfer )
{
    /* Check current count is greater than the buffer size */
    if (transfer->curr_cnt  >= (transfer->total_cnt))
    {
        transfer->status |= PDM_AUDIO_STATUS_DETECTION;

        /* disable irq */
        pdm->PDM_IRQ_ENABLE &= ~(PDM_AUDIO_DETECT_IRQ_STAT);
    }
    (void) pdm->PDM_AUDIO_DETECT_IRQ;
}

/**
  @fn          void pdm_warning_irq_handler(PDM_Type *pdm, pdm_transfer_t *transfer)
  @brief       IRQ handler for the PDM warning interrupt.
  @param[in]   pdm      : Pointer to the PDM register map
  @param[in]   transfer : The transfer structure of the PDM instance
  @return      none
*/
void pdm_warning_irq_handler(PDM_Type *pdm, pdm_transfer_t *transfer)
{
    uint8_t fifo_count;
    bool fifo_almost_full_irq;
    uint32_t audio_ch;
    uint32_t audio_ch_0_1;
    uint32_t audio_ch_2_3;
    uint32_t audio_ch_4_5;
    uint32_t audio_ch_6_7;

    fifo_count = pdm->PDM_FIFO_STAT;

    fifo_almost_full_irq = pdm->PDM_WARN_IRQ;

    /* User enabled channel */
    audio_ch = pdm->PDM_CTL0 & PDM_CHANNEL_ENABLE;

    if(fifo_almost_full_irq == 1)
    {
        for(uint32_t count = 0; count < fifo_count; count++)
        {
            audio_ch_0_1 = pdm->PDM_CH0_CH1_AUDIO_OUT;
            audio_ch_2_3 = pdm->PDM_CH2_CH3_AUDIO_OUT;
            audio_ch_4_5 = pdm->PDM_CH4_CH5_AUDIO_OUT;
            audio_ch_6_7 = pdm->PDM_CH6_CH7_AUDIO_OUT;

            if(transfer->curr_cnt < transfer->total_cnt)
            {
                /* Check for channel 0 and 1 */
                if(audio_ch & PDM_CHANNEL_0_1)
                {
                    /* Store the ch 0 and 1 audio output values in the user buffer memory */
                    transfer->ch0_1_addr[transfer->curr_cnt] = audio_ch_0_1;
                }

                /* Check for channel 2 and 3 */
                if(audio_ch & PDM_CHANNEL_2_3)
                {
                    /* Store the ch 2 and 3 audio output values in the user buffer memory */
                    transfer->ch2_3_addr[transfer->curr_cnt] = audio_ch_2_3;
                }

                /* Check for channel 4 and 5 */
                if(audio_ch & PDM_CHANNEL_4_5)
                {
                    /* Store the ch 4 and 5 audio output values in the user buffer memory */
                    transfer->ch4_5_addr[transfer->curr_cnt] = audio_ch_4_5;
                }

                /* Check for channel 6 and 7 */
                if(audio_ch & PDM_CHANNEL_6_7)
                {
                    /* Store the ch 6 and 7 audio output values in the user buffer memory */
                    transfer->ch6_7_addr[transfer->curr_cnt] = audio_ch_6_7;
                }
                transfer->curr_cnt ++;
            }
        }
    }

    if (transfer->curr_cnt  >= (transfer->total_cnt ))
    {
        if(fifo_almost_full_irq== 1)
        {
            /* disable irq */
            pdm->PDM_IRQ_ENABLE &= ~(PDM_FIFO_ALMOST_FULL_IRQ | PDM_AUDIO_DETECT_IRQ_STAT | PDM_FIFO_OVERFLOW_IRQ);
            transfer->status |=  PDM_CAPTURE_STATUS_COMPLETE;
        }
    }

    (void) pdm->PDM_ERROR_IRQ;
}
