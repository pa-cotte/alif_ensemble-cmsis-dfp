/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/**************************************************************************//**
 * @file     Driver_PDM.h
 * @author   Nisarga A M
 * @email    nisarga.am@alifsemi.com
 * @version  V1.0.0
 * @date     12-Jan-2023
 * @brief    CMSIS-Driver for PDM.
 * @bug      None.
 * @Note     None
 ******************************************************************************/

/* Project Includes */
#include "Driver_PDM.h"
#include "PDM_dev.h"

#define ARM_PDM_DRV_VERSION    ARM_DRIVER_VERSION_MAJOR_MINOR(1, 0)  /*  Driver version */

/*Driver version*/
static const ARM_DRIVER_VERSION DriverVersion = {
    ARM_PDM_API_VERSION,
    ARM_PDM_DRV_VERSION
};

/*Driver Capabilities   */
static const ARM_PDM_CAPABILITIES DriverCapabilities = {
    1,  /* Supports Microphone sleep mode*/
    1,  /* Supports Standard voice mode */
    1,  /* Supports High quality voice mode */
    1,  /* Supports Wide bandwidth audio mode */
    1,  /* Supports Full bandwidth audio mode */
    1,  /* Supports Ultrasound mode */
    0   /* Reserved ( must be ZERO) */
};

/**
 @fn           ARM_DRIVER_VERSION PDM_GetVersion(void)
 @brief        get PDM version
 @param        none
 @return       driver version
 */
static ARM_DRIVER_VERSION PDM_GetVersion(void)
{
    return DriverVersion;
}

/**
 @fn           ARM_PDM_CAPABILITIES PDM_GetCapabilities(void)
 @brief        get PDM Capabilites
 @param        none
 @return       driver Capabilites
 */
static ARM_PDM_CAPABILITIES PDM_GetCapabilities(void)
{
    return DriverCapabilities;
}

/**
 @fn          void pdm_bypass_iir (int arg,PDM_resources_t *PDM)
 @brief       To select the Bypass DC blocking IIR filter
 @param[in]   pdm : Pointer to PDM resources
 @param[in]   arg : To enable or disable the bypass IIR filter
 @return      none
 */
static void pdm_bypass_iir(int arg, PDM_resources_t *pdm)
{
    if (arg)  /* To enable the bypass IIR filter */
        pdm->reg_base->AUDIO_CNTRL_1 |= PDM_BYPASS_IIR;

    else  /* To disable the bypass IIR filter */
        pdm->reg_base->AUDIO_CNTRL_1 &= ~(PDM_BYPASS_IIR);
}

/**
 @fn          void pdm_bypass_fir (int arg,PDM_resources_t *PDM)
 @brief       To select the Bypass FIR filter
 @param[in]   pdm : Pointer to PDM resources
 @param[in]   arg : To enable or disable the bypass FIR filter
 @return      none
 */
static void pdm_bypass_fir(int arg, PDM_resources_t *pdm)
{
    if (arg)  /* To enable the bypass FIR filter */
        pdm->reg_base->AUDIO_CNTRL_1 |= PDM_BYPASS_FIR;

    else  /* To disable the bypass FIR filter */
        pdm->reg_base->AUDIO_CNTRL_1 &= ~(PDM_BYPASS_FIR);
}

/**
 @fn          void pdm_peak_detect (int arg,PDM_resources_t *PDM)
 @brief       To select the Bypass FIR filter
 @param[in]   pdm : Pointer to PDM resources
 @param[in]   arg : To enable or disable the peak detection node
 @return      none
 */
static void pdm_peak_detect(int arg, PDM_resources_t *pdm)
{
    if(arg)  /* peak detection after gain stage */
        pdm->reg_base->AUDIO_CNTRL_1 |= PDM_PEAK_DETECT_NODE;

    else  /* peak detection before gain stage */
        pdm->reg_base->AUDIO_CNTRL_1 &= ~(PDM_PEAK_DETECT_NODE);
}

/**
 @fn          void pdm_sample_advance (int arg,PDM_resources_t *PDM)
 @brief       To select the Sample advance
 @param[in]   pdm : Pointer to PDM resources
 @param[in]   arg : To enable or disable the Sample advance
 @return      none
 */
static void pdm_sample_advance(int arg, PDM_resources_t *pdm)
{
    if(arg)  /* To enable the Sample advance */
        pdm->reg_base->AUDIO_CNTRL_1 |= PDM_SAMPLE_ADV;

    else  /* To disable the Sample advance */
        pdm->reg_base->AUDIO_CNTRL_1 &= ~(PDM_SAMPLE_ADV);
}

/**
 @fn          void pdm_dma_handshake (int arg,PDM_resources_t *PDM)
 @brief       To Use DMA handshaking signals for flow control
 @param[in]   pdm : Pointer to PDM resources
 @param[in]   arg : To enable or disable the DMA Handshake
 @return      none
 */
static void pdm_dma_handshake(int arg, PDM_resources_t *pdm)
{
    if(arg)  /* To enable the DMA handshake */
        pdm->reg_base->AUDIO_CNTRL_1 |= PDM_DMA_HANDSHAKE;

    else  /* To enable the DMA handshake */
        pdm->reg_base->AUDIO_CNTRL_1 &= ~(PDM_DMA_HANDSHAKE);
}

/**
 @fn          void pdm_synchronize_clk (PDM_resources_t *PDM)
 @brief       To give four APB transactions delay
 @param[in]   pdm : Pointer to PDM resources
 @return      None
 */
static void pdm_synchronize_clk(PDM_resources_t *pdm)
{
    uint32_t rd_data;

    /* Firmware should avoid writing back-to-back values to AUDIO_CNTRL_0 reg
       to synchronize audio clock domain
     * Inserting four APB transactions between consecutive writes would be sufficient
       for this purpose
    */
    rd_data = pdm->reg_base->AUDIO_CNTRL_0;
    rd_data = pdm->reg_base->AUDIO_CNTRL_0;
    rd_data = pdm->reg_base->AUDIO_CNTRL_0;
    rd_data = pdm->reg_base->AUDIO_CNTRL_0;

    (void)rd_data;
}

/**
 @fn          int32_t pdm_enable_irq (PDM_resources_t *PDM)
 @brief       Enable the IRQ
 @param[in]   pdm : Pointer to PDM resources
 @return      ARM_DRIVER_OK : if enable the PDM is successful
 */
static int32_t pdm_enable_irq(PDM_resources_t *pdm)
{
    uint32_t audio_ch;

    pdm->reg_base->IRQ_ENABLE &= ~(PDM_IRQ_ENABLE); /* Clear IRQ */

    /* User enabled channel */
    audio_ch = ((pdm->reg_base->AUDIO_CNTRL_0)) & PDM_CHANNEL_ENABLE;

    /* Enable the Interrupt */
    pdm->reg_base->IRQ_ENABLE |= (( audio_ch  << 8) | (PDM_FIFO_ALMOST_FULL_IRQ |PDM_FIFO_OVERFLOW_IRQ));

    return ARM_DRIVER_OK;
}

/**
 @fn          void PDM_ERROR_IRQ_handler (PDM_resources_t *PDM)
 @brief       IRQ handler for the error interrupt
 @param[in]   pdm : Pointer to PDM resources
 @return      none
 */
void PDM_ERROR_IRQ_handler(PDM_resources_t *pdm)
{
    int32_t event = 0U;  /* Storing event call */

    uint32_t error_info = *(uint32_t *)(pdm->reg_base->ERROR_STAT);

    /* Check for error interrupt */
    if(error_info & PDM_INTERRUPT_STATUS_VALUE)
    {
        event |= ARM_PDM_EVENT_ERROR;

        /* disable irq */
        pdm->reg_base->IRQ_ENABLE &= ~(PDM_FIFO_OVERFLOW_IRQ);
        NVIC_DisableIRQ(pdm->error_irq);
    }

    if((event != 0U) && (pdm->cb_event != NULL))
    {
        /* call user callback */
        pdm->cb_event(event);
    }

}

/**
 @fn          void PDM_AUDIO_DETECT_IRQ_handler (PDM_resources_t *PDM)
 @brief       IRQ handler for the audio detect interrupt
 @param[in]   pdm : Pointer to PDM resources
 @return      none
 */
void PDM_AUDIO_DETECT_IRQ_handler (PDM_resources_t *pdm)
{
    int32_t event = 0U;

    /* Check current count is greater than the buffer size */
    if (pdm->info.curr_cnt  >= (pdm->info.total_cnt))
    {
        event |= ARM_PDM_EVENT_AUDIO_DETECTION;

        /* disable irq */
        pdm->reg_base->IRQ_ENABLE &= ~(PDM_AUDIO_DETECT_IRQ);
        NVIC_DisableIRQ(pdm->audio_detect_irq);
    }

    if((event != 0U) && (pdm->cb_event != NULL))
    {
        /* call user callback */
        pdm->cb_event(event);
    }
}

/**
 @fn          void PDM_WARNING_IRQ_handler (PDM_resources_t *PDM)
 @brief       IRQ handler for the PDM warning interrupt
 @param[in]   pdm : Pointer to PDM resources
 @return      none
 */
void PDM_WARNING_IRQ_handler(PDM_resources_t *pdm)
{
    uint32_t fifo_count;
    uint32_t audio_ch;
    uint32_t audio_ch_0_1;
    uint32_t audio_ch_2_3;
    uint32_t audio_ch_4_5;
    uint32_t audio_ch_6_7;
    int32_t event = 0U;
    uint32_t i;

    /* Store the fifo count value */
    fifo_count = (pdm->reg_base->FIFO_STAT) ;

    /* User enabled channel */
    audio_ch = ((pdm->reg_base->AUDIO_CNTRL_0)) & PDM_CHANNEL_ENABLE;

    for(i = 0; i < fifo_count; i++)
    {
        audio_ch_0_1 = pdm->reg_base->AUDIO_OUT[PDM_AUDIO_CH_0_1];
        audio_ch_2_3 = pdm->reg_base->AUDIO_OUT[PDM_AUDIO_CH_2_3];
        audio_ch_4_5 = pdm->reg_base->AUDIO_OUT[PDM_AUDIO_CH_4_5];
        audio_ch_6_7 = pdm->reg_base->AUDIO_OUT[PDM_AUDIO_CH_6_7];

        if(pdm->info.curr_cnt < pdm->info.total_cnt)
        {
            /* Check for channel 0 and 1 */
            if((audio_ch & PDM_CHANNEL_0_1) )
            {
                /* Store the ch 0 and 1 audio output values in the user buffer memory */
                pdm->info.ch0_1_addr[pdm->info.curr_cnt] = audio_ch_0_1;
            }

            /* Check for channel 2 and 3 */
            if((audio_ch & PDM_CHANNEL_2_3) )
            {
                /* Store the ch 2 and 3 audio output values in the user buffer memory */
                pdm->info.ch2_3_addr[pdm->info.curr_cnt] = audio_ch_2_3;
            }

            /* Check for channel 4 and 5 */
            if((audio_ch & PDM_CHANNEL_4_5) )
            {
                /* Store the ch 4 and 5 audio output values in the user buffer memory */
                pdm->info.ch4_5_addr[pdm->info.curr_cnt] = audio_ch_4_5;
            }

            /* Check for channel 6 and 7 */
            if((audio_ch & PDM_CHANNEL_6_7) )
            {
                /* Store the ch 6 and 7 audio output values in the user buffer memory */
                pdm->info.ch6_7_addr[pdm->info.curr_cnt] = audio_ch_6_7;
            }

            pdm->info.curr_cnt ++;
        }

    }

    if (pdm->info.curr_cnt  >= (pdm->info.total_cnt))
    {
        /* disable irq */
        pdm->reg_base->IRQ_ENABLE &= ~(PDM_FIFO_ALMOST_FULL_IRQ);
        NVIC_DisableIRQ(pdm->warning_irq);

        event |=  ARM_PDM_EVENT_CAPTURE_COMPLETE;
    }

    if((event != 0U) && (pdm->cb_event != NULL))
    {
        /* call user callback */
        pdm->cb_event(event);
    }
}

/**
 @fn          void pdm_expmast0_clk (PDM_resources_t *PDM)
 @brief       Clock configuration for PDM
 @param[in]   pdm : Pointer to PDM resources
 @return      none
 */
static void pdm_expmast0_clk(PDM_resources_t *pdm)
{
    /* Clock configuration is provided on the CFGMST0 address */
    uint32_t *config_address = (uint32_t *)CLKCTL_PER_SLV_BASE;

    /* Enable the clock for PDM */
    *(uint32_t * )config_address = (PDM_CLOCK_CONFIG);
}

/**
@fn          int32_tPDM_Initialize (PDM_resources_t *PDM)
@brief       Initialize the PDM interface
@param[in]   pdm : Pointer to PDM resources
 @return     ARM_DRIVER_ERROR_PARAMETER : if PDM device is invalid
             ARM_DRIVER_OK              : if PDM successfully initialized or already initialized
 */
static int32_t PDM_Initialize(ARM_PDM_SignalEvent_t cb_event,PDM_resources_t *pdm)
{

    if(!cb_event)
        return ARM_DRIVER_ERROR_PARAMETER;

    pdm_expmast0_clk(pdm);

    /* Set the FIFO clear bit */
    pdm->reg_base->AUDIO_CNTRL_0 |= PDM_FIFO_CLEAR;

    pdm_synchronize_clk(pdm);

    /* Clear the FIFO clear bit */
    pdm->reg_base->AUDIO_CNTRL_0 &= ~(PDM_FIFO_CLEAR);

    /* User call back Event */
    pdm->cb_event = cb_event;

    /* Setting the flag */
    pdm->flags |= PDM_FLAG_DRV_INIT_DONE;

    return ARM_DRIVER_OK;
}

/**
@fn          int32_t PDM_Uninitialize (PDM_resources_t *PDM)
@brief       UnInitialize the PDM interface
@param[in]   PDM : Pointer to PDM resources
 @return     ARM_DRIVER_ERROR_PARAMETER : if PDM device is invalid
             ARM_DRIVER_OK              : if PDM successfully initialized or already initialized
 */
static int32_t PDM_Uninitialize(PDM_resources_t *pdm)
{
    if(!pdm)
        return ARM_DRIVER_ERROR_PARAMETER;

    if(!(pdm->flags & PDM_FLAG_DRV_INIT_DONE))
        return ARM_DRIVER_OK;

    /* Clear the fifo clear bit */
    pdm->reg_base->AUDIO_CNTRL_0 &= ~(PDM_FIFO_CLEAR);

    /* set call back to NULL */
    pdm->cb_event = NULL;

    /* Reset the flag */
    pdm->flags = 0U;

    return ARM_DRIVER_OK;
}

/**
 @fn           int32_t PDM_PowerControl (ARM_POWER_STATE state,
                                          PDM_resources_t *pdm)
 @brief        CMSIS-DRIVER PDM power control
 @param[in]    state : Power state
 @param[in]    pdm   : Pointer to PDM resources
 @return       ARM_DRIVER_ERROR_PARAMETER  : if PDM device is invalid
               ARM_DRIVER_OK               : if PDM successfully uninitialized or already not initialized
 */
static int32_t PDM_PowerControl(ARM_POWER_STATE status,
                                PDM_resources_t *pdm)
{
    switch(status)
    {
    case ARM_POWER_OFF:

        NVIC_ClearPendingIRQ(pdm->warning_irq);
        NVIC_ClearPendingIRQ(pdm->error_irq);
        NVIC_ClearPendingIRQ(pdm->audio_detect_irq);

        NVIC_DisableIRQ(pdm->warning_irq);
        NVIC_DisableIRQ(pdm->error_irq);
        NVIC_DisableIRQ(pdm->audio_detect_irq);

        /* Reset the power status of PDM */
        pdm->flags &= ~(PDM_FLAG_DRV_POWER_DONE);

    break;

    case ARM_POWER_FULL:

        if(!(pdm->flags & PDM_FLAG_DRV_INIT_DONE))
            return ARM_DRIVER_ERROR;

        if((pdm->flags & PDM_FLAG_DRV_POWER_DONE))
            return ARM_DRIVER_OK;

        /* Clear Any Pending IRQ */
        NVIC_ClearPendingIRQ(pdm->warning_irq);
        NVIC_ClearPendingIRQ(pdm->error_irq);
        NVIC_ClearPendingIRQ(pdm->audio_detect_irq);

        /* Set priority */
        NVIC_SetPriority (pdm->warning_irq, pdm->warning_irq_priority);
        NVIC_SetPriority(pdm->error_irq,pdm->error_irq_priority);
        NVIC_SetPriority(pdm->audio_detect_irq,pdm->audio_irq_priority);

        /* Enable the NIVC */
        NVIC_EnableIRQ(pdm->warning_irq);
        NVIC_EnableIRQ(pdm->error_irq);
        NVIC_EnableIRQ(pdm->audio_detect_irq);

        /* Set the power flag enabled */
        pdm->flags |= PDM_FLAG_DRV_POWER_DONE;

    break;

    case ARM_POWER_LOW:

    default:
        return ARM_DRIVER_ERROR_UNSUPPORTED;
    }
    return ARM_DRIVER_OK;
}

/**
 @fn           int32_t PDM_Control (uint32_t control,
                                    uint32_t arg,
                                    PDM_resources_t *pdm)
 @brief        CMSIS-Driver PDM control.
               Control PDM Interface.
 @param[in]    control : Operation \ref Driver_PDM.h : PDM control codes
 @param[in]    arg     : Argument of operation (optional)
 @param[in]    pdm     : Pointer to PDM resources
 @return       ARM_DRIVER_ERROR_PARAMETER  : if PDM device is invalid
               ARM_DRIVER_OK               : if PDM successfully uninitialized or already not initialized
 */
static int32_t PDM_Control (uint32_t control,
                            uint32_t arg,
                            PDM_resources_t *pdm)
{
    switch (control)
    {
    case ARM_PDM_MODE:

        /* Clear the PDM modes */
        pdm->reg_base->AUDIO_CNTRL_0 &= ~(PDM_MODES);

        pdm_synchronize_clk(pdm);

        /* Select the PDM modes */
        pdm->reg_base->AUDIO_CNTRL_0 |= (arg << PDM_CLK_MODE);

        break;

    case  ARM_PDM_BYPASS_IIR_FILTER:

        /* To select the Bypass IIR filter */
        pdm_bypass_iir(arg,pdm);

        break;

    case ARM_PDM_BYPASS_FIR_FILTER:

        /* To select the Bypass FIR filter */
        pdm_bypass_fir(arg,pdm);

        break;

    case ARM_PDM_PEAK_DETECTION_NODE:

        /* To select the peak detect node */
        pdm_peak_detect(arg,pdm);

        break;

    case ARM_PDM_SAMPLE_ADVANCE:

        /* To select the sample advance */
        pdm_sample_advance(arg,pdm);

        break;

    case ARM_PDM_DMA_HANDSHAKE:

        /* To use DMA handshake */
        pdm_dma_handshake(arg,pdm);
    }

    return ARM_DRIVER_OK;
}

/**
 @fn          int32_t PDM_Channel_Config (PDM_CH_CONFIG *cnfg,PDM_resources_t *PDM)
 @brief       PDM channel configurations
 @param[in]   pdm : Pointer to PDM resources
 @param[in]   cngf : Pointer to PDM_CH_CONFIG
 @return      ARM_DRIVER_OK : if function return successfully
 */
static int32_t PDM_Channel_Config(PDM_CH_CONFIG *cnfg, PDM_resources_t *pdm)
{
    uint32_t i;

    if (!(pdm->flags & PDM_FLAG_DRV_INIT_DONE))
        return ARM_DRIVER_ERROR;

    if (!(pdm->flags & PDM_FLAG_DRV_POWER_DONE))
        return ARM_DRIVER_ERROR;

    for(i = 0; i< PDM_MAX_FIR_COEFFICIENT; i++)
    {
        /* Store FIR coefficients */
        pdm->reg_base->CH_CNFG[cnfg->ch_num].CH_FIR_COEF[i] = cnfg->ch_fir_coef[i];
    }

    /* Store the IIR filter coefficient */
    pdm->reg_base->CH_CNFG[cnfg->ch_num].IIR_COEF_SEL = cnfg->ch_iir_coef;

    /* Store the  Phase Control delay */
    pdm->reg_base->CH_CNFG[cnfg->ch_num].PHASE_CONTROL = cnfg->ch_phase;

    /* Store the Gain value */
    pdm->reg_base->CH_CNFG[cnfg->ch_num].GAIN = cnfg->ch_gain;

    /* Store the Peak Detector Threshold */
    pdm->reg_base->CH_CNFG[cnfg->ch_num].PKDET_TH = cnfg->ch_peak_detect_th;

    /* Store the Peak Detector Interval */
    pdm->reg_base->CH_CNFG[cnfg->ch_num].PKDET_ITV = cnfg->ch_peak_detect_itv;

    return ARM_DRIVER_OK;
}

/**
@fn         int32_t PDM_Capture (PDM_Capture_CONFIG *cap_cnfg,PDM_resources_t *pdm)
@brief      -> clear and set the fifo clear bit
            -> Store the capture configuration channel address
            -> Store the user enabled channel
            -> Store the fifo watermark value
            -> Enable the PDM IRQ
@param[in]  cap_cnfg : Pointer to PDM_Capture_CONFIG
@param[in]  pdm      : Pointer to PDM resources
@return     ARM_DRIVER_OK : if function return successfully
*/
static int32_t PDM_Capture (PDM_Capture_CONFIG *cap_cnfg,PDM_resources_t *pdm)
{

    if (!(pdm->flags & PDM_FLAG_DRV_INIT_DONE))
        return ARM_DRIVER_ERROR;

    if (!(pdm->flags & PDM_FLAG_DRV_POWER_DONE))
        return ARM_DRIVER_ERROR;

    /* Enable the fifo clear bit */
    pdm->reg_base->AUDIO_CNTRL_0 |= (PDM_FIFO_CLEAR);

    pdm_synchronize_clk(pdm);

    /* clear the fifo clear bit */
    pdm->reg_base->AUDIO_CNTRL_0 &= ~(PDM_FIFO_CLEAR);

    pdm_synchronize_clk(pdm);

    /* clear the PDM channel bits */
    pdm->reg_base->AUDIO_CNTRL_0 &= ~(PDM_CHANNEL_ENABLE);

    /* Set the fifo watermark value */
    (pdm->reg_base->FIFO_WATERMARK) |= cap_cnfg->fifo_watermark;

    pdm->info.total_cnt  = cap_cnfg->total_no_samples;
    pdm->info.ch0_1_addr = cap_cnfg->ch0_1_addr;
    pdm->info.ch2_3_addr = cap_cnfg->ch2_3_addr;
    pdm->info.ch4_5_addr = cap_cnfg->ch4_5_addr;
    pdm->info.ch6_7_addr = cap_cnfg->ch6_7_addr;
    pdm->info.curr_cnt = 0;

    /* Store the user enabled channel */
    (pdm->reg_base->AUDIO_CNTRL_0) |= cap_cnfg->en_multiple_ch;

    /* Enable irq */
    pdm_enable_irq(pdm);

    return ARM_DRIVER_OK;
}

/* RTE_PDM */
#if RTE_PDM

static PDM_resources_t PDM0 = {
    .cb_event              = NULL,
    .reg_base              = (LPPDM_Type*) PDM_BASE,
    .info                  = {0},
    .flags                 = 0,
    .error_irq             = (IRQn_Type)PDM_ERROR_IRQ_IRQn,
    .warning_irq           = (IRQn_Type)PDM_WARN_IRQ_IRQn,
    .audio_detect_irq      = (IRQn_Type)PDM_AUDIO_DET_IRQ_IRQn,
    .error_irq_priority    = (uint32_t)RTE_PDM_IRQ_PRIORITY,
    .warning_irq_priority  = (uint32_t)RTE_PDM_IRQ_PRIORITY,
    .audio_irq_priority    = (uint32_t)RTE_PDM_IRQ_PRIORITY
};

/* Function Name: PDM_Initialize */
static int32_t PDM0_Initialize(ARM_PDM_SignalEvent_t cb_event)
{
    return (PDM_Initialize(cb_event,&PDM0));
}

/* Function Name: PDM_Uninitialize */
static int32_t PDM0_Uninitialize(void)
{
    return (PDM_Uninitialize(&PDM0));
}

/* Function Name: PDM_PowerControl */
static int32_t PDM0_PowerControl(ARM_POWER_STATE status)
{
    return (PDM_PowerControl(status,&PDM0));
}

/* Function Name: PDM_Control */
static int32_t PDM0_Control(uint32_t control, uint32_t arg)
{
    return (PDM_Control(control,arg,&PDM0));
}

/* Function Name: PDM_Start */
static int32_t PDM0_Capture(PDM_Capture_CONFIG *cap_cnfg)
{
    return (PDM_Capture(cap_cnfg,&PDM0));
}

/* Function Name: PDM0_Channel_Config */
static int32_t PDM0_Channel_Config(PDM_CH_CONFIG *cnfg)
{
    return (PDM_Channel_Config(cnfg,&PDM0));
}

/*Function Name : PDM_WARNNING_IRQHANDLER */
void PDM_WARN_IRQHandler (void)
{
    PDM_WARNING_IRQ_handler(&PDM0);
}

/*Function Name : PDM_ERROR_IRQHandler */
void PDM_ERROR_IRQHandler (void)
{
    PDM_ERROR_IRQ_handler(&PDM0);
}

/*Function Name : PDM_AUDIO_DETECTED_IRQHANDLER */
void PDM_AUDIO_DET_IRQHandler (void)
{
    PDM_AUDIO_DETECT_IRQ_handler(&PDM0);
}

extern ARM_DRIVER_PDM Driver_PDM;
ARM_DRIVER_PDM Driver_PDM = {
    PDM_GetVersion,
    PDM_GetCapabilities,
    PDM0_Initialize,
    PDM0_Uninitialize,
    PDM0_PowerControl,
    PDM0_Control,
    PDM0_Channel_Config,
    PDM0_Capture
};
#endif /* RTE_PDM */
