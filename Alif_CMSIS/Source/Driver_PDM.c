/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/******************************************************************************
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
#include "Driver_PDM_Private.h"
#include "sys_ctrl_pdm.h"

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
 @fn          void PDM_ERROR_IRQ_handler(PDM_RESOURCES *PDM)
 @brief       IRQ handler for the error interrupt
 @param[in]   PDM : Pointer to PDM resources
 @return      none
 */
void PDM_ERROR_IRQ_handler(PDM_RESOURCES *PDM)
{
    pdm_error_detect_irq_handler(PDM->regs);

    /* call user callback */
    PDM->cb_event(ARM_PDM_EVENT_ERROR);
}

/**
 @fn          void PDM_AUDIO_DETECT_IRQ_handler(PDM_RESOURCES *PDM)
 @brief       IRQ handler for the audio detect interrupt
 @param[in]   PDM : Pointer to PDM resources
 @return      none
 */
void PDM_AUDIO_DETECT_IRQ_handler(PDM_RESOURCES *PDM)
{
    pdm_transfer_t *transfer = &PDM->transfer;

    pdm_audio_detect_irq_handler(PDM->regs, transfer);

    if(transfer->status == PDM_AUDIO_STATUS_DETECTION)
    {
        transfer->status  = PDM_CAPTURE_STATUS_NONE;

        /* call user callback */
        PDM->cb_event(ARM_PDM_EVENT_AUDIO_DETECTION);
    }
}

/**
 @fn          void PDM_WARNING_IRQ_handler(PDM_RESOURCES *PDM)
 @brief       IRQ handler for the PDM warning interrupt
 @param[in]   PDM : Pointer to PDM resources
 @return      none
 */
void PDM_WARNING_IRQ_handler(PDM_RESOURCES *PDM)
{
    pdm_transfer_t *transfer = &(PDM->transfer);

    pdm_warning_irq_handler(PDM->regs, transfer);

    if(transfer->status ==  PDM_CAPTURE_STATUS_COMPLETE)
    {
        transfer->status  = PDM_CAPTURE_STATUS_NONE;

        /* call user callback */
        PDM->cb_event(ARM_PDM_EVENT_CAPTURE_COMPLETE);
    }
}

/**
@fn          int32_t PDMx_Initialize(ARM_PDM_SignalEvent_t cb_event, PDM_RESOURCES *PDM)
@brief       Initialize the PDM interface
@param[in]   PDM : Pointer to PDM resources
 @return     ARM_DRIVER_ERROR_PARAMETER : if PDM device is invalid
             ARM_DRIVER_OK              : if PDM successfully initialized or already initialized
 */
static int32_t PDMx_Initialize(ARM_PDM_SignalEvent_t cb_event, PDM_RESOURCES *PDM)
{
    if(!cb_event)
        return ARM_DRIVER_ERROR_PARAMETER;

    /* User call back Event */
    PDM->cb_event = cb_event;

    /* Setting the state */
    PDM->state.initialized = 1;

    return ARM_DRIVER_OK;
}

/**
@fn          int32_t PDMx_Uninitialize(PDM_RESOURCES *PDM)
@brief       UnInitialize the PDM interface
@param[in]   PDM : Pointer to PDM resources
 @return     ARM_DRIVER_ERROR_PARAMETER : if PDM device is invalid
             ARM_DRIVER_OK              : if PDM successfully initialized or already initialized
 */
static int32_t PDMx_Uninitialize(PDM_RESOURCES *PDM)
{
    if(!PDM)
        return ARM_DRIVER_ERROR_PARAMETER;

    if(PDM->state.initialized == 0)
        return ARM_DRIVER_OK;

    /* set call back to NULL */
    PDM->cb_event = NULL;

    /* Reset the state */
    PDM->state.initialized = 0U;

    return ARM_DRIVER_OK;
}

/**
 @fn           int32_t PDMx_PowerControl (ARM_POWER_STATE state,
                                          PDM_RESOURCES *PDM)
 @brief        CMSIS-DRIVER PDM power control
 @param[in]    state : Power state
 @param[in]    PDM   : Pointer to PDM resources
 @return       ARM_DRIVER_ERROR_PARAMETER  : if PDM device is invalid
               ARM_DRIVER_OK               : if PDM successfully uninitialized or already not initialized
 */
static int32_t PDMx_PowerControl(ARM_POWER_STATE status,
                                PDM_RESOURCES *PDM)
{
    switch(status)
    {
    case ARM_POWER_OFF:

        /* Clear the fifo clear bit */
        pdm_disable_fifo_clear(PDM->regs);

        if(PDM->instance == PDM_INSTANCE_LPPDM)
        {
            /* Clear Any Pending IRQ */
            NVIC_ClearPendingIRQ(PDM->warning_irq);

            /* Disable the NIVC */
            NVIC_DisableIRQ(PDM->warning_irq);

            /* Disable LPPDM clock */
            disable_lppdm_periph_clk();
        }
        else
        {
            /* Clear Any Pending IRQ */
            NVIC_ClearPendingIRQ(PDM->warning_irq);
            NVIC_ClearPendingIRQ(PDM->error_irq);
            NVIC_ClearPendingIRQ(PDM->audio_detect_irq);

            /* Disable the NIVC */
            NVIC_DisableIRQ(PDM->warning_irq);
            NVIC_DisableIRQ(PDM->error_irq);
            NVIC_DisableIRQ(PDM->audio_detect_irq);

            /* Disable PDM clock */
            disable_pdm_periph_clk();
        }

        /* Reset the power status of PDM */
        PDM->state.powered = 0;

    break;

    case ARM_POWER_FULL:

        if(PDM->state.initialized == 0)
            return ARM_DRIVER_ERROR;

        if(PDM->state.powered == 1)
            return ARM_DRIVER_OK;

        if(PDM->instance == PDM_INSTANCE_LPPDM)
        {
            /* Clear Any Pending IRQ */
            NVIC_ClearPendingIRQ(PDM->warning_irq);

            /* Set priority */
            NVIC_SetPriority (PDM->warning_irq, PDM->warning_irq_priority);

            /* Enable the NIVC */
            NVIC_EnableIRQ(PDM->warning_irq);

            /* Enable LPPDM clock */
            enable_lppdm_periph_clk();
        }
        else
        {
            /* Clear Any Pending IRQ */
            NVIC_ClearPendingIRQ(PDM->warning_irq);
            NVIC_ClearPendingIRQ(PDM->error_irq);
            NVIC_ClearPendingIRQ(PDM->audio_detect_irq);

            /* Set priority */
            NVIC_SetPriority (PDM->warning_irq, PDM->warning_irq_priority);
            NVIC_SetPriority(PDM->error_irq, PDM->error_irq_priority);
            NVIC_SetPriority(PDM->audio_detect_irq, PDM->audio_irq_priority);

            /* Enable the NIVC */
            NVIC_EnableIRQ(PDM->warning_irq);
            NVIC_EnableIRQ(PDM->error_irq);
            NVIC_EnableIRQ(PDM->audio_detect_irq);

            /* Enable PDM clock */
            enable_pdm_periph_clk();
        }

        /* Set the FIFO clear bit */
        pdm_enable_fifo_clear(PDM->regs);

        /* Set the power state enabled */
        PDM->state.powered = 1;

    break;

    case ARM_POWER_LOW:

    default:
        return ARM_DRIVER_ERROR_UNSUPPORTED;
    }
    return ARM_DRIVER_OK;
}

/**
 @fn           int32_t PDMx_Control (uint32_t control,
                                    uint32_t arg,
                                    PDM_RESOURCES *PDM)
 @brief        CMSIS-Driver PDM control.
               Control PDM Interface.
 @param[in]    control : Operation \ref Driver_PDM.h : PDM control codes
 @param[in]    arg     : Argument of operation (optional)
 @param[in]    PDM     : Pointer to PDM resources
 @return       ARM_DRIVER_ERROR_PARAMETER  : if PDM device is invalid
               ARM_DRIVER_OK               : if PDM successfully uninitialized or already not initialized
 */
static int32_t PDMx_Control (uint32_t control,
                            uint32_t arg,
                            PDM_RESOURCES *PDM)
{
    switch (control)
    {
    case ARM_PDM_MODE:

        /* Clear the PDM modes */
        pdm_clear_modes(PDM->regs);

        /* Select the PDM modes */
        pdm_enable_modes(PDM->regs, arg);

        break;

    case  ARM_PDM_BYPASS_IIR_FILTER:

        /* To select the Bypass IIR filter */
        pdm_bypass_iir(PDM->regs, arg);

        break;

    case ARM_PDM_BYPASS_FIR_FILTER:

        /* To select the Bypass FIR filter */
        pdm_bypass_fir(PDM->regs, arg);

        break;

    case ARM_PDM_PEAK_DETECTION_NODE:

        /* To select the peak detect node */
        pdm_peak_detect(PDM->regs, arg);

        break;

    case ARM_PDM_SAMPLE_ADVANCE:

        /* To select the sample advance */
        pdm_sample_advance(PDM->regs, arg);

        break;

    case ARM_PDM_DMA_HANDSHAKE:

        /* To use DMA handshake */
        pdm_dma_handshake(PDM->regs, arg);
    }

    return ARM_DRIVER_OK;
}

/**
 @fn          int32_t PDMx_Channel_Config(PDM_CH_CONFIG *cnfg, PDM_RESOURCES *PDM)
 @brief       PDM channel configurations
 @param[in]   PDM : Pointer to PDM resources
 @param[in]   cngf : Pointer to PDM_CH_CONFIG
 @return      ARM_DRIVER_OK : if function return successfully
 */
static int32_t PDMx_Channel_Config(PDM_CH_CONFIG *cnfg, PDM_RESOURCES *PDM)
{
    if (PDM->state.initialized == 0)
        return ARM_DRIVER_ERROR;

    if (PDM->state.powered == 0)
        return ARM_DRIVER_ERROR;

    /* Store the fir coefficient values */
    pdm_set_fir_coeff(PDM->regs, cnfg->ch_num, cnfg->ch_fir_coef);

    /* Store the iirr coefficient values */
    pdm_set_ch_iir_coef(PDM->regs, cnfg->ch_num, cnfg->ch_iir_coef );

    /* Store the channel phase control values */
    pdm_set_ch_phase(PDM->regs, cnfg->ch_num, cnfg->ch_phase );

    /* Store the Gain value */
    pdm_set_ch_gain(PDM->regs, cnfg->ch_num, cnfg->ch_gain );

    /* Store the Peak Detector Threshold */
    pdm_set_peak_detect_th(PDM->regs, cnfg->ch_num, cnfg->ch_peak_detect_th);

    /* Store the Peak Detector Interval */
    pdm_set_peak_detect_itv(PDM->regs, cnfg->ch_num, cnfg->ch_peak_detect_itv);

    return ARM_DRIVER_OK;
}

/**
@fn         int32_t PDMx_Capture (PDM_Capture_CONFIG *cap_cnfg, PDM_RESOURCES *PDM)
@brief      -> clear and set the fifo clear bit
            -> Store the capture configuration channel address
            -> Store the user enabled channel
            -> Store the fifo watermark value
            -> Enable the PDM IRQ
@param[in]  cap_cnfg : Pointer to PDM_Capture_CONFIG
@param[in]  PDM      : Pointer to PDM resources
@return     ARM_DRIVER_OK : if function return successfully
*/
static int32_t PDMx_Capture (PDM_Capture_CONFIG *cap_cnfg, PDM_RESOURCES *PDM)
{

    if (PDM->state.initialized == 0)
        return ARM_DRIVER_ERROR;

    if (PDM->state.powered == 0)
        return ARM_DRIVER_ERROR;

    /* clear the fifo clear bit */
    pdm_disable_fifo_clear(PDM->regs);

    /* clear the PDM channel bits */
    pdm_clear_channel(PDM->regs);

    /* Set the fifo watermark value */
    pdm_set_fifo_watermark(PDM->regs, cap_cnfg->fifo_watermark);

    PDM->transfer.total_cnt  = cap_cnfg->total_no_samples;
    PDM->transfer.ch0_1_addr = cap_cnfg->ch0_1_addr;
    PDM->transfer.ch2_3_addr = cap_cnfg->ch2_3_addr;
    PDM->transfer.ch4_5_addr = cap_cnfg->ch4_5_addr;
    PDM->transfer.ch6_7_addr = cap_cnfg->ch6_7_addr;
    PDM->transfer.curr_cnt = 0;

    /* Store the user enabled channel */
    pdm_enable_multi_ch(PDM->regs, cap_cnfg->en_multiple_ch);

    /* Enable irq */
    pdm_enable_irq(PDM->regs);

    return ARM_DRIVER_OK;
}

/* RTE_PDM */
#if RTE_PDM

static PDM_RESOURCES PDM = {
    .cb_event              = NULL,
    .regs                  = (PDM_Type *)PDM_BASE,
    .transfer              = {0},
    .state                 = {0},
    .instance              = PDM_INSTANCE_PDM0,
    .error_irq             = (IRQn_Type)PDM_ERROR_IRQ_IRQn,
    .warning_irq           = (IRQn_Type)PDM_WARN_IRQ_IRQn,
    .audio_detect_irq      = (IRQn_Type)PDM_AUDIO_DET_IRQ_IRQn,
    .error_irq_priority    = (uint32_t)RTE_PDM_IRQ_PRIORITY,
    .warning_irq_priority  = (uint32_t)RTE_PDM_IRQ_PRIORITY,
    .audio_irq_priority    = (uint32_t)RTE_PDM_IRQ_PRIORITY
};

/* Function Name: PDM_Initialize */
static int32_t PDM_Initialize(ARM_PDM_SignalEvent_t cb_event)
{
    return (PDMx_Initialize(cb_event, &PDM));
}

/* Function Name: PDM_Uninitialize */
static int32_t PDM_Uninitialize(void)
{
    return (PDMx_Uninitialize(&PDM));
}

/* Function Name: PDM_PowerControl */
static int32_t PDM_PowerControl(ARM_POWER_STATE status)
{
    return (PDMx_PowerControl(status, &PDM));
}

/* Function Name: PDM_Control */
static int32_t PDM_Control(uint32_t control, uint32_t arg)
{
    return (PDMx_Control(control, arg, &PDM));
}

/* Function Name: PDM_Capture */
static int32_t PDM_Capture(PDM_Capture_CONFIG *cap_cnfg)
{
    return (PDMx_Capture(cap_cnfg, &PDM));
}

/* Function Name: PDM_Channel_Config */
static int32_t PDM_Channel_Config(PDM_CH_CONFIG *cnfg)
{
    return (PDMx_Channel_Config(cnfg, &PDM));
}

/*Function Name : PDM_WARNNING_IRQHANDLER */
void PDM_WARN_IRQHandler (void)
{
    PDM_WARNING_IRQ_handler(&PDM);
}

/*Function Name : PDM_ERROR_IRQHandler */
void PDM_ERROR_IRQHandler (void)
{
    PDM_ERROR_IRQ_handler(&PDM);
}

/*Function Name : PDM_AUDIO_DET_IRQHandler */
void PDM_AUDIO_DET_IRQHandler (void)
{
    PDM_AUDIO_DETECT_IRQ_handler(&PDM);
}

extern ARM_DRIVER_PDM Driver_PDM;
ARM_DRIVER_PDM Driver_PDM = {
    PDM_GetVersion,
    PDM_GetCapabilities,
    PDM_Initialize,
    PDM_Uninitialize,
    PDM_PowerControl,
    PDM_Control,
    PDM_Channel_Config,
    PDM_Capture
};
#endif /* RTE_PDM */

/* RTE_LPPDM */
#if RTE_LPPDM

static PDM_RESOURCES LPPDM  = {
    .cb_event              = NULL,
    .regs                  = (PDM_Type *)LPPDM_BASE,
    .transfer              = {0},
    .state                 = {0},
    .instance              = PDM_INSTANCE_LPPDM,
    .warning_irq           = (IRQn_Type)LPPDM_IRQ_IRQn,
    .warning_irq_priority  = (uint32_t)RTE_LPPDM_IRQ_PRIORITY
};

/* Function Name: LPPDM_Initialize */
static int32_t LPPDM_Initialize(ARM_PDM_SignalEvent_t cb_event)
{
    return (PDMx_Initialize(cb_event, &LPPDM));
}

/* Function Name: LPPDM_Uninitialize */
static int32_t LPPDM_Uninitialize(void)
{
    return (PDMx_Uninitialize(&LPPDM));
}

/* Function Name: LPPDM_PowerControl */
static int32_t LPPDM_PowerControl(ARM_POWER_STATE status)
{
    return (PDMx_PowerControl(status, &LPPDM));
}

/* Function Name: LPPDM_Control */
static int32_t LPPDM_Control(uint32_t control, uint32_t arg)
{
    return (PDMx_Control(control, arg, &LPPDM));
}

/* Function Name: LPPDM_Capture */
static int32_t LPPDM_Capture(PDM_Capture_CONFIG *cap_cnfg)
{
    return (PDMx_Capture(cap_cnfg, &LPPDM));
}

/* Function Name: LPPDM_Channel_Config */
static int32_t LPPDM_Channel_Config(PDM_CH_CONFIG *cnfg)
{
    return (PDMx_Channel_Config(cnfg, &LPPDM));
}

/*Function Name : LPPDM_IRQHandler */
void LPPDM_IRQHandler (void)
{
    PDM_WARNING_IRQ_handler(&LPPDM);
}

extern ARM_DRIVER_PDM Driver_LPPDM;
ARM_DRIVER_PDM Driver_LPPDM = {
    PDM_GetVersion,
    PDM_GetCapabilities,
    LPPDM_Initialize,
    LPPDM_Uninitialize,
    LPPDM_PowerControl,
    LPPDM_Control,
    LPPDM_Channel_Config,
    LPPDM_Capture
};
#endif /* RTE_LPPDM */
