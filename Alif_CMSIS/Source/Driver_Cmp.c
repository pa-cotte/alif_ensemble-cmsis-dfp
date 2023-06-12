/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/* Project Includes */
#include "cmp.h"
#include "Driver_Comparator.h"
#include "Driver_Cmp_Private.h"
#include "analog_config.h"
#include "sys_ctrl_cmp.h"

#if !(RTE_HSCMP0 || RTE_HSCMP1 || RTE_HSCMP2 || RTE_HSCMP3)
#error "Comparator is not configured in RTE_device.h!"
#endif

#if (defined(RTE_Drivers_CMP0) && !RTE_HSCMP0)
#error "HSCMP0 not configured in RTE_Device.h!"
#endif

#if (defined(RTE_Drivers_CMP1) && !RTE_HSCMP1)
#error "CMP1 not configured in RTE_Device.h!"
#endif

#if (defined(RTE_Drivers_CMP2) && !RTE_HSCMP2)
#error "HSCMP2 not configured in RTE_Device.h!"
#endif

#if (defined(RTE_Drivers_CMP3) && !RTE_HSCMP3)
#error "HSCMP3 not configured in RTE_Device.h!"
#endif

#define ARM_CMP_DRV_VERSION    ARM_DRIVER_VERSION_MAJOR_MINOR(1, 0)  /*  Driver version */

/*Driver version*/
static const ARM_DRIVER_VERSION DriverVersion = {
        ARM_CMP_API_VERSION,
        ARM_CMP_DRV_VERSION
};

/*Driver Capabilities   */
static const ARM_COMPARATOR_CAPABILITIES DriverCapabilities = {
    1,/* Ability to invert the input signal */
    1,/* Used to define when to look at the comparator input */
    1,/* Supports Filter function */
    1,/* Supports Prescaler function */
    0 /* Reserved ( must be ZERO) */
};

/**
 @fn           ARM_DRIVER_VERSION CMP_GetVersion(void)
 @brief        get CMP version
 @param        none
 @return       driver version
 */
static ARM_DRIVER_VERSION CMP_GetVersion(void)
{
    return DriverVersion;
}

/**
 @fn           ARM_COMPARATOR_CAPABILITIES CMP_GetCapabilities(void)
 @brief        get Comparator Capabilities
 @param        none
 @return       driver Capabilities
 */
static ARM_COMPARATOR_CAPABILITIES CMP_GetCapabilities(void)
{
    return DriverCapabilities;
}

/**
 @fn           void AnalogConfig(void)
 @brief        Analog configuration register includes Vbat and comparator
 @param[in]    none
 @return       none
 */
static void AnalogConfig(void)
{
    /* Analog configuration Vbat register2 */
    analog_config_vbat_reg2();

    /* Analog configuration comparator register2 */
    analog_config_comp_reg2();
}

/**
 * @fn         CMP_Initialize(ARM_Comparator_SignalEvent_t cb_event, CMP_RESOURCES *CMP )
 * @brief      Initialize the Analog Comparator
 * @param[in]  cb_event : Pointer to /ref ARM_Comparator_SignalEvent_t cb_event
 * @param[in]  CMP      : Pointer to Comparator resources
 * @return     ARM_DRIVER_OK : if driver initialized successfully
 */
static int32_t CMP_Initialize(ARM_Comparator_SignalEvent_t cb_event, CMP_RESOURCES *CMP )
{
    int32_t ret = ARM_DRIVER_OK;

    if(!cb_event)
        return ARM_DRIVER_ERROR_PARAMETER;

    /* User call back Event */
    CMP->cb_event = cb_event;

    /* Set state to initialize */
    CMP->state.initialized = 1;

    return ret;
}

/**
 @fn           int32_t CMP_Uninitialize(CMP_RESOURCES *CMP)
 @brief        Un-Initialize the Analog Comparator
 @param[in]    CMP  : Pointer to Comparator resources
 @return       ARM_DRIVER_OK : if Comparator successfully uninitialized or already not initialized
 */
static int32_t CMP_Uninitialize(CMP_RESOURCES *CMP)
{
    int32_t ret = ARM_DRIVER_OK;

    if(!CMP)
        return ARM_DRIVER_ERROR_PARAMETER;

    /* Check initialized has done or not */
    if(CMP->state.initialized == 0)
        return ARM_DRIVER_OK;

    /* set call back to NULL */
    CMP->cb_event = NULL;

    /* Reset the state */
    CMP->state.initialized = 0;

    return ret;
}

/**
 * @fn          CMP_PowerControl(ARM_POWER_STATE state,CMP_RESOURCES *CMP )
 * @brief       power the driver and enable NVIC
                Initialize the following in ARM_POWER_FULL:
                 - Access vbat and CMP register configuration
                 - Select the positive and negative terminal of CMP
                 - Select the instances for the output CMP
                 - Mask the interrupt
               Un-Initialize the following in ARM_POWER_OFF:
                 -clear the CMP configuration
                 -set call back to NULL
                 -Disable the interrupt mask
                 -Reset the state
 * @param[in]   state : power state
 * @param[in]   CMP   : pointer to /ref CMP_RESOURCES
 * @return      ARM_DRIVER_OK    : if power done successful
                ARM_DRIVER_ERROR : if initialize is not done
 */
static int32_t CMP_PowerControl(ARM_POWER_STATE state,CMP_RESOURCES *CMP )
{
    int32_t ret = ARM_DRIVER_OK;

    switch (state)
    {
    case ARM_POWER_FULL:
        if(CMP->state.initialized == 0)
            return ARM_DRIVER_ERROR;

        if(CMP->state.powered == 1)
            return ARM_DRIVER_OK;

        /* Clear Any Pending IRQ */
        NVIC_ClearPendingIRQ(CMP->irq_num);

        /* Set the priority */
        NVIC_SetPriority(CMP->irq_num, CMP->irq_priority);

        /* Enable the NIVC */
        NVIC_EnableIRQ(CMP->irq_num);

        CmpCoreClkControl(CMP, true);

        /*function include vbat and CMP address and it value */
        AnalogConfig();

        CMP->regs->CMP_POLARITY_CTRL ^= 1;

        /* Initialize the CMP configurations */
        CmpSetConfig(CMP->config);

        /* To select the output of the CMP module */
        CmpOutputSel(CMP);

        /* To disable the interrupt */
        cmp_disable_interrupt(CMP->regs);

        /* Set the power state enabled */
        CMP->state.powered = 1;

        break;

    case ARM_POWER_OFF:

        /* Disable ADC NVIC */
        NVIC_DisableIRQ(CMP->irq_num);

        /* Clear Any Pending IRQ */
        NVIC_ClearPendingIRQ(CMP->irq_num);

        /* clear the CMP configuration */
        cmp_clear_config(CMP->regs);

        /* To the enable the interrupt */
        cmp_disable_interrupt(CMP->regs);

        CmpCoreClkControl(CMP, false);

        /* Reset the power status of ADC */
        CMP->state.powered = 0;

        break;

    case ARM_POWER_LOW:
    default:
        return ARM_DRIVER_ERROR_UNSUPPORTED;
    }

    return ret;
}

/**
 * @brief        CMP_Control(CMP_RESOURCES *CMP, uint32_t control, uint32_t arg)
 * @brief        CMSIS-Driver comparator control.
                 Control comparator Interface.
 * @param[in]    CMP     : Pointer to comparator resources
 * @param[in]    control : Operation \ref Driver_Comparator.h : comparator control codes
 * @param[in]    arg     : Argument of operation (optional)
 * @return       ARM_DRIVER_ERROR_PARAMETER  : if comparator device is invalid
                 ARM_DRIVER_OK               : if comparator successfully uninitialized or already not initialized
 */
static int32_t CMP_Control(CMP_RESOURCES *CMP, uint32_t control, uint32_t arg)
{
    int32_t ret = ARM_DRIVER_OK;

    if (CMP->state.initialized == 0)
        return ARM_DRIVER_ERROR;

    if (CMP->state.powered == 0)
        return ARM_DRIVER_ERROR;

    switch (control)
    {
    case ARM_CMP_POLARITY_CONTROL:

        if(arg > 2)
            return ARM_DRIVER_ERROR_PARAMETER;

        /* If active, invert the value of comparator input before processing  */
        cmp_set_polarity_ctrl(CMP->regs, arg);

        break;

    case ARM_CMP_WINDOW_CONTROL:

        if(arg > 3)
            return ARM_DRIVER_ERROR_PARAMETER;

        /* enable the windowing function */
        cmp_set_window_ctrl(CMP->regs, arg);

        break;

    case ARM_CMP_FILTER_CONTROL:

        if(arg < 2 || arg > 8 )
            return ARM_DRIVER_ERROR_PARAMETER;

        /* To enable the filter function and adding filter values to the filter control register */
        cmp_set_filter_ctrl(CMP->regs, arg);

        break;

    case ARM_CMP_PRESCALER_CONTROL:

        if(arg > CMP_PRESCALER_MAX_VALUE)
            return ARM_DRIVER_ERROR_PARAMETER;

        /* Comparator input will be sampled at the given clocks */
        cmp_set_prescaler_ctrl(CMP->regs, arg);

        break;

    default:
        ret = ARM_DRIVER_ERROR_UNSUPPORTED;
    }
    return ret;
}

/**
 * @fn         CMP_Start (CMP_RESOURCES *CMP)
 * @brief      CMSIS-Driver Comparator Start
 *             Clear the IRQ before re-starting
 *             Enable the Comparator.
 * @param[in]  CMP  : Pointer to Comparator resources
 * @return     ARM_DRIVER_OK  : if the function are return successful
 *             ARM_DRIVER_ERROR : if initialize is not done
 */
static int32_t CMP_Start (CMP_RESOURCES *CMP)
{
    int32_t ret = ARM_DRIVER_OK;

    if (CMP->state.initialized == 0)
        return ARM_DRIVER_ERROR;

    if (CMP->state.powered == 0)
        return ARM_DRIVER_ERROR;

    /* enable the interrupt(unmask the interrupt 0x0)*/
    cmp_enable_interrupt(CMP->regs);

    /* clear the interrupt before re-starting */
    while(cmp_get_interrupt_status(CMP->regs))
        cmp_clear_interrupt(CMP->regs);

    /* Enable the Comparator module */
    CmpEnable(CMP, ENABLE);

    return ret;
}

/**
 * @fn         CMP_Stop (CMP_RESOURCES *CMP)
 * @brief      CMSIS-Driver Comparator Stop
 *               Disable the Comparator.
 * @param[in]  CMP  : Pointer to Comparator resources
 * @return     ARM_DRIVER_OK  : if the function are return successful
 *             ARM_DRIVER_ERROR : if initialize is not done
 */
static int32_t CMP_Stop (CMP_RESOURCES *CMP)
{
    int32_t ret = ARM_DRIVER_OK;
    if (CMP->state.initialized == 0)
        return ARM_DRIVER_ERROR;

    if (CMP->state.powered == 0)
        return ARM_DRIVER_ERROR;

    /* Disable the Comparator module */
    CmpEnable(CMP, DISABLE);

    /* Disable the interrupt */
    cmp_disable_interrupt(CMP->regs);

    return ret;
}

/**
  \fn          void CMP_IRQ_handler(CMP_RESOURCES *CMP)
  \brief       CMP instance specific part of CMP Interrupt handler.
  \param[in]   CMP : Pointer to Comparator resources
 */
static void CMP_IRQ_handler(CMP_RESOURCES *CMP)
{
    cmp_irq_handler(CMP->regs);

    /* call user callback */
    CMP->cb_event(ARM_CMP_FILTER_EVENT_OCCURRED);
}

/* HSCMP0 driver instance */
#if (RTE_HSCMP0)

/* Comparator Configurations */
static CMP_RESOURCES HSCMP0 = {
    .cb_event           = NULL,
    .regs               = (CMP0_Type *)CMP0_BASE,
    .drv_instance       = CMP_INSTANCE_0,
    .state              = 0,
    .irq_num            = (IRQn_Type)CMP0_IRQ_IRQn,
    .config             = (RTE_CMP0_SEL_POSITIVE << 0 )     |
                          (RTE_CMP0_SEL_NEGATIVE << 2)      |
                          (RTE_CMP0_SEL_HYSTERISIS << 4 ),
    .irq_priority       = RTE_CMP0_IRQ_PRIORITY,
    .output_mux_sel     = RTE_CMP0_OUTPUT_MUX_SEL,
    .input_enable       = CMP0_ENABLE
};

/* Function Name: CMP0_Initialize */
static int32_t CMP0_Initialize(ARM_Comparator_SignalEvent_t cb_event)
{
    return (CMP_Initialize(cb_event,&HSCMP0));
}

/* Function Name: CMP0_Uninitialize */
static int32_t CMP0_Uninitialize(void)
{
    return (CMP_Uninitialize(&HSCMP0));
}

/* Function Name: CMP0_PowerControl */
static int32_t CMP0_PowerControl(ARM_POWER_STATE state)
{
    return (CMP_PowerControl(state, &HSCMP0));
}

/* Function Name: CMP0_Control */
static int32_t CMP0_Control(uint32_t control, uint32_t arg)
{
    return (CMP_Control(&HSCMP0, control, arg));
}

/* Function Name: CMP0_Start */
static int32_t CMP0_Start (void)
{
    return (CMP_Start(&HSCMP0));
}

/* Function name :CMP0_Stop */
static int32_t CMP0_Stop (void)
{
    return (CMP_Stop(&HSCMP0));
}

/*Function Name : CMP0_IRQHandler */
void CMP0_IRQHandler (void)
{
    return CMP_IRQ_handler(&HSCMP0);
}

extern ARM_DRIVER_CMP Driver_CMP0;
ARM_DRIVER_CMP Driver_CMP0 =
{
    CMP_GetVersion,
    CMP_GetCapabilities,
    CMP0_Initialize,
    CMP0_Uninitialize,
    CMP0_PowerControl,
    CMP0_Control,
    CMP0_Start,
    CMP0_Stop
};

#endif

/* HSCMP1 driver instance */
#if (RTE_HSCMP1)

/* Comparator Configurations */
static CMP_RESOURCES HSCMP1 = {
    .cb_event           = NULL,
    .regs               = (CMP0_Type *)CMP1_BASE,
    .drv_instance       = CMP_INSTANCE_1,
    .state              = 0,
    .irq_num            = (IRQn_Type)CMP1_IRQ_IRQn,
    .config             = (RTE_CMP1_SEL_POSITIVE << 7)      |
                          (RTE_CMP1_SEL_NEGATIVE << 9)      |
                          (RTE_CMP1_SEL_HYSTERISIS << 11),
    .irq_priority       = RTE_CMP1_IRQ_PRIORITY,
    .output_mux_sel     = RTE_CMP1_OUTPUT_MUX_SEL,
    .input_enable       = CMP1_ENABLE
};

/* Function Name: CMP1_Initialize */
static int32_t CMP1_Initialize(ARM_Comparator_SignalEvent_t cb_event)
{
    return (CMP_Initialize(cb_event,&HSCMP1));
}

/* Function Name: CMP1_Uninitialize */
static int32_t CMP1_Uninitialize(void)
{
    return (CMP_Uninitialize(&HSCMP1));
}

/* Function Name: CMP1_PowerControl */
static int32_t CMP1_PowerControl(ARM_POWER_STATE state)
{
    return (CMP_PowerControl(state,&HSCMP1));
}

/* Function Name: CMP1_Control */
static int32_t CMP1_Control(uint32_t control, uint32_t arg)
{
    return (CMP_Control(&HSCMP1, control, arg));
}

/* Function Name: CMP1_Start */
static int32_t CMP1_Start (void)
{
    return (CMP_Start(&HSCMP1));
}

/* Function name :CMP1_Stop */
static int32_t CMP1_Stop (void)
{
    return (CMP_Stop(&HSCMP1));
}

/*Function Name : CMP1_IRQHandler */
void CMP1_IRQHandler (void)
{
    return CMP_IRQ_handler(&HSCMP1);
}

extern ARM_DRIVER_CMP Driver_CMP1;
ARM_DRIVER_CMP Driver_CMP1 =
{
    CMP_GetVersion,
    CMP_GetCapabilities,
    CMP1_Initialize,
    CMP1_Uninitialize,
    CMP1_PowerControl,
    CMP1_Control,
    CMP1_Start,
    CMP1_Stop
};

#endif

/* HSCMP2 driver instance */
#if (RTE_HSCMP2)

/* Comparator Configurations */
static CMP_RESOURCES HSCMP2 = {
    .cb_event           = NULL,
    .regs               = (CMP0_Type *)CMP2_BASE,
    .drv_instance       = CMP_INSTANCE_2,
    .state              = 0,
    .irq_num            = (IRQn_Type)CMP2_IRQ_IRQn,
    .config             = (RTE_CMP2_SEL_POSITIVE << 14)      |
                          (RTE_CMP2_SEL_NEGATIVE << 16)      |
                          (RTE_CMP2_SEL_HYSTERISIS << 18),
    .irq_priority       = RTE_CMP2_IRQ_PRIORITY,
    .output_mux_sel     = RTE_CMP2_OUTPUT_MUX_SEL,
    .input_enable       = CMP2_ENABLE
};

/* Function Name: CMP2_Initialize */
static int32_t CMP2_Initialize(ARM_Comparator_SignalEvent_t cb_event)
{
    return (CMP_Initialize(cb_event,&HSCMP2));
}

/* Function Name: CMP2_Uninitialize */
static int32_t CMP2_Uninitialize(void)
{
    return (CMP_Uninitialize(&HSCMP2));
}

/* Function Name: CMP2_PowerControl */
static int32_t CMP2_PowerControl(ARM_POWER_STATE state)
{
    return (CMP_PowerControl(state,&HSCMP2));
}

/* Function Name: CMP2_Control */
static int32_t CMP2_Control(uint32_t control, uint32_t arg)
{
    return (CMP_Control(&HSCMP2, control, arg));
}

/* Function Name: CMP2_Start */
static int32_t CMP2_Start (void)
{
    return (CMP_Start(&HSCMP2));
}

/* Function name :CMP2_Stop */
static int32_t CMP2_Stop (void)
{
    return (CMP_Stop(&HSCMP2));
}

/*Function Name : CMP2_IRQHandler */
void CMP2_IRQHandler (void)
{
    return CMP_IRQ_handler(&HSCMP2);
}

extern ARM_DRIVER_CMP Driver_CMP2;
ARM_DRIVER_CMP Driver_CMP2 =
{
    CMP_GetVersion,
    CMP_GetCapabilities,
    CMP2_Initialize,
    CMP2_Uninitialize,
    CMP2_PowerControl,
    CMP2_Control,
    CMP2_Start,
    CMP2_Stop
};

#endif

/* HSCMP3 driver instance */
#if (RTE_HSCMP3)

/* Comparator Configurations */
static CMP_RESOURCES HSCMP3 = {
    .cb_event           = NULL,
    .regs               = (CMP0_Type *)CMP3_BASE,
    .drv_instance       = CMP_INSTANCE_3,
    .state              = 0,
    .irq_num            = (IRQn_Type)CMP3_IRQ_IRQn,
    .config             = (RTE_CMP3_SEL_POSITIVE << 21)      |
                          (RTE_CMP3_SEL_NEGATIVE << 23)      |
                          (RTE_CMP3_SEL_HYSTERISIS << 25),
    .irq_priority       = RTE_CMP3_IRQ_PRIORITY,
    .output_mux_sel     = RTE_CMP3_OUTPUT_MUX_SEL,
    .input_enable       = CMP3_ENABLE
};

/* Function Name: CMP3_Initialize */
static int32_t CMP3_Initialize(ARM_Comparator_SignalEvent_t cb_event)
{
    return (CMP_Initialize(cb_event,&HSCMP3));
}

/* Function Name: CMP3_Uninitialize */
static int32_t CMP3_Uninitialize(void)
{
    return (CMP_Uninitialize(&HSCMP3));
}

/* Function Name: CMP3_PowerControl */
static int32_t CMP3_PowerControl(ARM_POWER_STATE state)
{
    return (CMP_PowerControl(state,&HSCMP3));
}

/* Function Name: CMP3_Control */
static int32_t CMP3_Control(uint32_t control, uint32_t arg)
{
    return (CMP_Control(&HSCMP3, control, arg));
}

/* Function Name: CMP3_Start */
static int32_t CMP3_Start (void)
{
    return (CMP_Start(&HSCMP3));
}

/* Function name :CMP3_Stop */
static int32_t CMP3_Stop (void)
{
    return (CMP_Stop(&HSCMP3));
}

/*Function Name : CMP3_IRQHandler */
void CMP3_IRQHandler (void)
{
    return CMP_IRQ_handler(&HSCMP3);
}

extern ARM_DRIVER_CMP Driver_CMP3;
ARM_DRIVER_CMP Driver_CMP3 =
{
    CMP_GetVersion,
    CMP_GetCapabilities,
    CMP3_Initialize,
    CMP3_Uninitialize,
    CMP3_PowerControl,
    CMP3_Control,
    CMP3_Start,
    CMP3_Stop
};

#endif
