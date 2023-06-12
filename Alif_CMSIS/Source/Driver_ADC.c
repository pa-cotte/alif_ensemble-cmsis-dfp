/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/* Include */
#include "Driver_ADC_Private.h"
#include "analog_config.h"

#define ARM_ADC_DRV_VERISON ARM_DRIVER_VERSION_MAJOR_MINOR(1,0) /*DRIVER VERSION*/

/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion ={
    ARM_ADC_API_VERSION,
    ARM_ADC_DRV_VERISON
};

/* Driver Capabilities */
static const ARM_ADC_CAPABILITIES DriverCapabilities = {
    1,    /* Resolution 12 or 20 bits*/
    0     /* Reserved                */
};

/*
 * @func      : void Analog_config()
 * @brief     : vbat comparator value and register configuration
 * @parameter : NONE
 * @return    : NONE
 */
static void Analog_config()
{
    /* vbat register configuration       */
    analog_config_vbat_reg2();

    /* Comparator register configuration */
    analog_config_comp_reg2();
}

/*
 *    @func           : int32_t ADC_Initialize(ADC_RESOURCES *ADC, ARM_ADC_SignalEvent_t cb_event)
 *    @brief          : initialize the device
 *    @parameter[1]   : adc      : Pointer to /ref ADC_RESOURCES structure
 *    @parameter[2]   : cb_event : Pointer to /ref ARM_ADC_Signal_Event_t cb_event
 *    @return         : ARM_DRIVER_OK              : if driver initialized successfully
 *                    : ARM_DRIVER_ERROR_PARAMETER : if parameter is invalid or not
 */
static int32_t ADC_Initialize(ADC_RESOURCES *ADC, ARM_ADC_SignalEvent_t cb_event)
{
    int ret =  ARM_DRIVER_OK ;

    if(!cb_event)
        return ARM_DRIVER_ERROR_PARAMETER;

    /* User call back Event */
    ADC->cb_event = cb_event;

    /* Setting flag to initialize */
    ADC->state |= ADC_FLAG_DRV_INIT_DONE;

    return ret;
}

/*
 *    @func           : int32_t ADC_Uninitialize (ARM_ADC_SignalEvent_t cb_event)
 *    @brief          : Uninitialize the adc device
 *    @parameter[in]  : ADC    : Pointer to the structure ADC_RESOURCES
 *    @return         : ARM_DRIVER_OK              : if adc is successfully initialized
 *                    : ARM_DRIVER_ERROR_PARAMETER : if adc device is invalid
 */
static int32_t ADC_Uninitialize(ADC_RESOURCES *ADC)
{
    int ret = ARM_DRIVER_OK;

    /* parameter checking */
    if(!ADC)
        return ARM_DRIVER_ERROR_PARAMETER;

    /* Checking initialized has done or not */
    if(!(ADC->state & ADC_FLAG_DRV_INIT_DONE))
        return ARM_DRIVER_OK;

    /* set call back to NULL */
    ADC->cb_event = NULL;

    /* Reset last read channel */
    ADC->conv.last_read_channel = 0;

    /* flags */
    ADC->state = 0;

    return ret;
}

/*
 *    @func         : int32_t ADC_PowerControl(ARM_POWER_status status, ADC_RESOURCES *adc)
 *    @brief        : power the driver and enable NVIC
 *    @parameter[1] : ADC              : pointer to /ref ADC_RESOURCES
 *    @parameter[2] : state            : power state
 *    @return       : ARM_DRIVER_OK    : if power done successful
 *                    ARM_DRIVER_ERROR : if initialize is not done
*/
static int32_t ADC_PowerControl(ADC_RESOURCES *ADC, ARM_POWER_STATE state)
{
    int32_t ret = ARM_DRIVER_OK;

    switch(state)
    {
        case ARM_POWER_FULL:

            if (!(ADC->state & ADC_FLAG_DRV_INIT_DONE))
                return ARM_DRIVER_ERROR;

            if ((ADC->state & ADC_FLAG_DRV_POWER_DONE))
                return ARM_DRIVER_OK;

            /* Clear Any Pending IRQ */
            NVIC_ClearPendingIRQ (ADC->intr_done_irq_num);

            /* Set priority */
            NVIC_SetPriority (ADC->intr_done_irq_num, ADC->intr_done_irq_priority);

            /* Enable the NIVC */
            NVIC_EnableIRQ (ADC->intr_done_irq_num);

            /* Adc clock enable */
            AdcCoreClkControl(ADC, true);

            /*function include vbat and comparator address and it value */
            Analog_config();

            /* function reg1 register */
            AdcConfig(ADC->reg1_value);

            /* set user channel input */
            adc_init_channel_select(ADC->regs, ADC->conv.user_input);

            /* set the clock divisor */
            adc_set_clk_div(ADC->regs, ADC->clock_div);

            /* set avg sample value */
            adc_set_avg_sample(ADC->regs, ADC->avg_sample_num);

            /* set Sample width value */
            adc_set_sample_width(ADC->regs, ADC->sample_width);

            /* set number of n shift bits */
            adc_set_n_shift_bit(ADC->regs, ADC->shift_n_bit, ADC->shift_left_or_right);

            /* set sequencer control */
            adc_set_single_scan_mode(ADC->regs, &ADC->conv);

            /* Disable the interrupt (mask the interrupt(0xF))*/
            adc_mask_interrupt(ADC->regs);

            /* Set the power flag enabled */
            ADC->state |= ADC_FLAG_DRV_POWER_DONE;

            break;

        case ARM_POWER_OFF:

            /* Disable ADC NVIC */
            NVIC_DisableIRQ (ADC->intr_done_irq_num);

            /* Clear Any Pending IRQ */
            NVIC_ClearPendingIRQ (ADC->intr_done_irq_num);

            /* set the clock divisor */
            adc_set_clk_div(ADC->regs, ADC_CLOCK_DIV_MIN_VALUE);

            /* set avg sample value */
            adc_set_avg_sample(ADC->regs, ADC_AVG_SAMPLES_FOR_AVG_MIN);

            /* set Sample width value */
            adc_set_sample_width(ADC->regs, ADC_SAMPLE_WIDTH_MIN_VALUE);

            /* set number of n shift bits */
            adc_set_n_shift_bit(ADC->regs, 0, 0);

            /* Disable the interrupt (mask the interrupt(0xF)) */
            adc_mask_interrupt(ADC->regs);

            /* set sequencer control */
            adc_set_single_scan_mode(ADC->regs, &ADC->conv);

            /* Adc clock enable */
            AdcCoreClkControl(ADC, false);

            /* Reset the power status of ADC */
            ADC->state &= ~ADC_FLAG_DRV_POWER_DONE;

            break;

        case ARM_POWER_LOW:
        default:
             return ARM_DRIVER_ERROR_UNSUPPORTED;
            break;
    }
        return ret;
}

/*
 *    @func           : int32_t ADC_Start(void *data, uint32_t num, ADC_RESOURCES *adc)
 *    @brief          : this will start the adc and initialize interrupt
 *    @parameter[1]   : ADC  : pointer to ADC_RESOURCES structure
 *    @parameter[2]   : data : pointer to input data
 *    @parameter[3]   : num  : number of data
 *    @return         : ARM_DRIVER_OK              : if the function are return successful
 *                      ARM_DRIVER_ERROR_PARAMETER : if parameter are invalid
 */
static int32_t ADC_Start( ADC_RESOURCES *ADC, uint32_t *data, uint32_t num)
{
    if (!(ADC->state & ADC_FLAG_DRV_INIT_DONE))
        return ARM_DRIVER_ERROR;

    if (!(ADC->state & ADC_FLAG_DRV_POWER_DONE))
        return ARM_DRIVER_ERROR;

    /* checking data is present or not */
    if(data == NULL || num == 0U)
        return ARM_DRIVER_ERROR_PARAMETER;

    if(ADC->busy == 1U)
        return ARM_DRIVER_ERROR_BUSY;

    /* setup conversion structure as per info */
    ADC->conv.conv_buff = (uint32_t *) data;
    ADC->conv.total_cnt = num;
    ADC->conv.curr_cnt  = 0;
    ADC->conv.status    = ADC_CONV_STAT_NONE;

    /* active the conv busy flag */
    ADC->busy = 1U;

    /* enable the interrupt(unmask the interrupt 0x0)*/
    adc_unmask_interrupt(ADC->regs);

    /* Start the ADC */
    adc_enable(ADC->regs);

    return ARM_DRIVER_OK;
}

/*
 *    @func           : int32_t ADC_Stop( ADC_RESOURCES *adc)
 *    @brief          : Disable the adc
 *    @parameter      : ADC  : pointer to ADC_RESOURCES structure
 *    @return         : ARM_DRIVER_OK : if function return successfully
 */
static int32_t ADC_Stop(ADC_RESOURCES *ADC)
{
    if (!(ADC->state & ADC_FLAG_DRV_INIT_DONE))
        return ARM_DRIVER_ERROR;

    if (!(ADC->state & ADC_FLAG_DRV_POWER_DONE))
        return ARM_DRIVER_ERROR;

    /* Disable the adc */
    adc_disable(ADC->regs);

    return ARM_DRIVER_OK;
}

/*
 *    @func         : in32_t ADC_Control(uint32_t control , uint32_t arg, ADC_RESOURCES adc)
 *    @brief        : control the following
 *                    - ARM_SET_SHIFT_CONTROL             : to control shift control of bits
 *                    - ARM_SET_SEQUENCER_CTRL            : selecting sample individual or rotate through
 *                                                          each unmasked sample
 *                    - ARM_ADC_SEQUENCER_MSK_CTRL        : to control masking of the channel
 *                    - ARM_ADC_CHANNEL_INIT_VAL          : to select initial channel for storing
 *                    - ARM_SET_ADC_COMPARATOR_A          : to set comparator a value
 *                    - ARM_SET_ADC_COMPARATOR_B          : to set comparator b value
 *                    - ARM_SET_ADC_THRESHOLD_COMPARISON  : to set the threshold comparison
 *                    - ARM_SET_ADC_COMPARATOR_CONTROLLER : to control comparator
 *    @parameter[1] : ADC  : pointer to ADC_RESOURCES structure
 *    @parameter[2] : Control : Selecting the operation
 *    @parameter[3] : arg     : values for the the operation
 *    @return[1]    : ARM_DRIVER_OK              : if function return successfully
 *    @return[2]    : ARM_DRIVER_ERROR_PARAMETER : if adc parameter are invalid
 */
static int32_t ADC_Control(ADC_RESOURCES *ADC, uint32_t Control, uint32_t arg)
{
    int ret = ARM_DRIVER_OK;

    if (!(ADC->state & ADC_FLAG_DRV_INIT_DONE))
        return ARM_DRIVER_ERROR;

    if (!(ADC->state & ADC_FLAG_DRV_POWER_DONE))
        return ARM_DRIVER_ERROR;

    switch(Control)
    {
        case ARM_ADC_SHIFT_CONTROL:

            /*selecting the mode for the shifting bit left(0) or right(1) */
            if(arg)
            {
                adc_output_right_shift(ADC->regs);
            }
            else
            {
                adc_output_left_shift(ADC->regs);
            }

        break;

        case ARM_ADC_SEQUENCER_CTRL:

            if(!(arg == 0 || arg == 1))
                return ARM_DRIVER_ERROR_PARAMETER;

            /*selecting the mode of control for taking fixed sample(1) or rotate through the all sample(0)*/
            if(arg == ADC_SINGLE_SCAN_MODE)
            {
                adc_set_single_scan_mode(ADC->regs, &ADC->conv);
            }
            else
            {
                adc_set_continuous_scan_mode(ADC->regs, &ADC->conv);
            }

        break;

        case ARM_ADC_SEQUENCER_MSK_CH_CTRL:

            if(!(arg < ADC_MSK_ALL_CHANNELS))
                 return ARM_DRIVER_ERROR_PARAMETER;

            /* set channel to be masked */
            adc_sequencer_msk_ch_control(ADC->regs, arg);

        break;

        case ARM_ADC_CHANNEL_INIT_VAL:

            if(!(arg < ADC_MAX_INIT_CHANNEL))
                 return ARM_DRIVER_ERROR_PARAMETER;

            /* selecting the initial value */
            adc_init_channel_select(ADC->regs, arg);

            ADC->conv.user_input = 0;

        break;

        case ARM_ADC_COMPARATOR_A:
            /* set comparator A */
            adc_set_comparator_A(ADC->regs, arg);
        break;

        case ARM_ADC_COMPARATOR_B:
            /* set comparator B */
            adc_set_comparator_B(ADC->regs, arg);
        break;

        case ARM_ADC_THRESHOLD_COMPARISON:

            if(!(arg < 3))
                return ARM_DRIVER_ERROR_PARAMETER;
            /* set comparison control bit */
            adc_set_comparator_ctrl_bit(ADC->regs, arg);

        break;

        case ARM_ADC_COMPARATOR_CONTROLLER:

            if(!(arg == 0 || arg == 1))
                return ARM_DRIVER_ERROR_PARAMETER;

            /* set threshold comparison control*/
            if (arg)
            {
                adc_enable_process_control(ADC->regs);
                /* storing value of comparator control */
                ADC->conv.comp_ctrl_status = ENABLE;
            }
            else
            {
                adc_disable_process_control(ADC->regs);
                /* storing value of comparator control */
                ADC->conv.comp_ctrl_status = DISABLE;
            }
        break;

        default:
            return ARM_DRIVER_ERROR_PARAMETER;
    }

    return ret;
}

/* RTE_ADC120 */
#if (RTE_ADC120)

static ADC_RESOURCES ADC120_RES = {
  .cb_event               = NULL,                                    /* ARM_ADC_SignalEvent_t        */
  .regs                   = (ADC120_Type *)ADC120_BASE,              /* ADC register base address    */
  .conv.user_input        = RTE_ADC120_INPUT_NUM,                    /* user input                   */
  .drv_instance           = ADC_INSTANCE_0,                          /* Driver instances             */
  .intr_done_irq_num      = (IRQn_Type) ADC120_DONE0_IRQ_IRQn,        /* ADC IRQ number               */
  .intr_done_irq_priority = (uint32_t) RTE_ADC120_IRQ_PRIORITY,      /* ADC irq priority             */
  .busy                   = 0,                                       /* ADC busy                     */
  .clock_div              = RTE_ADC120_CLOCK_DIV,                    /* clock divisor                */
  .avg_sample_num         = RTE_ADC120_AVG_SAMPLE_NUM,               /* average sample number        */
  .sample_width           = RTE_ADC120_SAMPLE_WIDTH,                 /* sample width                 */
  .shift_n_bit            = RTE_ADC120_SHIFT_N_BIT,                  /* number of shift bit          */
  .shift_left_or_right    = RTE_ADC120_SHIFT_LEFT_OR_RIGHT,          /* shifting left to right       */
  .reg1_value             = (RTE_ADC120_TEST_EN << 12)                  |
                            (RTE_ADC120_DIFFERENTIAL_EN << 13)          |
                            (RTE_ADC120_COMPARATOR_EN << 14)            |
                            (RTE_ADC120_COMPARATOR_BIAS << 15)          |
                            (RTE_ADC120_VCM_RDIV_EN << 17)              |
                            (RTE_ADC12_CONFG_RESERVED_bits_18_23 << 18) |
                            (RTE_ADC12_CONFG_amux_cont << 24)
};

/*Function Name : ADC120_INTR_DONE_IRQHandler*/
void ADC120_DONE0_IRQHandler(void)
{
  conv_info_t *conv = &(ADC120_RES.conv);

  adc_irq_handler(ADC120_RES.regs, conv);

  if (conv->status & ADC_CONV_STAT_COMPLETE)
  {
      /* set busy flag to 0U */
      ADC120_RES.busy = 0U;

      ADC120_RES.cb_event(ARM_ADC_EVENT_CONVERSION_COMPLETE);
  }

  if (conv->status & ADC_CONV_STAT_CMP_THLD_ABOVE_A)
  {
      ADC120_RES.cb_event(ARM_ADC_COMPARATOR_THRESHOLD_ABOVE_A);
  }

  if (conv->status & ADC_CONV_STAT_CMP_THLD_ABOVE_B)
  {
      ADC120_RES.cb_event(ARM_ADC_COMPARATOR_THRESHOLD_ABOVE_B);
  }

  if (conv->status & ADC_CONV_STAT_CMP_THLD_BELOW_A)
  {
      ADC120_RES.cb_event(ARM_ADC_COMPARATOR_THRESHOLD_BELOW_A);
  }

  if (conv->status & ADC_CONV_STAT_CMP_THLD_BELOW_B)
  {
      ADC120_RES.cb_event(ARM_ADC_COMPARATOR_THRESHOLD_BELOW_B);
  }

  if (conv->status & ADC_CONV_STAT_CMP_THLD_BETWEEN_A_B)
  {
      ADC120_RES.cb_event(ARM_ADC_COMPARATOR_THRESHOLD_BETWEEN_A_B);
  }

  if (conv->status & ADC_CONV_STAT_CMP_THLD_OUTSIDE_A_B)
  {
      ADC120_RES.cb_event(ARM_ADC_COMPARATOR_THRESHOLD_OUTSIDE_A_B);
  }
  /* Clearing all events */
  conv->status = ADC_CONV_STAT_NONE;
}

/**
 @fn       ARM_DRIVER_VERSION ADC120_GetVersion(void)
 @brief    Get ADC120 VERSION
 @return   DriverVersion
**/
static ARM_DRIVER_VERSION ADC120_GetVersion(void)
{
  return DriverVersion;
}
/**
 @fn       ARM_ADC120_CAPABILITIES ADC120_GetCapabilities(void)
 @brief    Get ADC120 CAPABILITIES
 @return   DriverCapabilities
**/
static ARM_ADC_CAPABILITIES ADC120_GetCapabilities(void)
{
  return DriverCapabilities;
}

/*Function Name : ADC120_Intialize*/
static int32_t ADC120_Initialize(ARM_ADC_SignalEvent_t cb_event)
{
	 return (ADC_Initialize(&ADC120_RES, cb_event));
}

/*Function Name : ADC120_Unintialize*/
static int32_t ADC120_Uninitialize(void)
{
  return (ADC_Uninitialize(&ADC120_RES));
}

/*Function Name : ADC120_Stop*/
static int32_t ADC120_Start(uint32_t *data, uint32_t num)
{
  return (ADC_Start(&ADC120_RES, data, num));
}

/*Function Name : ADC120_Stop*/
static int32_t ADC120_Stop(void)
{
  return (ADC_Stop(&ADC120_RES));
}

/*Function Name : ADC120_PowerControl*/
static int32_t ADC120_PowerControl(ARM_POWER_STATE status)
{
  return(ADC_PowerControl(&ADC120_RES, status));
}

/*Function Name : ADC120_Control*/
static int32_t ADC120_Control(uint32_t Control, uint32_t arg)
{
  return (ADC_Control(&ADC120_RES, Control, arg));
}

extern ARM_DRIVER_ADC Driver_ADC120;
ARM_DRIVER_ADC Driver_ADC120 ={
    ADC120_GetVersion,
    ADC120_GetCapabilities,
    ADC120_Initialize,
    ADC120_Uninitialize,
    ADC120_Start,
    ADC120_Stop,
    ADC120_PowerControl,
    ADC120_Control
};
#endif /* RTE_ADC120 */

/* RTE_ADC121 */
#if (RTE_ADC121)

static ADC_RESOURCES ADC121_RES = {
  .cb_event               = NULL,                                    /* ARM_ADC_SignalEvent_t        */
  .regs                   = (ADC120_Type *)ADC121_BASE,              /* ADC register base address    */
  .conv.user_input        = RTE_ADC121_INPUT_NUM,                    /* user input                   */
  .drv_instance           = ADC_INSTANCE_1,                          /* Driver instances             */
  .intr_done_irq_num      = (IRQn_Type) ADC121_DONE0_IRQ_IRQn,        /* ADC IRQ number               */
  .intr_done_irq_priority = (uint32_t) RTE_ADC120_IRQ_PRIORITY,      /* ADC irq priority             */
  .busy                   = 0,                                       /* ADC busy                     */
  .clock_div              = RTE_ADC121_CLOCK_DIV,                    /* clock divisor                */
  .avg_sample_num         = RTE_ADC121_AVG_SAMPLE_NUM,               /* average sample number        */
  .sample_width           = RTE_ADC121_SAMPLE_WIDTH,                 /* sample width                 */
  .shift_n_bit            = RTE_ADC121_SHIFT_N_BIT,                  /* number of shift bit          */
  .shift_left_or_right    = RTE_ADC121_SHIFT_LEFT_OR_RIGHT,          /* shifting left to right       */
  .reg1_value             = (RTE_ADC121_TEST_EN << 12)                  |
                            (RTE_ADC121_DIFFERENTIAL_EN << 13)          |
                            (RTE_ADC121_COMPARATOR_EN << 14)            |
                            (RTE_ADC121_COMPARATOR_BIAS << 15)          |
                            (RTE_ADC121_VCM_RDIV_EN << 17)              |
                            (RTE_ADC12_CONFG_RESERVED_bits_18_23 << 18)   |
                            (RTE_ADC12_CONFG_amux_cont << 24)
};

/*Function Name : ADC121_INTR_DONE_IRQHandler*/
void ADC121_DONE0_IRQHandler(void)
{
  conv_info_t *conv = &(ADC121_RES.conv);

  adc_irq_handler(ADC121_RES.regs, conv);

  if (conv->status & ADC_CONV_STAT_COMPLETE)
  {
      /* set busy flag to 0U */
      ADC121_RES.busy = 0U;

      ADC121_RES.cb_event(ARM_ADC_EVENT_CONVERSION_COMPLETE);
  }

  if (conv->status & ADC_CONV_STAT_CMP_THLD_ABOVE_A)
  {
      ADC121_RES.cb_event(ARM_ADC_COMPARATOR_THRESHOLD_ABOVE_A);
  }

  if (conv->status & ADC_CONV_STAT_CMP_THLD_ABOVE_B)
  {
      ADC121_RES.cb_event(ARM_ADC_COMPARATOR_THRESHOLD_ABOVE_B);
  }

  if (conv->status & ADC_CONV_STAT_CMP_THLD_BELOW_A)
  {
      ADC121_RES.cb_event(ARM_ADC_COMPARATOR_THRESHOLD_BELOW_A);
  }

  if (conv->status & ADC_CONV_STAT_CMP_THLD_BELOW_B)
  {
      ADC121_RES.cb_event(ARM_ADC_COMPARATOR_THRESHOLD_BELOW_B);
  }

  if (conv->status & ADC_CONV_STAT_CMP_THLD_BETWEEN_A_B)
  {
      ADC121_RES.cb_event(ARM_ADC_COMPARATOR_THRESHOLD_BETWEEN_A_B);
  }

  if (conv->status & ADC_CONV_STAT_CMP_THLD_OUTSIDE_A_B)
  {
      ADC121_RES.cb_event(ARM_ADC_COMPARATOR_THRESHOLD_OUTSIDE_A_B);
  }
  /* Clearing all events */
  conv->status = ADC_CONV_STAT_NONE;
}

/**
 @fn       ARM_DRIVER_VERSION ADC121_GetVersion(void)
 @brief    Get ADC121 VERSION
 @return   DriverVersion
**/
static ARM_DRIVER_VERSION ADC121_GetVersion(void)
{
  return DriverVersion;
}
/**
 @fn       ARM_ADC121_CAPABILITIES ADC121_GetCapabilities(void)
 @brief    Get ADC121 CAPABILITIES
 @return   DriverCapabilities
**/
static ARM_ADC_CAPABILITIES ADC121_GetCapabilities(void)
{
  return DriverCapabilities;
}

/*Function Name : ADC121_Intialize*/
static int32_t ADC121_Initialize(ARM_ADC_SignalEvent_t cb_event)
{
  return (ADC_Initialize(&ADC121_RES, cb_event));
}

/*Function Name : ADC121_Unintialize*/
static int32_t ADC121_Uninitialize(void)
{
  return (ADC_Uninitialize(&ADC121_RES));
}

/*Function Name : ADC121_Start*/
static int32_t ADC121_Start(uint32_t *data, uint32_t num)
{
  return (ADC_Start(&ADC121_RES, data, num));
}

/*Function Name : ADC121_Stop*/
static int32_t ADC121_Stop(void)
{
  return (ADC_Stop(&ADC121_RES));
}

/*Function Name : ADC121_PowerControl*/
static int32_t ADC121_PowerControl(ARM_POWER_STATE status)
{
  return(ADC_PowerControl(&ADC121_RES, status));
}

/*Function Name : ADC121_Control*/
static int32_t ADC121_Control(uint32_t Control, uint32_t arg)
{
  return (ADC_Control(&ADC121_RES, Control, arg));
}

extern ARM_DRIVER_ADC Driver_ADC121;
ARM_DRIVER_ADC Driver_ADC121 ={
    ADC121_GetVersion,
    ADC121_GetCapabilities,
    ADC121_Initialize,
    ADC121_Uninitialize,
    ADC121_Start,
    ADC121_Stop,
    ADC121_PowerControl,
    ADC121_Control
};
#endif /* RTE_ADC121 */

/* RTE_ADC122 */
#if (RTE_ADC122)

static ADC_RESOURCES ADC122_RES = {
  .cb_event               = NULL,                                    /* ARM_ADC_SignalEvent_t        */
  .regs                   = (ADC120_Type *)ADC122_BASE,              /* ADC register base address    */
  .conv.user_input        = RTE_ADC122_INPUT_NUM,                    /* user input                   */
  .drv_instance           = ADC_INSTANCE_2,                          /* Driver instances             */
  .intr_done_irq_num      = (IRQn_Type) ADC122_DONE0_IRQ_IRQn,       /* ADC IRQ number               */
  .intr_done_irq_priority = (uint32_t) RTE_ADC120_IRQ_PRIORITY,      /* ADC irq priority             */
  .busy                   = 0,                                       /* ADC busy                     */
  .clock_div              = RTE_ADC122_CLOCK_DIV,                    /* clock divisor                */
  .avg_sample_num         = RTE_ADC122_AVG_SAMPLE_NUM,               /* average sample number        */
  .sample_width           = RTE_ADC122_SAMPLE_WIDTH,                 /* sample width                 */
  .shift_n_bit            = RTE_ADC122_SHIFT_N_BIT,                  /* number of shift bit          */
  .shift_left_or_right    = RTE_ADC122_SHIFT_LEFT_OR_RIGHT,          /* shifting left to right       */
  .reg1_value             = (RTE_ADC122_TEST_EN << 12)                  |
                            (RTE_ADC122_DIFFERENTIAL_EN << 13)          |
                            (RTE_ADC122_COMPARATOR_EN << 14)            |
                            (RTE_ADC122_COMPARATOR_BIAS << 15)          |
                            (RTE_ADC122_VCM_RDIV_EN << 17)              |
                            (RTE_ADC12_CONFG_RESERVED_bits_18_23 << 18)   |
                            (RTE_ADC12_CONFG_amux_cont << 24)
};

/*Function Name : ADC120_INTR_DONE_IRQHandler*/
void ADC122_DONE0_IRQHandler(void)
{
  conv_info_t *conv = &(ADC122_RES.conv);

  adc_irq_handler(ADC122_RES.regs, conv);

  if (conv->status & ADC_CONV_STAT_COMPLETE)
  {
      conv->status = ADC_CONV_STAT_NONE;

      /* set busy flag to 0U */
      ADC122_RES.busy = 0U;

      ADC122_RES.cb_event(ARM_ADC_EVENT_CONVERSION_COMPLETE);
  }

  if (conv->status & ADC_CONV_STAT_CMP_THLD_ABOVE_A)
  {
      ADC122_RES.cb_event(ARM_ADC_COMPARATOR_THRESHOLD_ABOVE_A);
  }

  if (conv->status & ADC_CONV_STAT_CMP_THLD_ABOVE_B)
  {
      ADC122_RES.cb_event(ARM_ADC_COMPARATOR_THRESHOLD_ABOVE_B);
  }

  if (conv->status & ADC_CONV_STAT_CMP_THLD_BELOW_A)
  {
      ADC122_RES.cb_event(ARM_ADC_COMPARATOR_THRESHOLD_BELOW_A);
  }

  if (conv->status & ADC_CONV_STAT_CMP_THLD_BELOW_B)
  {
      ADC122_RES.cb_event(ARM_ADC_COMPARATOR_THRESHOLD_BELOW_B);
  }

  if (conv->status & ADC_CONV_STAT_CMP_THLD_BETWEEN_A_B)
  {
      ADC122_RES.cb_event(ARM_ADC_COMPARATOR_THRESHOLD_BETWEEN_A_B);
  }

  if (conv->status & ADC_CONV_STAT_CMP_THLD_OUTSIDE_A_B)
  {
      ADC122_RES.cb_event(ARM_ADC_COMPARATOR_THRESHOLD_OUTSIDE_A_B);
  }
  /* Clearing all events */
  conv->status = ADC_CONV_STAT_NONE;
}

/**
 @fn       ARM_DRIVER_VERSION ADC1_GetVersion(void)
 @brief    Get ADC1 VERSION
 @return   DriverVersion
**/
static ARM_DRIVER_VERSION ADC122_GetVersion(void)
{
  return DriverVersion;
}
/**
 @fn       ARM_ADC122_CAPABILITIES ADC122_GetCapabilities(void)
 @brief    Get ADC122 CAPABILITIES
 @return   DriverCapabilities
**/
static ARM_ADC_CAPABILITIES ADC122_GetCapabilities(void)
{
  return DriverCapabilities;
}

/*Function Name : ADC122_Intialize*/
static int32_t ADC122_Initialize(ARM_ADC_SignalEvent_t cb_event)
{
  return (ADC_Initialize(&ADC122_RES, cb_event));
}

/*Function Name : ADC122_Unintialize*/
static int32_t ADC122_Uninitialize(void)
{
  return (ADC_Uninitialize(&ADC122_RES));
}

/*Function Name : ADC122_Start*/
static int32_t ADC122_Start(uint32_t *data, uint32_t num)
{
  return (ADC_Start(&ADC122_RES, data, num));
}

/*Function Name : ADC122_Stop*/
static int32_t ADC122_Stop(void)
{
  return (ADC_Stop(&ADC122_RES));
}

/*Function Name : ADC122_PowerControl*/
static int32_t ADC122_PowerControl(ARM_POWER_STATE status)
{
  return(ADC_PowerControl( &ADC122_RES, status));
}

/*Function Name : ADC122_Control*/
static int32_t ADC122_Control(uint32_t Control, uint32_t arg)
{
  return (ADC_Control(&ADC122_RES, Control, arg));
}

extern ARM_DRIVER_ADC Driver_ADC122;
ARM_DRIVER_ADC Driver_ADC122 ={
    ADC122_GetVersion,
    ADC122_GetCapabilities,
    ADC122_Initialize,
    ADC122_Uninitialize,
    ADC122_Start,
    ADC122_Stop,
    ADC122_PowerControl,
    ADC122_Control
};
#endif /* RTE_ADC122 */
