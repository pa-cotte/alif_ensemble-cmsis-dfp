/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

#ifndef DRIVER_CMP_PRIVATE_H_
#define DRIVER_CMP_PRIVATE_H_

#ifdef  __cplusplus
extern "C"
{
#endif

/* System includes */
#include "RTE_Device.h"
#include "RTE_Components.h"
#include CMSIS_device_header

/* Project includes */
#include"Driver_Comparator.h"
#include "cmp.h"

#define CMP_CLOCK_CTRL_BASE         0x4902F038
#define CMP_CTRL_BASE               CMP0_BASE
#define CMP_OUTPUT_BASE             0x49020038
/**
 * enum CMP_INSTANCE.
 * CMP instances.
 */
typedef enum _CMP_INSTANCE
{
    CMP_INSTANCE_0,                 /**< CMP instance - 0 */
    CMP_INSTANCE_1,                 /**< CMP instance - 1 */
    CMP_INSTANCE_2,                 /**< CMP instance - 2 */
    CMP_INSTANCE_3                  /**< CMP instance - 3 */
} CMP_INSTANCE;

/**
 @brief   : CMP Driver states
 */
typedef volatile struct _CMP_DRIVER_STATE {
    uint32_t initialized : 1;                    /* Driver Initialized    */
    uint32_t powered     : 1;                    /* Driver powered        */
    uint32_t reserved    : 30;                   /* Reserved              */
} CMP_DRIVER_STATE;

/**
 * struct CMP_RESOURCES: structure representing a Analog comparator device
 * @regs           : Register address of the Comparator
 * @drv_instance   : Driver instance
 * @state          : Comparator driver state
 * @irq_num        : Comparator interrupt number
 * @config         : Comparator configuration information
 * @irq_priority   : Comparator interrupt Priority
 * @output_mux_sel : Comparator output mux
 * @input_enable   : To select the comparator
 */
typedef struct _CMP_RESOURCES{
    ARM_Comparator_SignalEvent_t  cb_event;        /* Comparator application event callback */
    CMP0_Type                     *regs;           /* Comparator register base address      */
    CMP_INSTANCE                  drv_instance;    /* Driver instance                       */
    CMP_DRIVER_STATE              state;           /* Comparator Driver state */
    IRQn_Type                     irq_num;         /* Comparator interrupt number */
    uint32_t                      config;          /* Comparator configuration information */
    uint32_t                      irq_priority;    /* Comparator interrupt Priority */
    uint32_t                      output_mux_sel;  /* Comparator output mux */
    uint32_t                      input_enable;    /* To select the comparator */
}CMP_RESOURCES;

/*
 * @func        void CmpOutputSel(CMP_RESOURCES *cmp)
 * @brief       Select the Comparator instance for the output
 * @param[in]   cmp    : pointer to the CMP_RESOURCES
 */
static inline void CmpOutputSel(CMP_RESOURCES *cmp)
{
    volatile uint32_t *output = (volatile uint32_t *)CMP_OUTPUT_BASE;

    /* Select the comparator output and add to the ADC reg1 address */
    *output = cmp->output_mux_sel;
}

/**
 * @fn          void CmpEnable(CMP_RESOURCES *cmp, bool ctrl)
 * @brief       To enable or disable the comparator module
 * @param[in]   ctrl : Enable or Disable the comparator module
 * @param[in]   cmp  : Pointer to CMP_RESOURCES structure
 * @return      None
 */
static inline void CmpEnable(CMP_RESOURCES *cmp, bool ctrl)
{
    /* comparator configuration register is provided on CMP0 base address */
    volatile uint32_t *cmp_reg = (volatile uint32_t *)CMP_CTRL_BASE;

    if (ctrl)
    {
        /* Enable the Analog Comparator module */
        *cmp_reg |= (cmp->input_enable);
    }
    else
    {
        /* Disable the Analog Comparator module */
        *cmp_reg &= ~(cmp->input_enable);
    }
}

/**
 * @fn          void CmpSetConfig(uint32_t config_value)
 * @brief       Add configuration value to the reg1 of CMP0 instance
 * @param[in]   config_value : To store the configuration values
 * @return      None
 */
static void CmpSetConfig(uint32_t config_value)
{
    /* comparator configuration register is provided on CMP0 base address */
    volatile uint32_t *cmp_reg = (volatile uint32_t *)CMP_CTRL_BASE;

    /* Adding configuration values to the reg1 of Comp0 instance  */
    *cmp_reg = config_value;
}

#endif /* DRIVER_CMP_PRIVATE_H_ */
