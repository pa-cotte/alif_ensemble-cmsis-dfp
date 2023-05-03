/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

#ifndef DRIVER_DAC_PRIVATE_H_
#define DRIVER_DAC_PRIVATE_H_

#ifdef  __cplusplus
extern "C"
{
#endif

/* System includes */
#include "RTE_Device.h"
#include "RTE_Components.h"
#include CMSIS_device_header

/* Project includes */
#include"Driver_DAC.h"
#include "dac.h"

/**
 * enum DAC_INSTANCE.
 * DAC instances.
 */
typedef enum _DAC_INSTANCE
{
    DAC_INSTANCE_0,    /* DAC instance - 0 */
    DAC_INSTANCE_1     /* DAC instance - 1 */
}DAC_INSTANCE;

/**
 @brief   : DAC flags to check the DAC initialization, DAC power done and DAC started.
 */
typedef enum {
    DAC_FLAG_DRV_INIT_DONE    = (1U << 0),  /* DAC Driver is Initialized */
    DAC_FLAG_DRV_POWER_DONE   = (1U << 1),  /* DAC Driver is Powered */
    DAC_FLAG_DRV_STARTED      = (1U << 2)   /* DAC Driver is Started */
} DAC_DRIVER_STATE;

/**
 * struct DAC_RESOURCES: structure representing a DAC device
 * @regs     : Register address of the DAC
 * @flags    : DAC driver flags
 * @config   : DAC configuration information
 */
typedef struct _DAC_resources
{
    DAC_Type            *regs;         /* DAC register address */
    DAC_DRIVER_STATE     flags;        /* DAC Driver Flags */
    uint32_t             config;       /* DAC configuration information */
    DAC_INSTANCE         drv_instance; /* DAC Driver instance */
}DAC_RESOURCES;

#ifdef  __cplusplus
}
#endif

#endif /* DRIVER_DAC_PRIVATE_H_ */

