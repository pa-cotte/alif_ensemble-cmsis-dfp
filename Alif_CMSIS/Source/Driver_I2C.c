/* Copyright (C) 2022 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/**************************************************************************//**
 * @file     Driver_I2C.c
 * @author   Tanay Rami         | Prabhakar Kumar
 * @email    tanay@alifsemi.com | prabhakar.kumar@alifsemi.com
 * @version  V1.0.0
 * @date     20-June-2020       | 21-July-2022
 * @brief    CMSIS-Driver for I2C.
 * @bug      None.
 * @Note	 None
 ******************************************************************************/

/* Includes */
#include <stddef.h>
#include <stdint.h>
#include "stdio.h"

/* system includes */
#include "Driver_I2C.h"
/* Project includes */
#include "i2c_ll_drv.h"

/* Driver version */
#define ARM_I2C_DRV_VERSION    ARM_DRIVER_VERSION_MAJOR_MINOR(1, 0)

/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion = {
    ARM_I2C_API_VERSION,
    ARM_I2C_DRV_VERSION
};

/* Driver Capabilities */
static const ARM_I2C_CAPABILITIES DriverCapabilities = {
    1,  /* supports 10-bit addressing */
    0   /* reserved */
};

/* Functions ------------------------------------------------------------------*/

/**
 * @brief   get i2c version
 * @note    none
 * @param   none
 * @retval  driver version
 */
static ARM_DRIVER_VERSION ARM_I2C_GetVersion(void)
{
    return DriverVersion;
}

/**
 * @brief   get i2c capabilites
 * @note    none
 * @param   none
 * @retval  driver capabilites
 */
static ARM_I2C_CAPABILITIES ARM_I2C_GetCapabilities(void)
{
    return DriverCapabilities;
}

/**
 * @brief   CMSIS-Driver i2c initialize
 * @note    it will use interrupt method for data send and receive.
 * @param   cb_event    : Pointer to \ref ARM_I2C_SignalEvent
 * @param   i2c         : Pointer to i2c resources structure
 * @retval  ARM_DRIVER_OK : successfully initialized
 */
static int32_t ARM_I2C_Initialize (ARM_I2C_SignalEvent_t cb_event,
                                   i2c_resources_t       *i2c)
{
    int ret = ARM_DRIVER_OK;

    /* Driver is already initialized */
    if (i2c->info->flags & I2C_FLAG_INITIALIZED)
        return ARM_DRIVER_OK;

    /* calling i2c initialize lower level API */
    i2c_initialize(i2c);

    /* set the user callback event. */
    i2c->info->cb_event = cb_event;

    /* set the flag as initialized. */
    i2c->info->flags |= I2C_FLAG_INITIALIZED;

    return ARM_DRIVER_OK;
}
/**
 * @brief   CMSIS-Driver i2c uninitialize
 * @note    none
 * @param   i2c    : Pointer to i2c resources structure
 * @retval  ARM_DRIVER_OK : successfully uninitialized
 */
static int32_t ARM_I2C_Uninitialize(i2c_resources_t *i2c)
{
    int ret = ARM_DRIVER_OK;

    /* Driver is initialized or not */
    if ( (i2c->info->flags & I2C_FLAG_INITIALIZED) == 0)
        return ARM_DRIVER_OK;

    /* Calling i2c un-initialize lower level API */
    i2c_uninitialize(i2c);

    /* Reset the flags. */
    i2c->info->flags = 0U;

    return ret;
}

/**
 * @func    : static int32_t ARM_I2C_PowerControl (ARM_POWER_STATE state, i2c_resources_t *i2c,)
 * @brief   : Power the driver and enable the NVIC
 * @param   : state : Power state
 * @param   : i2c   : Pointer to i2c resources structure
 * @return  : ARM_DRIVER_OK
 */
static int32_t ARM_I2C_PowerControl (ARM_POWER_STATE state, i2c_resources_t *i2c)
{
    uint8_t ret = ARM_DRIVER_OK;

    i2c_info_t *i2c_info_ptr = (i2c_info_t *)(i2c->info);

    switch (state)
    {
    case ARM_POWER_FULL:

         /* check for Driver initialization */
         if (!(i2c_info_ptr->flags & I2C_FLAG_INITIALIZED))
             return ARM_DRIVER_ERROR;

         /* check for the power is done before initialization or not */
         if(i2c_info_ptr ->flags & I2C_FLAG_POWERED)
             return ARM_DRIVER_OK;

         /* Clear Any Pending Irq */
         NVIC_ClearPendingIRQ(i2c->irq_num);

         /* Set Priority */
         NVIC_SetPriority(i2c->irq_num, i2c->irq_priority);

         /* Enable IRQ */
         NVIC_EnableIRQ(i2c->irq_num);

        break;
    case ARM_POWER_LOW:

         /* Disable the IRQ */
         NVIC_DisableIRQ(i2c->irq_num);

         /* Clearing pending */
         NVIC_ClearPendingIRQ(i2c->irq_num);

        break;
    case ARM_POWER_OFF:
        return ARM_DRIVER_ERROR_UNSUPPORTED;
    }
    return ret;
}


/**
 * @brief   CMSIS-Driver i2c master transmit
 *          Start sending data to i2c transmitter.
 * @note    I2C_FLAG_MASTER_SETUP flag should be enabled first /ref ARM_I2C_BUS_SPEED
 * @param   data    : Pointer to buffer with data to send to i2c transmitter
 * @param   num     : Number of data items to send
 * @param   i2c     : Pointer to i2c resources structure
 * @retval  ARM_DRIVER_ERROR_PARAMETER  : error in parameter
 * @retval  ARM_DRIVER_ERROR            : error in driver
 * @retval  ARM_DRIVER_OK               : success in interrupt case
 * @retval  ARM_DRIVER_ERROR_BUSY       : driver busy in interrupt case
 * @retval  transmit count              : For data transmit count /ref ARM_I2C_GetDataCount
 */
static int32_t ARM_I2C_MasterTransmit(i2c_resources_t *i2c,
                                      uint32_t        addr,
                                      const uint8_t   *data,
                                      uint32_t        num,
                                      bool            xfer_pending)
{
    int ret = ARM_DRIVER_OK;

    /* addr 7bit addr: 0x7F , 10bit addr: 0x3FF */
    if ((data == NULL) || (num == 0U) || (addr > 0x3FF))
    {
        /* Invalid parameters */
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if ((i2c->info->flags & I2C_FLAG_MASTER_SETUP) == 0U)
    {
        /* error master mode is not configured (mode not selected)
         * I2C_FLAG_MASTER_SETUP should be enabled first \ref ARM_I2C_BUS_SPEED
         */
        return ARM_DRIVER_ERROR;
    }

    /* Enable the I2C master transmit interrupt */
    ret = i2c_enable_master_transmit_interrupt(i2c, addr, data, num, xfer_pending);
    return ret;
}


/**
 * @brief   CMSIS-Driver i2c master receive
 *          Start receiving data from i2c receiver.
 * @note    none
 * @param   data                        : Pointer to buffer for data to receive from i2c receiver
 * @param   num                         : Number of data items to receive
 * @param   i2c                         : Pointer to i2c resources structure
 * @retval  ARM_DRIVER_ERROR_PARAMETER  : error in parameter
 * @retval  ARM_DRIVER_ERROR            : error in driver
 * @retval  ARM_DRIVER_OK               : success in interrupt case
 * @retval  ARM_DRIVER_ERROR_BUSY       : driver busy in interrupt case
 * @retval  received count              : For data receive count /ref ARM_I2C_GetDataCount
 */
static int32_t ARM_I2C_MasterReceive(i2c_resources_t *i2c,
                                     uint32_t        addr,
                                     uint8_t         *data,
                                     uint32_t        num,
                                     bool            xfer_pending)
{
    int ret = ARM_DRIVER_OK;

    /* addr 7bit addr: 0x7F , 10bit addr: 0x3FF */
    if ((data == NULL) || (num == 0U) || (addr > 0x3FF))
    {
        /* Invalid parameters */
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if ((i2c->info->flags & I2C_FLAG_MASTER_SETUP) == 0U)
    {
        /* error master mode is not configured (mode not selected)
         * I2C_FLAG_MASTER_SETUP should be enabled first \ref ARM_I2C_BUS_SPEED
         */
        return ARM_DRIVER_ERROR;
    }


    /* Enable I2C master receiver interrupt */
    ret = i2c_enable_master_receive_interrupt(i2c, addr, data, num, xfer_pending);

    return ret;
}

/**
 * @brief   CMSIS-Driver i2c slave transmit
 * @note    not implemented yet.
 */
static int32_t ARM_I2C_SlaveTransmit(i2c_resources_t              *i2c,
                                     const uint8_t *data, uint32_t num)
{
    int ret = ARM_DRIVER_OK;

    if((data == NULL) || (num == 0U))
        return ARM_DRIVER_ERROR_PARAMETER;

    /* Check Slave mode is enabled */
    if((i2c->info->flags & I2C_FLAG_SLAVE_SETUP) == 0U)
        return ARM_DRIVER_ERROR;

    ret = i2c_enable_slave_transmit_interrupt(i2c, data, num);

    return ret;
}

/**
 * @brief   CMSIS-Driver i2c slave receive
 * @note    not implemented yet.
 */
static int32_t ARM_I2C_SlaveReceive(i2c_resources_t    *i2c, uint8_t *data,
                                                             uint32_t  num)
{
    int ret = ARM_DRIVER_OK;;

    if((data == NULL) || (num == 0U))
        return ARM_DRIVER_ERROR_PARAMETER;

    /* Check Slave mode is enabled */
    if((i2c->info->flags & I2C_FLAG_SLAVE_SETUP) == 0U)
        return ARM_DRIVER_ERROR;

    ret = i2c_enable_slave_receive_interrupt(i2c, data, num);

    return ret;
}

/**
 * @brief   CMSIS-Driver i2c get transfer data count
 * @note    it can be either transmit or receive data count which perform last
 *          (useful only in interrupt mode)
 * @param   i2c   : Pointer to i2c resources structure
 * @retval  transfer data count
 */
static int32_t ARM_I2C_GetDataCount(const i2c_resources_t *i2c)
{
    /* return common count for both tx/rx */
    return i2c->info->transfer.curr_cnt;
}

/**
 * @brief   CMSIS-Driver i2c control
 *          Control i2c Interface.
 * @note    none
 * @param   control    : Operation
 * @param   arg        : Argument of operation (optional)
 * @param   i2c        : Pointer to i2c resources structure
 * @retval  common \ref execution_status and driver specific \ref i2c_execution_status
 */
static int32_t ARM_I2C_Control(i2c_resources_t  *i2c,
                               uint32_t         control,
                               uint32_t         arg)
{
    int32_t ret = ARM_DRIVER_OK;

    switch (control)
    {
        case ARM_I2C_OWN_ADDRESS:
            /* Set Own Slave Address */
            ret = i2c_slave_init(i2c, arg);
            if(ret == ARM_DRIVER_OK)
            {
               i2c->info->flags |= I2C_FLAG_SLAVE_SETUP;
            }

            break;

        case ARM_I2C_BUS_SPEED:

            /* arg is i2c bus speed */
            ret = i2c_master_init(i2c, arg);
            if(ret == ARM_DRIVER_OK)
            {
                /* setup master flag */
                i2c->info->flags |= I2C_FLAG_MASTER_SETUP;
            }
            break;

        case ARM_I2C_BUS_CLEAR:
            /* disable device, clear all the interrupt, enable device. */
            i2c_reset_device(i2c);
            break;

        case ARM_I2C_ABORT_TRANSFER:

            /* only useful in interrupt method,
             * no effect on polling method.
             */

            /* i2c is half-duplex, at a time it can be either tx or rx
             * not sure which one is running, so clearing both. */
            i2c_abort_transmit(i2c);
            i2c_abort_receive(i2c);
            break;

        default:
            return ARM_DRIVER_ERROR_UNSUPPORTED;
    }
    return ret;
}

/**
 * @brief   CMSIS-Driver i2c get status
 * @note    none
 * @param   i2c    : Pointer to i2c resources structure
 * @retval  ARM_i2c_STATUS
 */
static ARM_I2C_STATUS ARM_I2C_GetStatus(const i2c_resources_t  *i2c)
{
    return i2c->info->status;
}

/**
 * @brief   CMSIS-Driver i2c interrupt handler
 * @note    none
 * @param   i2c    : Pointer to i2c resources structure
 * @retval  none
 */
static void ARM_I2C_IRQHandler (i2c_resources_t  *i2c)
{
    i2c_irq_handler (i2c);
}

/* I2C0 Driver Instance */
#if (RTE_I2C0)

static i2c_info_t i2c0_info = {0};

/* I2C0 Driver Resources */
static i2c_resources_t I2C0_Resources =
{
    .reg_base       = (uint32_t)I2C0_BASE,
    .clk            = (uint32_t)I2C_PERIPHERAL_CLOCK,
    .info           = &i2c0_info,
    .irq_num        = (IRQn_Type)I2C0_IRQ_IRQn,
    .irq_priority   = (uint32_t)RTE_I2C0_IRQ_PRIORITY
};

static int32_t I2C0_Initialize (ARM_I2C_SignalEvent_t cb_event)
{
    return ARM_I2C_Initialize(cb_event, &I2C0_Resources);
}

static int32_t I2C0_Uninitialize(void)
{
    return ARM_I2C_Uninitialize(&I2C0_Resources);
}

static int32_t I2C0_PowerControl(ARM_POWER_STATE state)
{
    return ARM_I2C_PowerControl(state, &I2C0_Resources);
}

static int32_t I2C0_MasterTransmit(uint32_t addr, const uint8_t *data,
                                   uint32_t num,  bool xfer_pending)
{
    return ARM_I2C_MasterTransmit(&I2C0_Resources, addr, data, num, xfer_pending);
}

static int32_t I2C0_MasterReceive(uint32_t addr, uint8_t *data,
                                  uint32_t num, bool xfer_pending)
{
    return (ARM_I2C_MasterReceive(&I2C0_Resources, addr, data, num, xfer_pending));
}

static int32_t I2C0_SlaveTransmit(const uint8_t *data, uint32_t num)
{
    return (ARM_I2C_SlaveTransmit(&I2C0_Resources, data, num));
}

static int32_t I2C0_SlaveReceive(uint8_t *data, uint32_t num)
{
    return (ARM_I2C_SlaveReceive(&I2C0_Resources, data, num));
}

static int32_t I2C0_GetDataCount(void)
{
    return (ARM_I2C_GetDataCount(&I2C0_Resources));
}

static int32_t I2C0_Control(uint32_t control, uint32_t arg)
{
    return (ARM_I2C_Control(&I2C0_Resources, control, arg));
}

static ARM_I2C_STATUS I2C0_GetStatus(void)
{
    return (ARM_I2C_GetStatus(&I2C0_Resources));
}

void I2C0_IRQHandler(void)
{
	ARM_I2C_IRQHandler (&I2C0_Resources);
}

/* I2C0 Driver Control Block */
extern ARM_DRIVER_I2C Driver_I2C0;
ARM_DRIVER_I2C Driver_I2C0 = {
    ARM_I2C_GetVersion,
    ARM_I2C_GetCapabilities,
    I2C0_Initialize,
    I2C0_Uninitialize,
    I2C0_PowerControl,
    I2C0_MasterTransmit,
    I2C0_MasterReceive,
    I2C0_SlaveTransmit,
    I2C0_SlaveReceive,
    I2C0_GetDataCount,
    I2C0_Control,
    I2C0_GetStatus
};
#endif /* RTE_I2C0 */

/* I2C1 Driver Instance */
#if (RTE_I2C1)

static i2c_info_t i2c1_info = {0};

/* I2C1 Driver Resources */
static i2c_resources_t I2C1_Resources =
{
    .reg_base       = (uint32_t)I2C1_BASE,
    .clk            = (uint32_t)I2C_PERIPHERAL_CLOCK,
    .info           = &i2c1_info,
    .irq_num        = (IRQn_Type)I2C1_IRQ_IRQn,
    .irq_priority   = (uint32_t)RTE_I2C1_IRQ_PRIORITY
};

static int32_t I2C1_Initialize (ARM_I2C_SignalEvent_t cb_event)
{
    return ARM_I2C_Initialize(cb_event, &I2C1_Resources);
}

static int32_t I2C1_Uninitialize(void)
{
    return ARM_I2C_Uninitialize(&I2C1_Resources);
}

static int32_t I2C1_PowerControl(ARM_POWER_STATE state)
{
	return ARM_I2C_PowerControl(state, &I2C1_Resources);
}

static int32_t I2C1_MasterTransmit(uint32_t addr, const uint8_t *data,
                                   uint32_t num, bool xfer_pending)
{
    return ARM_I2C_MasterTransmit(&I2C1_Resources, addr, data, num, xfer_pending);
}

static int32_t I2C1_MasterReceive(uint32_t addr, uint8_t *data,
                                  uint32_t num, bool xfer_pending)
{
    return (ARM_I2C_MasterReceive(&I2C1_Resources, addr, data, num, xfer_pending));
}

static int32_t I2C1_SlaveTransmit(const uint8_t *data, uint32_t num)
{
    return (ARM_I2C_SlaveTransmit(&I2C1_Resources, data, num));
}

static int32_t I2C1_SlaveReceive(uint8_t *data, uint32_t num)
{
    return (ARM_I2C_SlaveReceive(&I2C1_Resources, data, num));
}

static int32_t I2C1_GetDataCount(void)
{
    return (ARM_I2C_GetDataCount(&I2C1_Resources));
}

static int32_t I2C1_Control(uint32_t control, uint32_t arg)
{
    return (ARM_I2C_Control(&I2C1_Resources, control, arg));
}

static ARM_I2C_STATUS I2C1_GetStatus(void)
{
    return (ARM_I2C_GetStatus(&I2C1_Resources));
}

void I2C1_IRQHandler (void)
{
    ARM_I2C_IRQHandler (&I2C1_Resources);
}

/* I2C1 Driver Control Block */
extern ARM_DRIVER_I2C Driver_I2C1;
ARM_DRIVER_I2C Driver_I2C1 = {
    ARM_I2C_GetVersion,
    ARM_I2C_GetCapabilities,
    I2C1_Initialize,
    I2C1_Uninitialize,
    I2C1_PowerControl,
    I2C1_MasterTransmit,
    I2C1_MasterReceive,
    I2C1_SlaveTransmit,
    I2C1_SlaveReceive,
    I2C1_GetDataCount,
    I2C1_Control,
    I2C1_GetStatus
};
#endif /* RTE_I2C1 */


/* I2C2 Driver Instance */
#if (RTE_I2C2)

static i2c_info_t i2c2_info = {0};

/* I2C2 Driver Resources */
static i2c_resources_t I2C2_Resources =
{
    .reg_base       = (uint32_t)I2C2_BASE,
    .clk            = (uint32_t)I2C_PERIPHERAL_CLOCK,
    .info           = &i2c2_info,
    .irq_num        = (IRQn_Type)I2C2_IRQ_IRQn,
    .irq_priority   = (uint32_t)RTE_I2C2_IRQ_PRIORITY
};

static int32_t I2C2_Initialize (ARM_I2C_SignalEvent_t cb_event)
{
    return ARM_I2C_Initialize(cb_event, &I2C2_Resources);
}

static int32_t I2C2_Uninitialize(void)
{
    return ARM_I2C_Uninitialize(&I2C2_Resources);
}

static int32_t I2C2_PowerControl(ARM_POWER_STATE state)
{
	return ARM_I2C_PowerControl(state, &I2C2_Resources);
}

static int32_t I2C2_MasterTransmit(uint32_t addr, const uint8_t *data,
                                   uint32_t num, bool xfer_pending)
{
    return ARM_I2C_MasterTransmit(&I2C2_Resources, addr, data, num, xfer_pending);
}

static int32_t I2C2_MasterReceive(uint32_t addr, uint8_t *data,
                                  uint32_t num, bool xfer_pending)
{
    return (ARM_I2C_MasterReceive(&I2C2_Resources, addr, data, num, xfer_pending));
}

static int32_t I2C2_SlaveTransmit(const uint8_t *data, uint32_t num)
{
    return (ARM_I2C_SlaveTransmit(&I2C2_Resources, data, num));
}

static int32_t I2C2_SlaveReceive(uint8_t *data, uint32_t num)
{
    return (ARM_I2C_SlaveReceive(&I2C2_Resources, data, num));
}

static int32_t I2C2_GetDataCount(void)
{
    return (ARM_I2C_GetDataCount(&I2C2_Resources));
}

static int32_t I2C2_Control(uint32_t control, uint32_t arg)
{
    return (ARM_I2C_Control(&I2C2_Resources, control, arg));
}

static ARM_I2C_STATUS I2C2_GetStatus(void)
{
    return (ARM_I2C_GetStatus(&I2C2_Resources));
}

void I2C2_IRQHandler (void)
{
    ARM_I2C_IRQHandler (&I2C2_Resources);
}

/* I2C2 Driver Control Block */
extern ARM_DRIVER_I2C Driver_I2C2;
ARM_DRIVER_I2C Driver_I2C2 = {
    ARM_I2C_GetVersion,
    ARM_I2C_GetCapabilities,
    I2C2_Initialize,
    I2C2_Uninitialize,
    I2C2_PowerControl,
    I2C2_MasterTransmit,
    I2C2_MasterReceive,
    I2C2_SlaveTransmit,
    I2C2_SlaveReceive,
    I2C2_GetDataCount,
    I2C2_Control,
    I2C2_GetStatus
};
#endif /* RTE_I2C2 */


/* I2C3 Driver Instance */
#if (RTE_I2C3)

static i2c_info_t i2c3_info = {0};

/* I2C3 Driver Resources */
static i2c_resources_t I2C3_Resources =
{
    .reg_base       = (uint32_t)I2C3_BASE,
    .clk            = (uint32_t)I2C_PERIPHERAL_CLOCK,
    .info           = &i2c3_info,
    .irq_num        = (IRQn_Type)I2C3_IRQ_IRQn,
    .irq_priority   = (uint32_t)RTE_I2C3_IRQ_PRIORITY
};

static int32_t I2C3_Initialize (ARM_I2C_SignalEvent_t cb_event)
{
    return ARM_I2C_Initialize(cb_event, &I2C3_Resources);
}

static int32_t I2C3_Uninitialize(void)
{
    return ARM_I2C_Uninitialize(&I2C3_Resources);
}

static int32_t I2C3_PowerControl(ARM_POWER_STATE state)
{
	return ARM_I2C_PowerControl(state, &I2C3_Resources);
}

static int32_t I2C3_MasterTransmit(uint32_t addr, const uint8_t *data,
                                   uint32_t num, bool xfer_pending)
{
    return ARM_I2C_MasterTransmit(&I2C3_Resources, addr, data, num, xfer_pending);
}

static int32_t I2C3_MasterReceive(uint32_t addr, uint8_t *data,
                                  uint32_t num, bool xfer_pending)
{
    return (ARM_I2C_MasterReceive(&I2C3_Resources, addr, data, num, xfer_pending));
}

static int32_t I2C3_SlaveTransmit(const uint8_t *data, uint32_t num)
{
    return (ARM_I2C_SlaveTransmit(&I2C3_Resources, data, num));
}

static int32_t I2C3_SlaveReceive(uint8_t *data, uint32_t num)
{
    return (ARM_I2C_SlaveReceive(&I2C3_Resources, data, num));
}

static int32_t I2C3_GetDataCount(void)
{
    return (ARM_I2C_GetDataCount(&I2C3_Resources));
}

static int32_t I2C3_Control(uint32_t control, uint32_t arg)
{
    return (ARM_I2C_Control(&I2C3_Resources, control, arg));
}

static ARM_I2C_STATUS I2C3_GetStatus(void)
{
    return (ARM_I2C_GetStatus(&I2C3_Resources));
}

void I2C3_IRQHandler (void)
{
    ARM_I2C_IRQHandler (&I2C3_Resources);
}

/* I2C3 Driver Control Block */
extern ARM_DRIVER_I2C Driver_I2C3;
ARM_DRIVER_I2C Driver_I2C3 = {
    ARM_I2C_GetVersion,
    ARM_I2C_GetCapabilities,
    I2C3_Initialize,
    I2C3_Uninitialize,
    I2C3_PowerControl,
    I2C3_MasterTransmit,
    I2C3_MasterReceive,
    I2C3_SlaveTransmit,
    I2C3_SlaveReceive,
    I2C3_GetDataCount,
    I2C3_Control,
    I2C3_GetStatus
};
#endif /* RTE_I2C3 */

/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
