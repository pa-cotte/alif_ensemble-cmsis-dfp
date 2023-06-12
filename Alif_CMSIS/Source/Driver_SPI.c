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
 * @file     Driver_SPI.c
 * @author   Girish BN
 * @email    girish.bn@alifsemi.com
 * @version  V1.0.0
 * @date     20-04-2023
 * @brief    CMSIS-Driver for SPI.
 * @bug      None.
 * @Note     None
 ******************************************************************************/

#include "Driver_SPI.h"
#include "Driver_SPI_Private.h"
#include "sys_ctrl_spi.h"
#include "spi.h"

#ifdef RTE_Drivers_SPI_MultiSlave
#include "SPI_MultiSlave.h"
#include "SPI_MultiSlave_Config.h"
#endif

#if !((RTE_SPI0) || (RTE_SPI1) || (RTE_SPI2) || (RTE_SPI3))
#error "SPI is not enabled in the RTE_Device.h"
#endif

#if !defined(RTE_Drivers_SPI)
#error "SPI is not enabled in the RTE_Components.h"
#endif

#define ARM_SPI_DRV_VERSION    ARM_DRIVER_VERSION_MAJOR_MINOR(1, 0) /* driver version */

/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion = {
    ARM_SPI_API_VERSION,
    ARM_SPI_DRV_VERSION
};

/* Driver Capabilities */
static const ARM_SPI_CAPABILITIES DriverCapabilities = {
    1, /* Simplex Mode (Master and Slave) */
    1, /* TI Synchronous Serial Interface */
    1, /* Microwire Interface */
    0  /* Signal Mode Fault event: \ref ARM_SPI_EVENT_MODE_FAULT */
};

/**
 * @fn      ARM_DRIVER_VERSION ARM_SPI_GetVersion(void)
 * @brief   get spi version
 * @note    none
 * @param   none
 * @retval  driver version
 */
__STATIC_INLINE ARM_DRIVER_VERSION ARM_SPI_GetVersion(void)
{
    return DriverVersion;
}

/**
 * @fn      ARM_SPI_CAPABILITIES ARM_SPI_GetCapabilities(void)
 * @brief   get spi capabilities
 * @note    none
 * @param   none
 * @retval  driver capabilities
 */
__STATIC_INLINE ARM_SPI_CAPABILITIES ARM_SPI_GetCapabilities(void)
{
    return DriverCapabilities;
}

/**
 * @fn      int32_t ARM_SPI_Initialize(SPI_RESOURCES *SPI, ARM_SPI_SignalEvent_t cb_event).
 * @brief   Initialize the Spi for communication.
 * @note    none.
 * @param   SPI : Pointer to spi resources structure.
 * @param   cb_event : Pointer to user callback function.
 * @retval  \ref execution_status
 */
static int32_t ARM_SPI_Initialize(SPI_RESOURCES *SPI, ARM_SPI_SignalEvent_t cb_event)
{
    if (SPI->state.initialized == 1)
    {
        return ARM_DRIVER_OK;
    }

    if (cb_event == NULL)
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if ((SPI->tx_fifo_threshold > SPI_TX_FIFO_DEPTH) || (SPI->tx_fifo_start_level > SPI_TX_FIFO_DEPTH))
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if (SPI->rx_fifo_threshold > SPI_RX_FIFO_DEPTH)
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    /* Reset the transfer structure for this instance */
    SPI->transfer.tx_buff            = NULL;
    SPI->transfer.rx_buff            = NULL;

    SPI->transfer.tx_default_val     = 0;
    SPI->transfer.tx_default_enable  = false;

    SPI->transfer.total_cnt          = 0;
    SPI->transfer.tx_current_cnt     = 0;
    SPI->transfer.rx_current_cnt     = 0;
    SPI->transfer.status             = SPI_TRANSFER_STATUS_NONE;

    SPI->cb_event = cb_event;

    SPI->state.initialized = 1;

    return ARM_DRIVER_OK;
}

/**
 * @fn      int32_t ARM_SPI_Uninitialize(SPI_RESOURCES *SPI).
 * @brief   Un-Initialize the Spi.
 * @note    none.
 * @param   SPI : Pointer to spi resources structure.
 * @retval  \ref execution_status
 */
static int32_t ARM_SPI_Uninitialize(SPI_RESOURCES *SPI)
{
    if (SPI->state.initialized == 0)
    {
        return ARM_DRIVER_OK;
    }

    if (SPI->state.powered == 1)
    {
        return ARM_DRIVER_ERROR;
    }

    SPI->cb_event                   = NULL;

    SPI->transfer.tx_buff           = NULL;
    SPI->transfer.rx_buff           = NULL;
    SPI->transfer.tx_default_val    = 0;
    SPI->transfer.tx_default_enable = false;
    SPI->transfer.total_cnt         = 0;
    SPI->transfer.tx_current_cnt    = 0;
    SPI->transfer.rx_current_cnt    = 0;
    SPI->transfer.status            = SPI_TRANSFER_STATUS_NONE;

    SPI->state.initialized = 0;

    return ARM_DRIVER_OK;
}

/**
 * @fn      int32_t ARM_SPI_PowerControl(SPI_RESOURCES *SPI, ARM_POWER_STATE state).
 * @brief   Handles the spi power.
 * @note    none.
 * @param   SPI : Pointer to spi resources structure.
 * @param   state : power state.
 * @retval  \ref execution_status
 */
static int32_t ARM_SPI_PowerControl(SPI_RESOURCES *SPI, ARM_POWER_STATE state)
{
    if (SPI->state.initialized == 0)
    {
        return ARM_DRIVER_ERROR;
    }

    switch (state)
    {
        case ARM_POWER_OFF:
        {
            if (SPI->state.powered == 0)
            {
                return ARM_DRIVER_OK;
            }

            NVIC_ClearPendingIRQ(SPI->irq);
            NVIC_DisableIRQ(SPI->irq);

            if (SPI->drv_instance == LPSPI_INSTANCE)
            {
                disable_lpspi_clk();
            }
            else
            {
                /* reset to default value */
                ctrl_ss_in(SPI->drv_instance, SS_IN_IO_PIN);
            }

            SPI->state.powered = 0;

            break;
        }

        case ARM_POWER_FULL:
        {
            if( SPI->state.powered == 1)
            {
                return ARM_DRIVER_OK;
            }

            if (SPI->drv_instance == LPSPI_INSTANCE)
            {
                enable_lpspi_clk();
            }

            spi_mask_interrupts(SPI->regs);

            NVIC_ClearPendingIRQ(SPI->irq);
            NVIC_SetPriority(SPI->irq, SPI->irq_priority);
            NVIC_EnableIRQ(SPI->irq);

            spi_set_tx_threshold(SPI->regs, SPI->tx_fifo_threshold);
            spi_set_rx_threshold(SPI->regs, SPI->rx_fifo_threshold);

            if (SPI->drv_instance != LPSPI_INSTANCE)
            {
                spi_set_tx_fifo_start_level(SPI->regs, SPI->tx_fifo_start_level);
            }

            SPI->state.powered = 1;

            break;
        }

        case ARM_POWER_LOW:
        {
            return ARM_DRIVER_ERROR_UNSUPPORTED;
        }
    }
    return ARM_DRIVER_OK;
}

/**
 * @fn      int32_t ARM_SPI_Send(SPI_RESOURCES *SPI, const void *data, uint32_t num).
 * @brief   Used to send through spi.
 * @note    none.
 * @param   SPI : Pointer to spi resources structure.
 * @param   data : Pointer to the data to send.
 * @param   num : Number of data frames to send.
 * @retval  \ref execution_status
 */
static int32_t ARM_SPI_Send(SPI_RESOURCES *SPI, const void *data, uint32_t num)
{
    if (SPI->state.powered == 0)
    {
        return ARM_DRIVER_ERROR;
    }

    if ((data == NULL) && (SPI->transfer.tx_default_enable == false))
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if (num == 0)
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if (SPI->status.busy)
    {
        return ARM_DRIVER_ERROR_BUSY;
    }

    SPI->status.busy = 1;

    SPI->transfer.tx_buff        = (const uint8_t *) data;
    SPI->transfer.total_cnt      = num;
    SPI->transfer.tx_current_cnt = 0;
    SPI->transfer.status         = SPI_TRANSFER_STATUS_NONE;
    SPI->transfer.mode           = SPI_TMOD_TX;

    if (SPI->drv_instance == LPSPI_INSTANCE)
    {
        lpspi_send(SPI->regs);
    }
    else
    {
        spi_send(SPI->regs);
    }

    return ARM_DRIVER_OK;
}

/**
 * @fn      int32_t ARM_SPI_Receive(SPI_RESOURCES *SPI, void *data, uint32_t num).
 * @brief   Used to receive data through spi.
 * @note    none.
 * @param   SPI : Pointer to spi resources structure.
 * @param   data : Pointer to the data received.
 * @param   num : Number of data frames to receive.
 * @retval  \ref execution_status
 */
static int32_t ARM_SPI_Receive(SPI_RESOURCES *SPI, void *data, uint32_t num)
{
    if (SPI->state.powered == 0)
    {
        return ARM_DRIVER_ERROR;
    }

    if ((data == NULL) || (num == 0))
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if (SPI->status.busy)
    {
        return ARM_DRIVER_ERROR_BUSY;
    }

    SPI->status.busy = 1;

    SPI->transfer.rx_buff         = data;
    SPI->transfer.total_cnt       = num;
    SPI->transfer.rx_current_cnt  = 0;
    SPI->transfer.status          = SPI_TRANSFER_STATUS_NONE;
    SPI->transfer.mode            = SPI_TMOD_RX;

    if (SPI->drv_instance == LPSPI_INSTANCE)
    {
        lpspi_receive(SPI->regs, SPI->transfer.total_cnt);
    }
    else
    {
        spi_receive(SPI->regs, SPI->transfer.total_cnt);
    }

    return ARM_DRIVER_OK;
}

/**
 * @fn      int32_t ARM_SPI_Transfer(SPI_RESOURCES *SPI, const void *data_out, void *data_in, uint32_t num).
 * @brief   Used to Transfer and Receive data through spi.
 * @note    none.
 * @param   SPI : Pointer to spi resources structure.
 * @param   data_out : Pointer to the data send.
 * @param   data_in : Pointer to the data received.
 * @param   num : Number of data frames to transfer.
 * @retval  \ref execution_status
 */
static int32_t ARM_SPI_Transfer(SPI_RESOURCES *SPI, const void *data_out, void *data_in, uint32_t num)
{
    if (SPI->state.powered == 0)
    {
        return ARM_DRIVER_ERROR;
    }

    if ((data_out == NULL) && (SPI->transfer.tx_default_enable == false))
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if ((data_out == NULL) || (data_in == NULL) || (num == 0))
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if (SPI->status.busy)
    {
        return ARM_DRIVER_ERROR_BUSY;
    }

    SPI->status.busy = 1;

    SPI->transfer.tx_buff        = (const uint8_t *) data_out;
    SPI->transfer.rx_buff        = data_in;
    SPI->transfer.total_cnt      = num;
    SPI->transfer.tx_current_cnt = 0;
    SPI->transfer.rx_current_cnt = 0;
    SPI->transfer.status         = SPI_TRANSFER_STATUS_NONE;
    SPI->transfer.mode           = SPI_TMOD_TX_AND_RX;

    if (SPI->drv_instance == LPSPI_INSTANCE)
    {
        lpspi_transfer(SPI->regs, SPI->transfer.total_cnt);
    }
    else
    {
        spi_transfer(SPI->regs, SPI->transfer.total_cnt);
    }

    return ARM_DRIVER_OK;
}

/**
 * @fn      int32_t ARM_SPI_GetDataCount(SPI_RESOURCES *SPI).
 * @brief   Used to get the data count on spi data transferring modes.
 * @note    none.
 * @param   SPI : Pointer to spi resources structure.
 * @retval  \ref data count
 */
static uint32_t ARM_SPI_GetDataCount(SPI_RESOURCES *SPI)
{
    uint32_t count = 0;
    SPI_TMOD tmod;

    if (SPI->drv_instance == LPSPI_INSTANCE)
    {
        tmod = lpspi_get_tmod(SPI->regs);
    }
    else
    {
        tmod = spi_get_tmod(SPI->regs);
    }

    switch (tmod)
    {
        case SPI_TMOD_TX:
            count = SPI->transfer.tx_current_cnt;
            break;
        case SPI_TMOD_RX:
        case SPI_TMOD_TX_AND_RX:
            count = SPI->transfer.rx_current_cnt;
            break;
        case SPI_TMOD_EEPROM_READ:
        default:
            break;
    }

    return count;
}

/**
 * @fn      int32_t ARM_SPI_Control(SPI_RESOURCES *SPI, uint32_t control, uint32_t arg).
 * @brief   Used to configure spi.
 * @note    none.
 * @param   SPI : Pointer to spi resources structure.
 * @param   control : control code.
 * @param   arg : argument.
 * @retval  \ref execution_status
 */
static int32_t ARM_SPI_Control(SPI_RESOURCES *SPI, uint32_t control, uint32_t arg)
{
    int32_t ret = ARM_DRIVER_OK;
    uint32_t clk;

    if (SPI->state.powered == 0)
    {
        return ARM_DRIVER_ERROR;
    }

    switch (control & ARM_SPI_CONTROL_Msk)
    {
        case ARM_SPI_MODE_INACTIVE:
        {
            if(control == 0)
            {
                spi_disable(SPI->regs);
            }
            break;
        }

        /* SPI Master (Output on MOSI, Input on MISO); arg = Bus Speed in bps */
        case ARM_SPI_MODE_MASTER:
        {
            if (!(SPI->drv_instance == LPSPI_INSTANCE))
            {
                ctrl_ss_in(SPI->drv_instance, SS_IN_SS_IN_VAL);

                spi_mode_master(SPI->regs);
                clk = getSpiCoreClock(SPI->drv_instance);
                spi_set_bus_speed(SPI->regs, arg, clk);
            }

            break;
        }

        /* SPI Slave  (Output on MISO, Input on MOSI) arg = Bus speed in bps */
        case ARM_SPI_MODE_SLAVE:
        {
            if (SPI->drv_instance == LPSPI_INSTANCE)
            {
                return ARM_DRIVER_ERROR_UNSUPPORTED;
            }
            else
            {
                spi_mode_slave(SPI->regs);
            }
            break;
        }
        /* SPI Master (Output/Input on MOSI); arg = Bus Speed in bps */
        case ARM_SPI_MODE_MASTER_SIMPLEX:
        {
            //TODO: Implementation is pending
            break;
        }

        /* SPI Slave  (Output/Input on MISO) */
        case ARM_SPI_SET_BUS_SPEED:
        {
            clk = getSpiCoreClock(SPI->drv_instance);
            spi_set_bus_speed(SPI->regs, arg, clk);
            break;
        }

        /* Get Bus Speed in bps */
        case ARM_SPI_GET_BUS_SPEED:
        {
            clk = getSpiCoreClock(SPI->drv_instance);
            return (int32_t) spi_get_bus_speed(SPI->regs, clk);
        }

        /* Set the default transmission value */
        case ARM_SPI_SET_DEFAULT_TX_VALUE:
        {
            SPI->transfer.tx_default_val = arg;
            SPI->transfer.tx_default_enable = true;
            break;
        }

        /* Control the Slave Select signal */
        case ARM_SPI_CONTROL_SS:
        {
            if (arg == 1)
            {
                spi_control_ss(SPI->regs, SPI->slave_select, SPI_SS_STATE_ENABLE);
            }
            else if (arg == 0)
            {
                spi_control_ss(SPI->regs, SPI->slave_select, SPI_SS_STATE_DISABLE);
            }
            else
            {
                return ARM_DRIVER_ERROR_PARAMETER;
            }
            return ARM_DRIVER_OK;
        }

        /* Abort the current data transfer */
        case ARM_SPI_ABORT_TRANSFER:
        {
            spi_mask_interrupts(SPI->regs);

            SPI->transfer.tx_buff            = NULL;
            SPI->transfer.rx_buff            = NULL;
            SPI->transfer.tx_default_val     = 0;
            SPI->transfer.tx_default_enable  = false;
            SPI->transfer.total_cnt          = 0;
            SPI->transfer.tx_current_cnt     = 0;
            SPI->transfer.rx_current_cnt     = 0;
            SPI->status.busy                 = 0;

            spi_disable(SPI->regs);
            spi_enable(SPI->regs);
            break;
        }

        default:
        {
            ret = ARM_DRIVER_ERROR_UNSUPPORTED;
            break;
        }
    }

    switch (control & ARM_SPI_FRAME_FORMAT_Msk)
    {
        /* SPI Mode configuration */
        case ARM_SPI_CPOL0_CPHA0:
        {
            if (SPI->drv_instance == LPSPI_INSTANCE)
            {
                lpspi_set_mode(SPI->regs, SPI_MODE_0);
            }
            else
            {
                spi_set_mode(SPI->regs, SPI_MODE_0);
            }
            break;
        }
        case ARM_SPI_CPOL0_CPHA1:
        {
            if (SPI->drv_instance == LPSPI_INSTANCE)
            {
                lpspi_set_mode(SPI->regs, SPI_MODE_1);
            }
            else
            {
                spi_set_mode(SPI->regs, SPI_MODE_1);
            }
            break;
        }
        case ARM_SPI_CPOL1_CPHA0:
        {
            if (SPI->drv_instance == LPSPI_INSTANCE)
            {
                lpspi_set_mode(SPI->regs, SPI_MODE_2);
            }
            else
            {
                spi_set_mode(SPI->regs, SPI_MODE_2);
            }
            break;
        }
        case ARM_SPI_CPOL1_CPHA1:
        {
            if (SPI->drv_instance == LPSPI_INSTANCE)
            {
                lpspi_set_mode(SPI->regs, SPI_MODE_3);
            }
            else
            {
                spi_set_mode(SPI->regs, SPI_MODE_3);
            }
            break;
        }

        /* Texas Instruments Frame Format */
        case ARM_SPI_TI_SSI:
        {
            if (SPI->drv_instance == LPSPI_INSTANCE)
            {
                lpspi_set_protocol(SPI->regs, SPI_PROTO_SSP);
            }
            else
            {
                spi_set_protocol(SPI->regs, SPI_PROTO_SSP);
            }
            break;
        }

        /* National Microwire Frame Format */
        case ARM_SPI_MICROWIRE:
        {
            if (SPI->drv_instance == LPSPI_INSTANCE)
            {
                lpspi_set_protocol(SPI->regs, SPI_PROTO_MICROWIRE);
            }
            else
            {
                spi_set_protocol(SPI->regs, SPI_PROTO_MICROWIRE);
            }
            break;
        }

        default:
        {
            ret = ARM_DRIVER_ERROR_UNSUPPORTED;
            break;
        }
    }

    /* Configure frame size */
    if (control & ARM_SPI_DATA_BITS_Msk)
    {
        SPI->transfer.frame_size = ((control & ARM_SPI_DATA_BITS_Msk) >> ARM_SPI_DATA_BITS_Pos);

        if (SPI->drv_instance == LPSPI_INSTANCE)
        {
            lpspi_set_dfs(SPI->regs, SPI->transfer.frame_size);
        }
        else
        {
            spi_set_dfs(SPI->regs, SPI->transfer.frame_size);
        }
    }

    switch (control & ARM_SPI_BIT_ORDER_Msk)
    {
        /* SPI Bit order from MSB to LSB (default) */
        case ARM_SPI_MSB_LSB:
        {
            break;
        }
        /* SPI Bit order from LSB to MSB */
        case ARM_SPI_LSB_MSB:
        {
            return ARM_DRIVER_ERROR_UNSUPPORTED;
        }
    }

    switch (control & ARM_SPI_SS_MASTER_MODE_Msk)
    {
        /* SPI Slave Select when Master: Not used (default) */
        case ARM_SPI_SS_MASTER_UNUSED:
        {
            break;
        }

        /* SPI Slave Select when Master: Software controlled */
        case ARM_SPI_SS_MASTER_SW:
        {
            if((control & ARM_SPI_CONTROL_Msk) == ARM_SPI_MODE_MASTER )
            {
                SPI->sw_slave_select = 1;
            }
            break;
        }

        /* SPI Slave Select when Master: Hardware controlled Output */
        case ARM_SPI_SS_MASTER_HW_OUTPUT:
        {
            /* This is the default state in the IP, No need to configure */
            break;
        }

        /* SPI Slave Select when Master: Hardware monitored Input */
        case ARM_SPI_SS_MASTER_HW_INPUT:
        {
            //TODO: Need to Implement
            break;
        }
    }

    switch (control & ARM_SPI_SS_SLAVE_MODE_Msk)
    {
        if (SPI->drv_instance == LPSPI_INSTANCE)
        {
            return ARM_DRIVER_ERROR_UNSUPPORTED;
        }
        else
        {
            /* SPI Slave Select when Slave: Hardware monitored (default) */
            case ARM_SPI_SS_SLAVE_HW:
            {
                break;
            }

            /* SPI Slave Select when Slave: Software controlled */
            case ARM_SPI_SS_SLAVE_SW:
            {
                return ARM_DRIVER_ERROR_UNSUPPORTED;
            }
        }
    }
    return ret;
}

/**
 * @fn      int32_t ARM_SPI_Control_SlaveSelect(SPI_RESOURCES *SPI, uint32_t device, uint32_t ss_state).
 * @brief   Used to configure spi.
 * @note    none.
 * @param   SPI : Pointer to spi resources structure.
 * @param   device : each bit represent chip selection line.
 * @param   ss_state : Set to active or inactive.
 * @retval  \ref execution_status
 */
#ifdef RTE_Drivers_SPI_MultiSlave
static int32_t ARM_SPI_Control_SlaveSelect(SPI_RESOURCES *SPI, uint32_t device, uint32_t ss_state)
{
    if (SPI->state.powered == 0)
    {
        return ARM_DRIVER_ERROR;
    }

    if ((device & SPI_SLAVE_SELECT_PIN_MASK) || (ss_state > ARM_SPI_SS_ACTIVE))
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    if (SPI->status.busy == 1)
    {
        return ARM_DRIVER_ERROR_BUSY;
    }

    spi_control_ss(SPI->regs, device, ss_state);

    return ARM_DRIVER_OK;
}
#endif
/**
 * @fn      ARM_SPI_STATUS ARM_SPI_GetStatus(SPI_RESOURCES *SPI)
 * @brief   Used to get spi status.
 * @note    none.
 * @param   SPI : Pointer to spi resources structure.
 * @retval  \ref spi driver status.
 */
__STATIC_INLINE ARM_SPI_STATUS ARM_SPI_GetStatus(SPI_RESOURCES *SPI)
{
    return SPI->status;
}

/* SPI0 driver instance */
#if RTE_SPI0
static SPI_RESOURCES SPI0_RES = {
    .regs                   = (SPI_Type*) SPI0_BASE,
    .cb_event               = NULL,
    .irq_priority           = RTE_SPI0_IRQ_PRIORITY,
    .drv_instance           = SPI_INSTANCE_0,
    .slave_select           = RTE_SPI0_CHIP_SELECTION_PIN,
    .spi_frf                = RTE_SPI0_SPI_FRAME_FORMAT,
    .tx_fifo_threshold      = RTE_SPI0_TX_FIFO_THRESHOLD,
    .tx_fifo_start_level    = RTE_SPI0_TX_FIFO_LEVEL_TO_START_TRANSFER,
    .rx_fifo_threshold      = RTE_SPI0_RX_FIFO_THRESHOLD,
    .irq                    = SPI0_IRQ_IRQn
};

void SPI0_IRQHandler(void)
{
    spi_transfer_t *transfer = &(SPI0_RES.transfer);

    spi_irq_handler(SPI0_RES.regs, transfer);

    if (transfer->status == SPI_TRANSFER_STATUS_COMPLETE)
    {
        transfer->status = SPI_TRANSFER_STATUS_NONE;
        SPI0_RES.status.busy = 0;
        SPI0_RES.cb_event(ARM_SPI_EVENT_TRANSFER_COMPLETE);
    }

    if (transfer->status == SPI_TRANSFER_STATUS_OVERFLOW)
    {
        transfer->status = SPI_TRANSFER_STATUS_NONE;
        SPI0_RES.status.data_lost = 1;
        SPI0_RES.status.busy = 0;
        SPI0_RES.cb_event(ARM_SPI_EVENT_DATA_LOST);
    }
}

static int32_t ARM_SPI0_Initialize(ARM_SPI_SignalEvent_t cb_event)
{
    return ARM_SPI_Initialize(&SPI0_RES, cb_event);
}

static int32_t ARM_SPI0_Uninitialize(void)
{
    return ARM_SPI_Uninitialize(&SPI0_RES);
}

static int32_t ARM_SPI0_PowerControl(ARM_POWER_STATE state)
{
    return ARM_SPI_PowerControl(&SPI0_RES, state);
}

static int32_t ARM_SPI0_Send(const void *data, uint32_t num)
{
    return ARM_SPI_Send(&SPI0_RES, data, num);
}

static int32_t ARM_SPI0_Receive(void *data, uint32_t num)
{
    return ARM_SPI_Receive(&SPI0_RES, data, num);
}

static int32_t ARM_SPI0_Transfer(const void *data_out, void *data_in, uint32_t num)
{
    return ARM_SPI_Transfer(&SPI0_RES, data_out, data_in, num);
}

static uint32_t ARM_SPI0_GetDataCount(void)
{
    return ARM_SPI_GetDataCount(&SPI0_RES);
}

static int32_t ARM_SPI0_Control(uint32_t control, uint32_t arg)
{
    return ARM_SPI_Control(&SPI0_RES, control, arg);
}

static ARM_SPI_STATUS ARM_SPI0_GetStatus(void)
{
    return ARM_SPI_GetStatus(&SPI0_RES);
}

#ifdef RTE_Drivers_SPI_MultiSlave
    #if SPI_DRIVER == 0
    static int32_t ARM_SPI0_Control_SlaveSelect(uint32_t device, uint32_t ss_state)
    {
        return ARM_SPI_Control_SlaveSelect(&SPI0_RES, device, ss_state);
    }
    #endif
#endif

extern ARM_DRIVER_SPI Driver_SPI0;
ARM_DRIVER_SPI Driver_SPI0 = {
    ARM_SPI_GetVersion,
    ARM_SPI_GetCapabilities,
    ARM_SPI0_Initialize,
    ARM_SPI0_Uninitialize,
    ARM_SPI0_PowerControl,
    ARM_SPI0_Send,
    ARM_SPI0_Receive,
    ARM_SPI0_Transfer,
    ARM_SPI0_GetDataCount,
    ARM_SPI0_Control,
    ARM_SPI0_GetStatus
};
#endif /* RTE_SPI0 */

/* SPI1 driver instance */
#if RTE_SPI1
static SPI_RESOURCES SPI1_RES = {
    .regs                   = (SPI_Type*) SPI1_BASE,
    .cb_event               = NULL,
    .irq_priority           = RTE_SPI1_IRQ_PRIORITY,
    .drv_instance           = SPI_INSTANCE_1,
    .slave_select           = RTE_SPI1_CHIP_SELECTION_PIN,
    .spi_frf                = RTE_SPI1_SPI_FRAME_FORMAT,
    .tx_fifo_threshold      = RTE_SPI1_TX_FIFO_THRESHOLD,
    .tx_fifo_start_level    = RTE_SPI1_TX_FIFO_LEVEL_TO_START_TRANSFER,
    .rx_fifo_threshold      = RTE_SPI1_RX_FIFO_THRESHOLD,
    .irq                    = SPI1_IRQ_IRQn
};

void SPI1_IRQHandler(void)
{
    spi_transfer_t *transfer = &(SPI1_RES.transfer);

    spi_irq_handler(SPI1_RES.regs, transfer);

    if (transfer->status == SPI_TRANSFER_STATUS_COMPLETE)
    {
        transfer->status = SPI_TRANSFER_STATUS_NONE;
        SPI1_RES.status.busy = 0;
        SPI1_RES.cb_event(ARM_SPI_EVENT_TRANSFER_COMPLETE);
    }

    if (transfer->status == SPI_TRANSFER_STATUS_OVERFLOW)
    {
        transfer->status = SPI_TRANSFER_STATUS_NONE;
        SPI1_RES.status.data_lost = 1;
        SPI1_RES.cb_event(ARM_SPI_EVENT_DATA_LOST);
    }
}

static int32_t ARM_SPI1_Initialize(ARM_SPI_SignalEvent_t cb_event)
{
    return ARM_SPI_Initialize(&SPI1_RES, cb_event);
}

static int32_t ARM_SPI1_Uninitialize(void)
{
    return ARM_SPI_Uninitialize(&SPI1_RES);
}

static int32_t ARM_SPI1_PowerControl(ARM_POWER_STATE state)
{
    return ARM_SPI_PowerControl(&SPI1_RES, state);
}

static int32_t ARM_SPI1_Send(const void *data, uint32_t num)
{
    return ARM_SPI_Send(&SPI1_RES, data, num);
}

static int32_t ARM_SPI1_Receive(void *data, uint32_t num)
{
    return ARM_SPI_Receive(&SPI1_RES, data, num);
}

static int32_t ARM_SPI1_Transfer(const void *data_out, void *data_in, uint32_t num)
{
    return ARM_SPI_Transfer(&SPI1_RES, data_out, data_in, num);
}

static uint32_t ARM_SPI1_GetDataCount(void)
{
    return ARM_SPI_GetDataCount(&SPI1_RES);
}

static int32_t ARM_SPI1_Control(uint32_t control, uint32_t arg)
{
    return ARM_SPI_Control(&SPI1_RES, control, arg);
}

static ARM_SPI_STATUS ARM_SPI1_GetStatus(void)
{
    return ARM_SPI_GetStatus(&SPI1_RES);
}

#ifdef RTE_Drivers_SPI_MultiSlave
    #if SPI_DRIVER == 1
    static int32_t ARM_SPI1_Control_SlaveSelect(uint32_t device, uint32_t ss_state)
    {
        return ARM_SPI_Control_SlaveSelect(&SPI1_RES, device, ss_state);
    }
    #endif
#endif

extern ARM_DRIVER_SPI Driver_SPI1;
ARM_DRIVER_SPI Driver_SPI1 = {
    ARM_SPI_GetVersion,
    ARM_SPI_GetCapabilities,
    ARM_SPI1_Initialize,
    ARM_SPI1_Uninitialize,
    ARM_SPI1_PowerControl,
    ARM_SPI1_Send,
    ARM_SPI1_Receive,
    ARM_SPI1_Transfer,
    ARM_SPI1_GetDataCount,
    ARM_SPI1_Control,
    ARM_SPI1_GetStatus
};
#endif /* RTE_SPI1 */

/* SPI2 driver instance */
#if RTE_SPI2
static SPI_RESOURCES SPI2_RES = {
    .regs                   = (SPI_Type*) SPI2_BASE,
    .cb_event               = NULL,
    .irq_priority           = RTE_SPI2_IRQ_PRIORITY,
    .drv_instance           = SPI_INSTANCE_2,
    .slave_select           = RTE_SPI2_CHIP_SELECTION_PIN,
    .spi_frf                = RTE_SPI2_SPI_FRAME_FORMAT,
    .tx_fifo_threshold      = RTE_SPI2_TX_FIFO_THRESHOLD,
    .tx_fifo_start_level    = RTE_SPI2_TX_FIFO_LEVEL_TO_START_TRANSFER,
    .rx_fifo_threshold      = RTE_SPI2_RX_FIFO_THRESHOLD,
    .irq                    = SPI2_IRQ_IRQn
};

void SPI2_IRQHandler(void)
{
    spi_transfer_t *transfer = &(SPI2_RES.transfer);

    spi_irq_handler(SPI2_RES.regs, transfer);

    if (transfer->status == SPI_TRANSFER_STATUS_COMPLETE)
    {
        transfer->status = SPI_TRANSFER_STATUS_NONE;
        SPI2_RES.status.busy = 0;
        SPI2_RES.cb_event(ARM_SPI_EVENT_TRANSFER_COMPLETE);
    }

    if (transfer->status == SPI_TRANSFER_STATUS_OVERFLOW)
    {
        transfer->status = SPI_TRANSFER_STATUS_NONE;
        SPI2_RES.status.data_lost = 1;
        SPI2_RES.cb_event(ARM_SPI_EVENT_DATA_LOST);
    }
}

static int32_t ARM_SPI2_Initialize(ARM_SPI_SignalEvent_t cb_event)
{
    return ARM_SPI_Initialize(&SPI2_RES, cb_event);
}

static int32_t ARM_SPI2_Uninitialize(void)
{
    return ARM_SPI_Uninitialize(&SPI2_RES);
}

static int32_t ARM_SPI2_PowerControl(ARM_POWER_STATE state)
{
    return ARM_SPI_PowerControl(&SPI2_RES, state);
}

static int32_t ARM_SPI2_Send(const void *data, uint32_t num)
{
    return ARM_SPI_Send(&SPI2_RES, data, num);
}

static int32_t ARM_SPI2_Receive(void *data, uint32_t num)
{
    return ARM_SPI_Receive(&SPI2_RES, data, num);
}

static int32_t ARM_SPI2_Transfer(const void *data_out, void *data_in, uint32_t num)
{
    return ARM_SPI_Transfer(&SPI2_RES, data_out, data_in, num);
}

static uint32_t ARM_SPI2_GetDataCount(void)
{
    return ARM_SPI_GetDataCount(&SPI2_RES);
}

static int32_t ARM_SPI2_Control(uint32_t control, uint32_t arg)
{
    return ARM_SPI_Control(&SPI2_RES, control, arg);
}

static ARM_SPI_STATUS ARM_SPI2_GetStatus(void)
{
    return ARM_SPI_GetStatus(&SPI2_RES);
}

#ifdef RTE_Drivers_SPI_MultiSlave
    #if SPI_DRIVER == 2
    int32_t ARM_SPI2_Control_SlaveSelect(uint32_t device, uint32_t ss_state)
    {
        return ARM_SPI_Control_SlaveSelect(&SPI2_RES, device, ss_state);
    }
    #endif
#endif

extern ARM_DRIVER_SPI Driver_SPI2;
ARM_DRIVER_SPI Driver_SPI2 = {
    ARM_SPI_GetVersion,
    ARM_SPI_GetCapabilities,
    ARM_SPI2_Initialize,
    ARM_SPI2_Uninitialize,
    ARM_SPI2_PowerControl,
    ARM_SPI2_Send,
    ARM_SPI2_Receive,
    ARM_SPI2_Transfer,
    ARM_SPI2_GetDataCount,
    ARM_SPI2_Control,
    ARM_SPI2_GetStatus
};
#endif /* RTE_SPI2 */

/* SPI3 driver instance */
#if RTE_SPI3
static SPI_RESOURCES SPI3_RES = {
    .regs                   = (SPI_Type*) SPI3_BASE,
    .cb_event               = NULL,
    .irq_priority           = RTE_SPI3_IRQ_PRIORITY,
    .drv_instance           = SPI_INSTANCE_3,
    .slave_select           = RTE_SPI3_CHIP_SELECTION_PIN,
    .spi_frf                = RTE_SPI3_SPI_FRAME_FORMAT,
    .tx_fifo_threshold      = RTE_SPI3_TX_FIFO_THRESHOLD,
    .tx_fifo_start_level    = RTE_SPI3_TX_FIFO_LEVEL_TO_START_TRANSFER,
    .rx_fifo_threshold      = RTE_SPI3_RX_FIFO_THRESHOLD,
    .irq                    = SPI3_IRQ_IRQn
};

void SPI3_IRQHandler(void)
{
    spi_transfer_t *transfer = &(SPI3_RES.transfer);

    spi_irq_handler(SPI3_RES.regs, transfer);

    if (transfer->status == SPI_TRANSFER_STATUS_COMPLETE)
    {
        transfer->status = SPI_TRANSFER_STATUS_NONE;
        SPI3_RES.status.busy = 0;
        SPI3_RES.cb_event(ARM_SPI_EVENT_TRANSFER_COMPLETE);
    }

    if (transfer->status == SPI_TRANSFER_STATUS_OVERFLOW)
    {
        transfer->status = SPI_TRANSFER_STATUS_NONE;
        SPI3_RES.status.data_lost = 1;
        SPI3_RES.cb_event(ARM_SPI_EVENT_DATA_LOST);
    }
}

static int32_t ARM_SPI3_Initialize(ARM_SPI_SignalEvent_t cb_event)
{
    return ARM_SPI_Initialize(&SPI3_RES, cb_event);
}

static int32_t ARM_SPI3_Uninitialize(void)
{
    return ARM_SPI_Uninitialize(&SPI3_RES);
}

static int32_t ARM_SPI3_PowerControl(ARM_POWER_STATE state)
{
    return ARM_SPI_PowerControl(&SPI3_RES, state);
}

static int32_t ARM_SPI3_Send(const void *data, uint32_t num)
{
    return ARM_SPI_Send(&SPI3_RES, data, num);
}

static int32_t ARM_SPI3_Receive(void *data, uint32_t num)
{
    return ARM_SPI_Receive(&SPI3_RES, data, num);
}

static int32_t ARM_SPI3_Transfer(const void *data_out, void *data_in, uint32_t num)
{
    return ARM_SPI_Transfer(&SPI3_RES, data_out, data_in, num);
}

static uint32_t ARM_SPI3_GetDataCount(void)
{
    return ARM_SPI_GetDataCount(&SPI3_RES);
}

static int32_t ARM_SPI3_Control(uint32_t control, uint32_t arg)
{
    return ARM_SPI_Control(&SPI3_RES, control, arg);
}

static ARM_SPI_STATUS ARM_SPI3_GetStatus(void)
{
    return ARM_SPI_GetStatus(&SPI3_RES);
}

#ifdef RTE_Drivers_SPI_MultiSlave
    #if SPI_DRIVER == 3
    static int32_t ARM_SPI3_Control_SlaveSelect(uint32_t device, uint32_t ss_state)
    {
        return ARM_SPI_Control_SlaveSelect(&SPI3_RES, device, ss_state);
    }
    #endif
#endif

extern ARM_DRIVER_SPI Driver_SPI3;
ARM_DRIVER_SPI Driver_SPI3 = {
    ARM_SPI_GetVersion,
    ARM_SPI_GetCapabilities,
    ARM_SPI3_Initialize,
    ARM_SPI3_Uninitialize,
    ARM_SPI3_PowerControl,
    ARM_SPI3_Send,
    ARM_SPI3_Receive,
    ARM_SPI3_Transfer,
    ARM_SPI3_GetDataCount,
    ARM_SPI3_Control,
    ARM_SPI3_GetStatus
};

#ifdef RTE_Drivers_SPI_MultiSlave
void SPI_Control_SlaveSelect(uint32_t device, uint32_t ss_state)
{
    #if SPI_DRIVER == 0
       ARM_SPI0_Control_SlaveSelect(device, ss_state);
    #elif SPI_DRIVER == 1
       ARM_SPI1_Control_SlaveSelect(device, ss_state);
    #elif SPI_DRIVER == 2
       ARM_SPI2_Control_SlaveSelect(device, ss_state);
    #elif SPI_DRIVER == 3
       ARM_SPI3_Control_SlaveSelect(device, ss_state);
    #endif
}
#endif
#endif /* RTE_SPI3 */

/* LPSPI driver instance */
#if RTE_LPSPI
static SPI_RESOURCES LPSPI_RES = {
    .regs                   = (SPI_Type*) LPSPI_BASE,
    .cb_event               = NULL,
    .irq_priority           = RTE_LPSPI_IRQ_PRIORITY,
    .drv_instance           = LPSPI_INSTANCE,
    .slave_select           = RTE_LPSPI_CHIP_SELECTION_PIN,
    .spi_frf                = RTE_LPSPI_SPI_FRAME_FORMAT,
    .tx_fifo_threshold      = RTE_LPSPI_TX_FIFO_THRESHOLD,
    .tx_fifo_start_level    = RTE_LPSPI_TX_FIFO_LEVEL_TO_START_TRANSFER,
    .rx_fifo_threshold      = RTE_LPSPI_RX_FIFO_THRESHOLD,
    .irq                    = LPSPI_IRQ_IRQn
};

extern void LPSPI_IRQHandler(void);
void LPSPI_IRQHandler(void)
{
    spi_transfer_t *transfer = &(LPSPI_RES.transfer);

    spi_irq_handler(LPSPI_RES.regs, transfer);

    if (transfer->status == SPI_TRANSFER_STATUS_COMPLETE)
    {
        transfer->status = SPI_TRANSFER_STATUS_NONE;
        LPSPI_RES.status.busy = 0;
        LPSPI_RES.cb_event(ARM_SPI_EVENT_TRANSFER_COMPLETE);
    }

    if (transfer->status == SPI_TRANSFER_STATUS_OVERFLOW)
    {
        transfer->status = SPI_TRANSFER_STATUS_NONE;
        LPSPI_RES.status.data_lost = 1;
        LPSPI_RES.status.busy = 0;
        LPSPI_RES.cb_event(ARM_SPI_EVENT_DATA_LOST);
    }
}

static int32_t ARM_LPSPI_Initialize(ARM_SPI_SignalEvent_t cb_event)
{
    return ARM_SPI_Initialize(&LPSPI_RES, cb_event);
}

static int32_t ARM_LPSPI_Uninitialize(void)
{
    return ARM_SPI_Uninitialize(&LPSPI_RES);
}

static int32_t ARM_LPSPI_PowerControl(ARM_POWER_STATE state)
{
    return ARM_SPI_PowerControl(&LPSPI_RES, state);
}

static int32_t ARM_LPSPI_Send(const void *data, uint32_t num)
{
    return ARM_SPI_Send(&LPSPI_RES, data, num);
}

static int32_t ARM_LPSPI_Receive(void *data, uint32_t num)
{
    return ARM_SPI_Receive(&LPSPI_RES, data, num);
}

static int32_t ARM_LPSPI_Transfer(const void *data_out, void *data_in, uint32_t num)
{
    return ARM_SPI_Transfer(&LPSPI_RES, data_out, data_in, num);
}

static uint32_t ARM_LPSPI_GetDataCount(void)
{
    return ARM_SPI_GetDataCount(&LPSPI_RES);
}

static int32_t ARM_LPSPI_Control(uint32_t control, uint32_t arg)
{
    return ARM_SPI_Control(&LPSPI_RES, control, arg);
}

static ARM_SPI_STATUS ARM_LPSPI_GetStatus(void)
{
    return ARM_SPI_GetStatus(&LPSPI_RES);
}

#ifdef RTE_Drivers_SPI_MultiSlave
    #if SPI_DRIVER == 0
    static int32_t ARM_LPSPI_Control_SlaveSelect(uint32_t device, uint32_t ss_state)
    {
        return ARM_SPI_Control_SlaveSelect(&LPSPI_RES, device, ss_state);
    }
    #endif
#endif

extern ARM_DRIVER_SPI Driver_SPILP;
ARM_DRIVER_SPI Driver_SPILP = {
    ARM_SPI_GetVersion,
    ARM_SPI_GetCapabilities,
    ARM_LPSPI_Initialize,
    ARM_LPSPI_Uninitialize,
    ARM_LPSPI_PowerControl,
    ARM_LPSPI_Send,
    ARM_LPSPI_Receive,
    ARM_LPSPI_Transfer,
    ARM_LPSPI_GetDataCount,
    ARM_LPSPI_Control,
    ARM_LPSPI_GetStatus
};
#endif /* RTE_LPSPI */
