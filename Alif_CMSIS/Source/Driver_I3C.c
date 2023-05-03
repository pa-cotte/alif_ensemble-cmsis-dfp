/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 */

/* System Includes */
#include <string.h>
#include "RTE_Device.h"
#include "RTE_Components.h"
#include CMSIS_device_header

/* Project Includes */
#include "Driver_I3C.h"
#include "Driver_I3C_Private.h"
#include "i3c.h"
#include "sys_ctrl_i3c.h"

#if !(RTE_I3C)
#error "I3C is not enabled in the RTE_Device.h"
#endif

#if (defined(RTE_Drivers_I3C) && !RTE_I3C)
#error "I3C not configured in RTE_Device.h!"
#endif

#define ARM_I3C_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1,0) /* driver version */

/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion =
{
  ARM_I3C_API_VERSION,
  ARM_I3C_DRV_VERSION
};

/* Driver Capabilities */
static const ARM_I3C_CAPABILITIES DriverCapabilities =
{
  1,   /* Supports legacy i2c device */
  0    /* Reserved (must be zero)    */
};


/**
  \fn           int I3cMasterGetAddrPos(I3C_RESOURCES *i3c, uint8_t addr)
  \brief        Get already assigned slave address position from DAT(Device Address Table).
                For i3c, dynamic address and for i2c, static address is used for communication.
  \param[in]    i3c     : Pointer to i3c resources structure
  \param[in]    addr    : Assigned Slave Address;
                           Dynamic Address for i3c, Static Address for i2c slave device
  \return       Assigned slave address position from DAT(Device Address Table) index OR
                ARM_DRIVER_ERROR in case slave is not already assigned.
*/
static int I3cMasterGetAddrPos(I3C_RESOURCES *i3c, uint8_t addr)
{
  int pos;

  for (pos = 0; pos < i3c->maxdevs; pos++)
  {
    if (addr == i3c->addrs[pos])
      return pos;
  }

  return ARM_DRIVER_ERROR;
}

/**
  \fn           int I3cMasterGetFreePos(I3C_RESOURCES *i3c)
  \brief        Get free position from DAT(Device Address Table)
  \param[in]    i3c     : Pointer to i3c resources structure
  \return       Free position from DAT OR
                ARM_DRIVER_ERROR in case DAT is Full.
                Maximum 8 Slave Devices are supported (\ref register DEVICE_ADDR_TABLE_POINTER)
*/
static int I3cMasterGetFreePos(I3C_RESOURCES *i3c)
{
  int i;

  if (!(i3c->freepos & GENMASK(i3c->maxdevs - 1, 0)))
    return ARM_DRIVER_ERROR;

  for (i = 0; i < i3c->maxdevs; i++)
  {
    if (i3c->freepos & (1 << i))
      return i;
  }

  return ARM_DRIVER_ERROR;
}

/**
  \fn           ARM_DRIVER_VERSION I3C_GetVersion(void)
  \brief        Get i3c driver version
  \return       i3c driver version
*/
static ARM_DRIVER_VERSION I3C_GetVersion(void)
{
  return DriverVersion;
}

/**
  \fn           ARM_I3C_CAPABILITIES I3C_GetCapabilities(void)
  \brief        Get i3c driver capabilities
  \return       i3c driver capabilities
*/
static ARM_I3C_CAPABILITIES I3C_GetCapabilities(void)
{
  return DriverCapabilities;
}

/**
  \fn           int I3Cx_MasterSendCommand(I3C_RESOURCES *i3c, I3C_CMD *ccc)
  \brief        Send an I3C command to the slave
  \param[in]    i3c      : Pointer to i3c resources structure
  \param[in]    ccc      : Pointer to i3c command structure
  \return       \ref execution_status
*/
static int I3Cx_MasterSendCommand(I3C_RESOURCES *i3c, I3C_CMD *ccc)
{
  int32_t index;

  if (i3c->state.powered == 0U)
    return ARM_DRIVER_ERROR;

  if (i3c->status.busy)
    return ARM_DRIVER_ERROR_BUSY;

  if (!ccc)
    return ARM_DRIVER_ERROR_PARAMETER;

  if (ccc->cmd_id == I3C_CCC_ENTDAA)
    return ARM_DRIVER_ERROR_PARAMETER;

  index = I3cMasterGetAddrPos(i3c, ccc->addr);
  if (index < 0)
    return ARM_DRIVER_ERROR;

  i3c->status.busy = 1;

  if (ccc->rw)
  {
    i3c->xfer.tx_buf = NULL;
    i3c->xfer.tx_len = 0;
    i3c->xfer.rx_buf = ccc->data;
    i3c->xfer.rx_len = ccc->len;

    i3c_ccc_get(i3c->regs, &(i3c->xfer), index, ccc->cmd_id, ccc->len);
  }
  else
  {
    i3c->xfer.rx_buf = NULL;
    i3c->xfer.rx_len = 0;
    i3c->xfer.tx_buf = ccc->data;
    i3c->xfer.tx_len = ccc->len;

    i3c_ccc_set(i3c->regs, &(i3c->xfer), index, ccc->cmd_id, ccc->len);
  }

  return ARM_DRIVER_OK;
}

/**
  \fn           int I3Cx_MasterTransmit(I3C_RESOURCES *i3c,  uint8_t  addr,
                                        const uint8_t *data, uint16_t len)
  \brief        Write data to the slave
  \param[in]    i3c      : Pointer to i3c resources structure
  \param[in]    addr     : Assigned Slave Address;
                            Dynamic Address for i3c, Static Address for i2c slave device
  \param[in]    data     : Pointer to buffer with data which needs to be transmit to slave
  \param[in]    len      : Number of bytes needs to be transmit
  \return       \ref execution_status
*/
static int I3Cx_MasterTransmit(I3C_RESOURCES *i3c,  uint8_t  addr,
                               const uint8_t *data, uint16_t len)
{
  int32_t index;

  if (i3c->state.powered == 0U)
    return ARM_DRIVER_ERROR;

  if (i3c->state.master_enabled == 0U)
      return ARM_DRIVER_ERROR;

  if (!data || !len)
    return ARM_DRIVER_ERROR_PARAMETER;

  if (i3c->status.busy)
    return ARM_DRIVER_ERROR_BUSY;

  index = I3cMasterGetAddrPos(i3c, addr);
  if (index < 0)
    return ARM_DRIVER_ERROR_PARAMETER;

  i3c->status.busy = 1;

  i3c->xfer.rx_buf = NULL;
  i3c->xfer.rx_len = 0;
  i3c->xfer.tx_buf = data;
  i3c->xfer.tx_len = len;

  i3c_master_tx(i3c->regs, &(i3c->xfer), index, len);

  return ARM_DRIVER_OK;
}

/**
  \fn           int I3Cx_MasterReceive(I3C_RESOURCES *i3c,  uint8_t  addr,
                                             uint8_t *data, uint16_t len)
  \brief        Read data from the slave
  \param[in]    i3c      : Pointer to i3c resources structure
  \param[in]    addr     : Assigned Slave Address;
                            Dynamic Address for i3c, Static Address for i2c slave device
  \param[in]    data     : Pointer to buffer for data to receive from slave
  \param[in]    len      : Number of bytes needs to be receive
  \return       \ref execution_status
*/
static int I3Cx_MasterReceive(I3C_RESOURCES *i3c,  uint8_t  addr,
                                    uint8_t *data, uint16_t len)
{
  int32_t index;

  if (i3c->state.powered == 0U)
    return ARM_DRIVER_ERROR;

  if (i3c->state.master_enabled == 0U)
      return ARM_DRIVER_ERROR;

  if (!data || !len)
    return ARM_DRIVER_ERROR_PARAMETER;

  if (i3c->status.busy)
    return ARM_DRIVER_ERROR_BUSY;

  index = I3cMasterGetAddrPos(i3c, addr);
  if (index < 0)
    return ARM_DRIVER_ERROR_PARAMETER;

  i3c->status.busy = 1;

  i3c->xfer.rx_buf = data;
  i3c->xfer.rx_len = len;
  i3c->xfer.tx_buf = NULL;
  i3c->xfer.tx_len = 0;

  i3c_master_rx(i3c->regs, &(i3c->xfer), index, len);

  return ARM_DRIVER_OK;
}

/**
  \fn           int I3Cx_SlaveTransmit(I3C_RESOURCES *i3c,
                                       const uint8_t *data,
                                       uint16_t       len)
  \brief        Write data to the master
  \param[in]    i3c      : Pointer to i3c resources structure
  \param[in]    data     : Pointer to buffer with data
                            which needs to be transmit to master
  \param[in]    len      : Number of bytes needs to be transmit
  \return       \ref execution_status
*/
static int I3Cx_SlaveTransmit(I3C_RESOURCES *i3c,
                              const uint8_t *data,
                              uint16_t       len)
{
  /* Checking for power done initialization */
  if (i3c->state.powered == 0U)
      return ARM_DRIVER_ERROR;

  /* Checking for slave initialization */
  if (i3c->state.slave_enabled == 0U)
      return ARM_DRIVER_ERROR;

  /* Parameter check */
  if (!data || !len)
      return ARM_DRIVER_ERROR_PARAMETER;

  if (i3c->status.busy)
      return ARM_DRIVER_ERROR_BUSY;

  i3c->status.busy = 1;

  i3c->xfer.tx_buf = data;
  i3c->xfer.tx_len = len;
  i3c->xfer.rx_buf = NULL;
  i3c->xfer.rx_len = 0;

  i3c_slave_tx(i3c->regs, &(i3c->xfer), len);

  return ARM_DRIVER_OK;
}

/**
  \fn           int I3Cx_SlaveReceive(I3C_RESOURCES *i3c,
                                      uint8_t       *data,
                                      uint32_t       len)
  \brief        Read data from the master
  \param[in]    i3c      : Pointer to i3c resources structure
  \param[in]    data     : Pointer to buffer for data
                            to receive from master
  \param[in]    len      : Number of bytes needs to be receive
  \return       \ref execution_status
*/
static int I3Cx_SlaveReceive(I3C_RESOURCES *i3c,
                             uint8_t       *data,
                             uint32_t       len)
{
  /* Checking for power done initialization */
  if (i3c->state.powered == 0U)
      return ARM_DRIVER_ERROR;

  if (i3c->state.slave_enabled == 0U)
      return ARM_DRIVER_ERROR;

  /* Parameter check */
  if (!data || !len)
      return ARM_DRIVER_ERROR_PARAMETER;

  if (i3c->status.busy)
      return ARM_DRIVER_ERROR_BUSY;

  i3c->status.busy = 1;

  /* Buffer initialization for TX/RX */
  i3c->xfer.rx_buf = data;
  i3c->xfer.rx_len = len;
  i3c->xfer.tx_buf = NULL;
  i3c->xfer.tx_len = 0;

  i3c_slave_rx(i3c->regs, &(i3c->xfer));

  return ARM_DRIVER_OK;
}

/**
  \fn           int I3Cx_MasterAssignDA(I3C_RESOURCES *i3c,
                                        uint8_t       *dyn_addr,
                                        uint8_t        sta_addr)
  \brief        Assign dynamic address to the i3c slave using SETDASA;
                Note: Only required for i3c slave devices;
                      i2c slave device uses static address
                      for communication \ref I3Cx_AttachI2Cdev.
  \param[in]    i3c      : Pointer to i3c resources structure
  \param[in]    dyn_addr : Pointer to char where dynamic address
                            will be copied.
  \param[in]    sta_addr : Static address of i3c device
  \return       \ref execution_status
*/
static int I3Cx_MasterAssignDA(I3C_RESOURCES *i3c,
                               uint8_t       *dyn_addr,
                               uint8_t        sta_addr)
{
  int32_t  pos;

  if (!dyn_addr || !sta_addr)
    return ARM_DRIVER_ERROR_PARAMETER;

  if (i3c->state.powered == 0U)
    return ARM_DRIVER_ERROR;

  if (i3c->status.busy)
    return ARM_DRIVER_ERROR_BUSY;

  i3c->status.busy = 1;

  /* Find the first unused index in freepos, note that this also
   * corresponds to the first unused location in the DAT
   */
  pos = I3cMasterGetFreePos(i3c);

  /* the dat is full */
  if (pos < 0) {
    i3c->status.busy = 0;
    return ARM_DRIVER_ERROR;
  }

  /* reserve the index */
  i3c->freepos &= ~(BIT(pos));
  *dyn_addr = pos + 0x09; /* we start assigning addresses from 0x09 */
  i3c->addrs[pos] = *dyn_addr;

  /* ok, we have space in the dat, program the dat in index pos */
  i3c_add_slv_to_dat(i3c->regs, pos, *dyn_addr, sta_addr);

  i3c_send_ccc_cmd(i3c->regs, I3C_CCC_SETDASA, pos);

  return ARM_DRIVER_OK;
}

/**
  \fn           int I3Cx_AttachI2Cdev(I3C_RESOURCES *i3c, uint8_t sta_addr)
  \brief        Attach legacy i2c device to the i3c bus.
  \param[in]    i3c      : Pointer to i3c resources structure
  \param[in]    sta_addr : Static address of legacy i2c device
  \return       \ref execution_status
*/
static int I3Cx_AttachI2Cdev(I3C_RESOURCES *i3c, uint8_t sta_addr)
{
  int32_t  pos;

  if (!sta_addr)
    return ARM_DRIVER_ERROR_PARAMETER;

  if (i3c->state.powered == 0U)
    return ARM_DRIVER_ERROR;

  /* Find the first unused index in freepos, note that this also
   * corresponds to the first unused location in the DAT(Device Address Table)
   */
  pos = I3cMasterGetFreePos(i3c);

  /* DAT(Device Address Table) is full? */
  if (pos < 0)
  {
    /* error: DAT is full */
    return ARM_DRIVER_ERROR;
  }

  /* reserve the index */
  i3c->freepos &= ~(BIT(pos));

  /* use static address for communication. */
  i3c->addrs[pos] = sta_addr;

  /* ok, we have space in the DAT, store the static address and
   * mark as i2c legacy device is present.
   */

  /* Program the DAT(device address table) in index pos. */
  /* for i2c slave dynamic address is not available. */
  i3c_add_slv_to_dat(i3c->regs, pos, 0, sta_addr);

  return ARM_DRIVER_OK;
}

/**
  \fn           int I3Cx_Detachdev(I3C_RESOURCES *i3c, uint8_t addr)
  \brief        Detach already attached i2c/i3c device from the i3c bus.
  \param[in]    i3c      : Pointer to i3c resources structure
  \param[in]    addr     : Static  address of already attached i2c device
                                        OR
                           Dynamic address of already attached i3c device
  \return       \ref execution_status
*/
static int I3Cx_Detachdev(I3C_RESOURCES *i3c, uint8_t addr)
{
  int32_t  pos;

  if (!addr)
    return ARM_DRIVER_ERROR_PARAMETER;

  if (i3c->state.powered == 0U)
    return ARM_DRIVER_ERROR;

  /* Get already attached i2c device address index in
   * DAT (device address table). */
  pos = I3cMasterGetAddrPos(i3c, addr);

  /* i2c i3c is not attached to DAT? */
  if (pos < 0)
  {
    /* err: i2c slave device is not attached to DAT,
     * first attach i2c device \ref I3Cx_AttachI2Cdev */
    return ARM_DRIVER_ERROR;
  }

  /* free the index */
  i3c->freepos |= (BIT(pos));
  i3c->addrs[pos] = 0;

  /* clear the DAT index pos */
  i3c_remove_slv_from_dat(i3c->regs, pos);

  return ARM_DRIVER_OK;
}

/**
  \fn           int32_t I3Cx_Control(I3C_RESOURCES *i3c,
                                     uint32_t       control,
                                     uint32_t       arg)
  \brief        Control i3c master and slave.
  \param[in]    i3c      : Pointer to i3c resources structure
  \param[in]    control  : Operation
  \param[in]    arg      : Argument of operation
  \return       \ref execution_status
*/
static int32_t I3Cx_Control(I3C_RESOURCES *i3c,
                            uint32_t       control,
                            uint32_t       arg)
{
  I3C_I2C_SPEED_MODE i2c_speed_mode = 0;
  uint8_t slv_addr = 0;

  if (i3c->state.powered == 0U)
    return ARM_DRIVER_ERROR;

  switch(control)
  {
  case I3C_MASTER_SET_BUS_MODE:

    i3c->datp    = i3c_get_dat_addr(i3c->regs);
    i3c->maxdevs = i3c_get_dat_depth(i3c->regs);
    i3c->freepos = GENMASK(i3c->maxdevs - 1, 0);

    i3c_master_init(i3c->regs);

    /* set state as master enabled. */
    i3c->state.master_enabled = 1;

    switch(arg)
    {
    case I3C_BUS_MODE_MIXED_FAST_I2C_FMP_SPEED_1_MBPS:
    case I3C_BUS_MODE_MIXED_FAST_I2C_FM_SPEED_400_KBPS:
    case I3C_BUS_MODE_MIXED_SLOW_I2C_SS_SPEED_100_KBPS:
    case I3C_BUS_MODE_MIXED_LIMITED:

      if(arg == I3C_BUS_MODE_MIXED_FAST_I2C_FMP_SPEED_1_MBPS)
        i2c_speed_mode = I3C_I2C_SPEED_MODE_FMP_1_MBPS;

      if(arg == I3C_BUS_MODE_MIXED_FAST_I2C_FM_SPEED_400_KBPS)
        i2c_speed_mode = I3C_I2C_SPEED_MODE_FM_400_KBPS;

      if(arg == I3C_BUS_MODE_MIXED_SLOW_I2C_SS_SPEED_100_KBPS)
        i2c_speed_mode = I3C_I2C_SPEED_MODE_SS_100_KBPS;

      if(arg == I3C_BUS_MODE_MIXED_LIMITED)
        i2c_speed_mode = I3C_I2C_SPEED_MODE_LIMITED;

      if (!(i3c->core_clk))
        return ARM_DRIVER_ERROR;

      /* i2c clock configuration for selected Speed mode. */
      i2c_clk_cfg(i3c->regs, i3c->core_clk, i2c_speed_mode);

    /* fall through */
    case I3C_BUS_MODE_PURE:

      if (!(i3c->core_clk))
        return ARM_DRIVER_ERROR;

      /* i3c clock configuration */
      i3c_clk_cfg(i3c->regs, i3c->core_clk);
      break;

    default:
      return ARM_DRIVER_ERROR_UNSUPPORTED;
    }
    break;
  case I3C_SLAVE_SET_ADDR:

      i3c->datp    = i3c_get_dat_addr(i3c->regs);
      i3c->maxdevs = i3c_get_dat_depth(i3c->regs);
      i3c->freepos = GENMASK(i3c->maxdevs - 1, 0);

      /* Initialize and Enable i3c Slave */
      slv_addr = arg;
      i3c_slave_init(i3c->regs, slv_addr);

      /* set state as slave enabled. */
      i3c->state.slave_enabled = 1;
      break;

  default:
    return ARM_DRIVER_ERROR_UNSUPPORTED;
  }

  return ARM_DRIVER_OK;
}

/**
  \fn           int32_t I3Cx_Initialize(I3C_RESOURCES         *i3c,
                                        ARM_I3C_SignalEvent_t  cb_event)
  \brief        Initialize the i3c device.
  \param[in]    i3c      : Pointer to i3c resources structure
  \param[in]    cb_event : Pointer to I3C Event
                            \ref ARM_I3C_SignalEvent_t
  \return       \ref execution_status
*/
static int32_t I3Cx_Initialize(I3C_RESOURCES         *i3c,
                               ARM_I3C_SignalEvent_t  cb_event)
{
  if (i3c->state.initialized == 1)
    return ARM_DRIVER_OK;

  if(!cb_event)
    return ARM_DRIVER_ERROR_PARAMETER;

  /* set the user callback event. */
  i3c->cb_event = cb_event;

  /* set the state as initialized. */
  i3c->state.initialized = 1;
  return ARM_DRIVER_OK;
}

/**
  \fn           int32_t I3Cx_Uninitialize(I3C_RESOURCES  *i3c)
  \brief        Uninitialize the i3c device
  \param[in]    i3c      : Pointer to i3c resources structure
  \return       \ref execution_status
*/
static int32_t I3Cx_Uninitialize(I3C_RESOURCES  *i3c)
{
  i3c->state.initialized = 0;
  return ARM_DRIVER_OK;
}

/**
  \fn           int32_t I3Cx_PowerControl(I3C_RESOURCES   *i3c,
                                          ARM_POWER_STATE  state)
  \brief        i3c power control
  \param[in]    i3c      : Pointer to i3c resources structure
  \param[in]    state    : Power state
  \return       \ref execution_status
*/
static int32_t I3Cx_PowerControl(I3C_RESOURCES   *i3c,
                                 ARM_POWER_STATE  state)
{
  switch (state)
  {
    case ARM_POWER_OFF:

      /* Disable i3c IRQ */
      NVIC_DisableIRQ(i3c->irq);

      /* Clear Any Pending i3c IRQ */
      NVIC_ClearPendingIRQ(i3c->irq);

      /* i3c EXPMST0 control configuration:
       *  Disable i3c clock. */
      disable_i3c_clock();

      /* Reset the power state. */
      i3c->state.powered = 0;
      break;

    case ARM_POWER_FULL:

      if (i3c->state.initialized == 0U)
        return ARM_DRIVER_ERROR;

      if (i3c->state.powered)
        break;

      /* i3c EXPMST0 control configuration:
       *  Enable i3c clock. */
      enable_i3c_clock();

      /* Enable i3c IRQ */
      NVIC_ClearPendingIRQ(i3c->irq);
      NVIC_SetPriority(i3c->irq, i3c->irq_priority);
      NVIC_EnableIRQ(i3c->irq);

      /* Set the state as powered */
      i3c->state.powered = 1;
      break;

    default:
      return ARM_DRIVER_ERROR_UNSUPPORTED;
  }

  return ARM_DRIVER_OK;
}


/* I3C Driver Instance */
#if (RTE_I3C)

/* I3C Device Resources */
static I3C_RESOURCES i3c =
{
  .regs         = (I3C_Type *)I3C_BASE,
  .cb_event     = NULL,
  .core_clk     = SYST_PCLK,
  .xfer         = {0},
  .status       = {0},
  .state        = {0},
  .irq          = (IRQn_Type) I3C_IRQ_IRQn,
  .irq_priority = RTE_I3C_IRQ_PRI,
};

void I3C_IRQHandler(void)
{
  I3C_XFER *xfer = &(i3c.xfer);
  uint32_t event = 0;

  i3c_irq_handler(i3c.regs, xfer);

  if(xfer->status == I3C_XFER_STATUS_DONE)
  {
    /* mark event as Transfer done. */
    event = ARM_I3C_EVENT_TRANSFER_DONE;
  }

  if(xfer->status == I3C_XFER_STATUS_ERROR)
  {
    /* error: Resume i3c controller and clear error status. */
    i3c_resume(i3c.regs);
    i3c_clear_xfer_error(i3c.regs);

    /* mark event as Transfer Error. */
    event = ARM_I3C_EVENT_TRANSFER_ERROR;
  }

  if(xfer->status == I3C_XFER_STATUS_MST_TX_DONE)
  {
    /* mark event as Master TX Transfer done. */
    event = ARM_I3C_EVENT_MST_TX_DONE;
  }

  if(xfer->status == I3C_XFER_STATUS_MST_RX_DONE)
  {
    /* mark event as Master RX Transfer done. */
    event = ARM_I3C_EVENT_MST_RX_DONE;
  }

  if(xfer->status == I3C_XFER_STATUS_SLV_TX_DONE)
  {
    /* mark event as Slave TX Transfer done. */
    event = ARM_I3C_EVENT_SLV_TX_DONE;
  }

  if(xfer->status == I3C_XFER_STATUS_SLV_RX_DONE)
  {
    /* mark event as Slave RX Transfer done. */
    event = ARM_I3C_EVENT_SLV_RX_DONE;
  }

  if(xfer->status == I3C_XFER_STATUS_SLV_DYN_ADDR_ASSGN)
  {
    /* mark event as Slave dynamic address assignment done. */
    event = ARM_I3C_EVENT_SLV_DYN_ADDR_ASSGN;
  }

  if(event)
  {
    /* clear transfer status. */
    xfer->status = I3C_XFER_STATUS_NONE;

    /* clear busy flag. */
    i3c.status.busy = 0;

    /* call the user callback */
    if(i3c.cb_event)
      i3c.cb_event(event);
  }
}

/* wrapper functions for I3C */
static int32_t I3C_Initialize(ARM_I3C_SignalEvent_t cb_event)
{
  return (I3Cx_Initialize(&i3c, cb_event));
}

static int32_t I3C_Uninitialize(void)
{
  return (I3Cx_Uninitialize(&i3c));
}

static int32_t I3C_PowerControl(ARM_POWER_STATE state)
{
  return (I3Cx_PowerControl(&i3c, state));
}

static int32_t I3C_MasterTransmit(uint8_t addr, const uint8_t *data, uint16_t len)
{
  return (I3Cx_MasterTransmit(&i3c, addr, data, len));
}

static int32_t I3C_MasterReceive(uint8_t addr, uint8_t *data, uint16_t len)
{
  return (I3Cx_MasterReceive(&i3c, addr, data, len));
}

static int32_t I3C_SlaveTransmit(const uint8_t *data, uint16_t len)
{
  return (I3Cx_SlaveTransmit(&i3c, data, len));
}

static int32_t I3C_SlaveReceive(uint8_t *data, uint16_t len)
{
  return (I3Cx_SlaveReceive(&i3c, data, len));
}

static int32_t I3C_MasterSendCommand(I3C_CMD *ccc)
{
  return (I3Cx_MasterSendCommand(&i3c, ccc));
}

static int32_t I3C_Control(uint32_t control, uint32_t arg)
{
  return (I3Cx_Control(&i3c, control, arg));
}

static int32_t I3C_MasterAssignDA(uint8_t *dyn_addr, uint8_t sta_addr)
{
  return (I3Cx_MasterAssignDA(&i3c, dyn_addr, sta_addr));
}

static int32_t I3C_AttachI2Cdev(uint8_t sta_addr)
{
  return (I3Cx_AttachI2Cdev(&i3c, sta_addr));
}

static int32_t I3C_Detachdev(uint8_t addr)
{
  return (I3Cx_Detachdev(&i3c, addr));
}


/* I3C Driver Control Block */
extern ARM_DRIVER_I3C Driver_I3C;
ARM_DRIVER_I3C Driver_I3C =
{
  I3C_GetVersion,
  I3C_GetCapabilities,
  I3C_Initialize,
  I3C_Uninitialize,
  I3C_PowerControl,
  I3C_MasterTransmit,
  I3C_MasterReceive,
  I3C_SlaveTransmit,
  I3C_SlaveReceive,
  I3C_Control,
  I3C_MasterSendCommand,
  I3C_MasterAssignDA,
  I3C_AttachI2Cdev,
  I3C_Detachdev
};
#endif /* RTE_I3C */

/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
