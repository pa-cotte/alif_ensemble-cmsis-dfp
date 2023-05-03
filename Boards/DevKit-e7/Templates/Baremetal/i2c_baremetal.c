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
 * @file     i2c_baremetal.c
 * @brief    TestApp to verify I2C Master and Slave functionality
 *           using baremetal without any operating system.
 *
 *           Code will verify:
 *            1.)Master transmit and Slave receive
 *            2.)Master receive  and Slave transmit
 *                I2C0 instance is taken as Master and
 *                I2C1 instance is taken as Slave.
 ******************************************************************************/

#include <stdio.h>
#include <string.h>

#include "RTE_Components.h"
#include CMSIS_device_header

#include "Driver_I2C.h"
#include "i2c.h"
#include "pinconf.h"

/* For Release build disable printf and semihosting */
#define DISABLE_PRINTF

#ifdef DISABLE_PRINTF
    #define printf(fmt, ...) (0)
    /* Also Disable Semihosting */
    #if __ARMCC_VERSION >= 6000000
            __asm(".global __use_no_semihosting");
    #elif __ARMCC_VERSION >= 5000000
            #pragma import(__use_no_semihosting)
    #else
            #error Unsupported compiler
    #endif

    void _sys_exit(int return_code) {
            while (1);
    }
#endif

/* I2C Driver instance */
extern ARM_DRIVER_I2C Driver_I2C0;
static ARM_DRIVER_I2C *I2C_mstdrv = &Driver_I2C0;

extern ARM_DRIVER_I2C Driver_I2C1;
static ARM_DRIVER_I2C *I2C_slvdrv = &Driver_I2C1;

volatile uint32_t mst_cb_status = 0;
volatile uint32_t slv_cb_status = 0;
int slv_rec_compare;
int slv_xfer_compare;

#define TAR_ADDRS         (0X50)   /* Target(Slave) Address, use by Master */
#define SAR_ADDRS         (0X50)   /* Slave Own Address,     use by Slave  */
#define RESTART           (0X01)
#define STOP              (0X00)

/* master transmit and slave receive */
#define MST_BYTE_TO_TRANSMIT            10

/* slave transmit and master receive */
#define SLV_BYTE_TO_TRANSMIT            15

/* Master parameter set */

/* Master TX Data (Any random value). */
uint8_t MST_TX_BUF[MST_BYTE_TO_TRANSMIT] =
{
    0XAF,0xCE,0xAB,0xDE,0x4A,
    0X22,0X55,0X89,0X46,0X88
};

/* master receive buffer */
uint8_t MST_RX_BUF[SLV_BYTE_TO_TRANSMIT];

/* Master parameter set END  */


/* Slave parameter set */

/* slave receive buffer */
uint8_t SLV_RX_BUF[MST_BYTE_TO_TRANSMIT];

/* Slave TX Data (Any random value). */
uint8_t SLV_TX_BUF[SLV_BYTE_TO_TRANSMIT] =
{
    0X84,0xCD,0x6F,0x5E,0x49,
    0X42,0X2B,0X23,0X46,0X78,
    0X67,0XCC,0xDD,0XAB,0XAE
};

/* Slave parameter set END */


static void i2c_mst_conversion_callback(uint32_t event)
{
      /* Save received events */
      /* Optionally, user can define specific actions for an event */

      if (event & ARM_I2C_EVENT_TRANSFER_INCOMPLETE) {
        /* Less data was transferred than requested */
      }

      if (event & ARM_I2C_EVENT_TRANSFER_DONE) {
        /* Transfer or receive is finished */
        mst_cb_status = 1;
      }

      if (event & ARM_I2C_EVENT_ADDRESS_NACK) {
        /* Slave address was not acknowledged */
      }

      if (event & ARM_I2C_EVENT_ARBITRATION_LOST) {
       /* Master lost bus arbitration */
      }

      if (event & ARM_I2C_EVENT_BUS_ERROR) {
        /* Invalid start/stop position detected */
      }

      if (event & ARM_I2C_EVENT_BUS_CLEAR) {
       /* Bus clear operation completed */
      }

      if (event & ARM_I2C_EVENT_GENERAL_CALL) {
       /* Slave was addressed with a general call address */
      }

      if (event & ARM_I2C_EVENT_SLAVE_RECEIVE) {
       /* Slave addressed as receiver but SlaveReceive operation is not started */
      }

      if (event & ARM_I2C_EVENT_SLAVE_TRANSMIT) {
       /* Slave addressed as transmitter but SlaveTransmit operation is not started */
      }
}

static void i2c_slv_conversion_callback(uint32_t event)
{
      /* Save received events */
      /* Optionally, user can define specific actions for an event */

      if (event & ARM_I2C_EVENT_TRANSFER_INCOMPLETE) {
        /* Less data was transferred than requested */
      }

      if (event & ARM_I2C_EVENT_TRANSFER_DONE) {
        /* Transfer or receive is finished */
           slv_cb_status = 1;
      }

      if (event & ARM_I2C_EVENT_ADDRESS_NACK) {
        /* Slave address was not acknowledged */
      }

      if (event & ARM_I2C_EVENT_ARBITRATION_LOST) {
       /* Master lost bus arbitration */
      }

      if (event & ARM_I2C_EVENT_BUS_ERROR) {
        /* Invalid start/stop position detected */
      }

      if (event & ARM_I2C_EVENT_BUS_CLEAR) {
       /* Bus clear operation completed */
      }

      if (event & ARM_I2C_EVENT_GENERAL_CALL) {
       /* Slave was addressed with a general call address */
      }

      if (event & ARM_I2C_EVENT_SLAVE_RECEIVE) {
       /* Slave addressed as receiver but SlaveReceive operation is not started */
      }

      if (event & ARM_I2C_EVENT_SLAVE_TRANSMIT) {
       /* Slave addressed as transmitter but SlaveTransmit operation is not started */
      }
}

/* Pinmux for B0 */
void i2c_pinmux_B0()
{
    /* I2C0_SDA_A */
    pinconf_set(PORT_0, PIN_2, PINMUX_ALTERNATE_FUNCTION_3, \
         (PADCTRL_READ_ENABLE | PADCTRL_DRIVER_DISABLED_PULL_UP));

    /* I2C0_SCL_A */
    pinconf_set(PORT_0, PIN_3, PINMUX_ALTERNATE_FUNCTION_3, \
         (PADCTRL_READ_ENABLE | PADCTRL_DRIVER_DISABLED_PULL_UP));

    /* I2C1_SDA_A */
    pinconf_set(PORT_0, PIN_4, PINMUX_ALTERNATE_FUNCTION_4, \
         (PADCTRL_READ_ENABLE | PADCTRL_DRIVER_DISABLED_PULL_UP));

    /* I2C1_SCL_A */
    pinconf_set(PORT_0, PIN_5, PINMUX_ALTERNATE_FUNCTION_4, \
         (PADCTRL_READ_ENABLE | PADCTRL_DRIVER_DISABLED_PULL_UP));
}

void I2C_demo()
{
    unsigned long events   = 0;
    int   ret      = 0;
    ARM_DRIVER_VERSION version;
    ARM_I2C_CAPABILITIES capabilities;

    printf("\r\n >>> I2C demo starting up!!! <<< \r\n");

    /* B0 Pinmux */
    i2c_pinmux_B0();

    version = I2C_mstdrv->GetVersion();
    printf("\r\n I2C version api:0x%X driver:0x%X...\r\n",version.api, version.drv);

    /* Initialize I2C driver */
    ret = I2C_mstdrv->Initialize(i2c_mst_conversion_callback);
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: I2C master init failed\n");
        return;
    }

    /* Initialize I2C driver */
    ret = I2C_slvdrv->Initialize(i2c_slv_conversion_callback);
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: I2C slave init failed\n");
        return;
    }

    /* Power control I2C */
    ret = I2C_mstdrv->PowerControl(ARM_POWER_FULL);
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: I2C Power up failed\n");
        goto error_uninitialize;
    }

    /* Power control I2C */
    ret = I2C_slvdrv->PowerControl(ARM_POWER_FULL);
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: I2C Power up failed\n");
        goto error_uninitialize;
    }

    ret = I2C_mstdrv->Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_STANDARD);
    if(ret != ARM_DRIVER_OK){
        printf("\r\n Error: I2C master init failed\n");
        goto error_uninitialize;
    }

    ret = I2C_slvdrv->Control(ARM_I2C_OWN_ADDRESS, SAR_ADDRS);
     if(ret != ARM_DRIVER_OK){
         printf("\r\n Error: I2C slave init failed\n");
         goto error_uninitialize;
     }

     printf("\n----------------Master transmit/slave receive-----------------------\n");

     I2C_slvdrv->SlaveReceive(SLV_RX_BUF, MST_BYTE_TO_TRANSMIT);

     /* delay */
     PMU_delay_loop_us(500);

     I2C_mstdrv->MasterTransmit(TAR_ADDRS, MST_TX_BUF, MST_BYTE_TO_TRANSMIT, STOP);

     /* wait for master/slave callback. */
     while(mst_cb_status == 0);
     mst_cb_status = 0;

     while(slv_cb_status == 0);
     slv_cb_status = 0;

     /* Compare received data. */
     slv_rec_compare = memcmp(&SLV_RX_BUF, &MST_TX_BUF, MST_BYTE_TO_TRANSMIT);
     if(slv_rec_compare != 0)
     {
         printf("\n XXX Err Master transmit/slave receive XXX \n");
         printf("\n ---Stop--- \r\n wait forever >>> \n");
         while(1);
     }

     printf("\n----------------Master receive/slave transmit-----------------------\n");

     I2C_mstdrv->MasterReceive(TAR_ADDRS, MST_RX_BUF, SLV_BYTE_TO_TRANSMIT, STOP);

     I2C_slvdrv->SlaveTransmit(SLV_TX_BUF, SLV_BYTE_TO_TRANSMIT);

     /* wait for master/slave callback. */
     while(mst_cb_status == 0);
     mst_cb_status = 0;

     while(slv_cb_status == 0);
     slv_cb_status = 0;

     /* Compare received data. */
     slv_xfer_compare = memcmp(&SLV_TX_BUF, &MST_RX_BUF, SLV_BYTE_TO_TRANSMIT);

     if(slv_xfer_compare != 0)
     {
         printf("\n XXX Err Master receive/slave transmit XXX \n");
         printf("\n ---Stop--- \r\n wait forever >>> \n");
         while(1);
     }

     ret =I2C_mstdrv->Uninitialize();
     ret =I2C_slvdrv->Uninitialize();

     printf("\n >>> I2C conversion completed without any error. \n");
     printf("\n ---END--- \r\n wait forever >>> \n");
     while(1);

  error_poweroff:
      /* Power off I2C peripheral */
      ret = I2C_mstdrv->PowerControl(ARM_POWER_OFF);
      if(ret != ARM_DRIVER_OK)
      {
         printf("\r\n Error: I2C Power OFF failed.\r\n");
      }
      ret = I2C_slvdrv->PowerControl(ARM_POWER_OFF);
      if(ret != ARM_DRIVER_OK)
      {
         printf("\r\n Error: I2C Power OFF failed.\r\n");
      }

  error_uninitialize:
      /* Un-initialize I2C driver */
      ret = I2C_mstdrv->Uninitialize();
      ret = I2C_slvdrv->Uninitialize();
      if(ret != ARM_DRIVER_OK)
      {
        printf("\r\n Error: I2C Uninitialize failed.\r\n");
      }
      printf("\r\n XXX I2C demo thread exiting XXX...\r\n");
}


int main (void)
{
    printf("\t\t myI2C_Thread_entry \r\n");
    I2C_demo();
    while(1);
}
