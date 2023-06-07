/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 */

/**************************************************************************//**
 * @file     I3C_Baremetal_slaveside.c
 * @author   Prabhakar kumar
 * @email    prabhakar.kumar@alifsemi.com
 * @version  V1.0.0
 * @date     27-May-2023
 * @brief    Baremetal to verify slave side loop back test
 *           - 1 byte of data transmitted from master and slave receive 1 byte and
 *             same 1 byte of data received by slave is transmitted through slave
 *             transmit and master receive 1byte.
 *
 *           I3C slave configuration.
 *           - In control API parameter:
 *             control : I3C_SET_SLAVE_ADDR(set slave address)
 *             arg     : I3C_SLAVE_ADDRESS macro is defined to set address
 *                       of slave
 *
 *           Hardware Setup:
 *           Connecting two flat board i3c pin as there is one instance of i3c
 *           on the board. connect Below pins on both the boards
 *           SDA P1_2 -> P1_2
 *           SCL P1_3 -> P1_3
 *           GND -> GND of both the boards
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

/* System Includes */
#include <stdio.h>

/* Project Includes */
/* I3C Driver */
#include "Driver_I3C.h"
#include "system_utils.h"

/* PINMUX Driver */
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

/* i3c Driver instance 0 */
extern ARM_DRIVER_I3C Driver_I3C;
static ARM_DRIVER_I3C *I3Cdrv = &Driver_I3C;

/* set slave address */
#define I3C_SLAVE_ADDRESS             (0X48)

/* receive data from i3c */
uint8_t rx_data[1] = {0x00};

volatile uint32_t cb_event;

void i3c_slave_loopback_demo(void);

/* i3c callback events */
typedef enum _I3C_CB_EVENT{
    I3C_CB_EVENT_SUCCESS        = (1 << 0),
    I3C_CB_EVENT_ERROR          = (1 << 1),
    I3C_CB_EVENT_MST_TX_DONE    = (1 << 2),
    I3C_CB_EVENT_MST_RX_DONE    = (1 << 3),
    I3C_CB_EVENT_SLV_TX_DONE    = (1 << 4),
    I3C_CB_EVENT_SLV_RX_DONE    = (1 << 5),
    I3C_CB_EVENT_DYN_ADDR_ASSGN = (1 << 6)
}I3C_CB_EVENT;

/**
  \fn          INT hardware_init(void)
  \brief       i3c hardware pin initialization:
                - PIN-MUX configuration
                - PIN-PAD configuration
  \param[in]   void
  \return      0:success; -1:failure
*/
int32_t hardware_init(void)
{
    /* I3C_SDA_B */
    pinconf_set(PORT_1, PIN_2, PINMUX_ALTERNATE_FUNCTION_3,
            PADCTRL_READ_ENABLE | PADCTRL_DRIVER_DISABLED_PULL_UP | \
            PADCTRL_OUTPUT_DRIVE_STRENGTH_04_MILI_AMPS);

    /* I3C_SCL_B */
    pinconf_set( PORT_1, PIN_3, PINMUX_ALTERNATE_FUNCTION_3,
            PADCTRL_READ_ENABLE | PADCTRL_DRIVER_DISABLED_PULL_UP | \
            PADCTRL_OUTPUT_DRIVE_STRENGTH_04_MILI_AMPS);

    return ARM_DRIVER_OK;
}

/**
  \fn          void I3C_callback(UINT event)
  \brief       i3c isr callback
  \param[in]   event: i3c Event
  \return      none
*/
static void I3C_callback(uint32_t event)
{
    if (event & ARM_I3C_EVENT_TRANSFER_DONE)
    {
        cb_event = I3C_CB_EVENT_SUCCESS;
    }
    if (event & ARM_I3C_EVENT_TRANSFER_ERROR)
    {
        cb_event = I3C_CB_EVENT_ERROR;
    }
    if (event & ARM_I3C_EVENT_MST_TX_DONE)
    {
        cb_event = I3C_CB_EVENT_MST_TX_DONE;
    }
    if (event & ARM_I3C_EVENT_MST_RX_DONE)
    {
        cb_event = I3C_CB_EVENT_MST_RX_DONE;
    }
    if (event & ARM_I3C_EVENT_SLV_TX_DONE)
    {
        cb_event = I3C_CB_EVENT_SLV_TX_DONE;
    }
    if (event & ARM_I3C_EVENT_SLV_RX_DONE)
    {
        cb_event = I3C_CB_EVENT_SLV_RX_DONE;
    }
    if (event & ARM_I3C_EVENT_SLV_DYN_ADDR_ASSGN)
    {
        cb_event = I3C_CB_EVENT_DYN_ADDR_ASSGN;
    }
}

/**
  \fn          void i3c_slave_loopback_demo(void)
  \brief       Baremetal to verify i3c slave mode
               This demo does:
                 - initialize i3c driver;
                 - set slave address and initialize slave
                 - Receive and transmit byte from master
  \return      none
*/
void i3c_slave_loopback_demo(void)
{
    int   ret    = 0;
    int   len    = 0;

    ARM_DRIVER_VERSION version;

    printf("\r\n \t\t >>> Slave loop back demo starting up!!! <<< \r\n");

    /* Get i3c driver version. */
    version = I3Cdrv->GetVersion();
    printf("\r\n i3c version api:0x%X driver:0x%X \r\n",  \
                           version.api, version.drv);

    /* Initialize i3c hardware pins using PinMux Driver. */
    ret = hardware_init();
    if(ret != 0)
    {
        printf("\r\n Error: i3c hardware_init failed.\r\n");
        return;
    }

    /* Initialize I3C driver */
    ret = I3Cdrv->Initialize(I3C_callback);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: I3C Initialize failed.\r\n");
        return;
    }

    /* Power up I3C peripheral */
    ret = I3Cdrv->PowerControl(ARM_POWER_FULL);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: I3C Power Up failed.\r\n");
        goto error_poweroff;
    }

    /* Control I3C interface */
    ret = I3Cdrv->Control(I3C_SLAVE_SET_ADDR, I3C_SLAVE_ADDRESS);
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: I3C Control failed.\r\n");
        goto error_uninitialize;
    }

while(1)
{
        len = 1;

        /* Slave Receive */
        ret = I3Cdrv->SlaveReceive(&rx_data[0], len);
        if(ret != ARM_DRIVER_OK)
        {
            printf("\r\n Error: I3C Slave Receive failed. \r\n");
            goto error_poweroff;
        }

        while(!((cb_event == I3C_CB_EVENT_SLV_RX_DONE) || (cb_event == I3C_CB_EVENT_ERROR)));

        if(cb_event == I3C_CB_EVENT_ERROR)
        {
            printf("\nError: I3C Slave Receive failed\n");
            while(1);
        }

        cb_event = 0;

        /* Delay */
        PMU_delay_loop_us(1000);

        /* For loop back test, same rx_data is passed as a parameter in
         * the slave transmit API.
         * To make it a loop, Master transmit 1 bytes and slave receive's it
         * where same byte is transmitted from slave transmit API and master
         * receive's it
         */

        /* Slave Transmit*/
        ret = I3Cdrv->SlaveTransmit(rx_data, 1);
        if(ret != ARM_DRIVER_OK)
        {
            printf("\r\n Error: I3C slave Transmit failed. \r\n");
            goto error_poweroff;
        }

        while(!((cb_event == I3C_CB_EVENT_MST_TX_DONE) || (cb_event == I3C_CB_EVENT_ERROR)));

        if(cb_event == I3C_CB_EVENT_ERROR)
        {
            printf("\nError: I2C Slave Transmit failed\n");
            while(1);
        }

        cb_event = 0;
}

error_poweroff:

    /* Power off I3C peripheral */
    ret = I3Cdrv->PowerControl(ARM_POWER_OFF);
    if(ret != ARM_DRIVER_OK)
    {
         printf("\r\n Error: I3C Power OFF failed.\r\n");
    }

error_uninitialize:

    /* Un-initialize I3C driver */
    ret = I3Cdrv->Uninitialize();
    if(ret != ARM_DRIVER_OK)
    {
        printf("\r\n Error: I3C Uninitialize failed.\r\n");
    }

    printf("\r\n I3C demo exiting ...\r\n");
}

/* Define main entry point.  */
int main()
{
    /* Enter the I3C.  */
    i3c_slave_loopback_demo();
}

/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
