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
 * @file     DPHY_Loopback_Test_Baremetal.c
 * @author   Chandra Bhushan Singh
 * @email    chandrabhushan.singh@alifsemi.com
 * @version  V1.0.0
 * @date     14-Feb-2023
 * @brief    DPHY loopback test source File.
 ******************************************************************************/

/* System Includes */
#include "stdio.h"
#include "stdint.h"
#include "string.h"

/* Loopback test header */
#include "DPHY_Loopback_test.h"

/* PINMUX Driver */
#include "Driver_PINMUX_AND_PINPAD.h"

/* Enable/Disable Redirect printf to UART.
* Providing option to user to select where to display loopback test result.
*/
#define PRINTF_REDIRECT	1

#if PRINTF_REDIRECT
/* USART Driver */
#include "Driver_USART.h"
#include "UART_dev.h"

/* UART Driver instance (UART0-UART7) */
#define UART      4

/* UART Driver */
extern ARM_DRIVER_USART ARM_Driver_USART_(UART);

/* UART Driver instance */
static ARM_DRIVER_USART *USARTdrv = &ARM_Driver_USART_(UART);

/* Disable Semihosting */
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

/* Used by fputc() */
void uartWrite(char c);

/* Used by fgetc() */
char uartRead(void);

FILE __stdout;
FILE __stdin;

#endif

#define DPHY_PLL_Frequency		80000000

/* Time in microseconds for which loopback test continue running */
#define DPHY_Loopback_Test_Runtime	9000000

#define PRINTF_REDIRECT_UART_RX_PORT		PORT_NUMBER_3
#define PRINTF_REDIRECT_UART_RX_PIN_NO		PIN_NUMBER_1
#define PRINTF_REDIRECT_UART_TX_PORT		PORT_NUMBER_3
#define PRINTF_REDIRECT_UART_TX_PIN_NO		PIN_NUMBER_2

#if PRINTF_REDIRECT
/* Redefine standard fputc function to redirect printf to UART instead of standard output */
int fputc(int c, FILE * stream)
{
	uartWrite(c);
	/* return the character written to denote a successful write */
	return c;
}

/* Redefine standard fgetc function to get input from UART instead of standard input */
int fgetc(FILE * stream)
{
	char c = uartRead();
	/* To echo Received characters back to serial Terminal */
	uartWrite(c);
	return c;
}

/* Used by fputc() */
void uartWrite(char c)
{
	uart_reg_set_t *uart_reg_ptr = (uart_reg_set_t *)UART4_BASE;

	/* wait until uart is to ready to send */
	while ( (uart_reg_ptr->usr & UART_USR_TRANSMIT_FIFO_NOT_FULL) == 0 );

	/* write a char to thr transmit holding register */
	uart_reg_ptr->rbr_thr_dll = c;
}

/* Used by fgetc() */
char uartRead(void)
{
	/* Device specific code to Receive a byte from RX pin.
	* return received chararacter(byte)
	*/
	uart_reg_set_t *uart_reg_ptr = (uart_reg_set_t *)UART4_BASE;

	/* wait until uart is ready to receive */
	while ( (uart_reg_ptr->usr & UART_USR_RECEIVE_FIFO_NOT_EMPTY) == 0 );

	/* read a char from receive buffer register */
	return (int32_t)uart_reg_ptr->rbr_thr_dll;
}

/**
  \fn          void UART_callback(UINT event)
  \brief       UART callback
  \param[in]   event: UART Event
  \return      none
  */
void UART_callback(uint32_t event)
{
	if (event & ARM_USART_EVENT_SEND_COMPLETE)
	{
		/* We are using polling method so interrupt events not used*/
	}

	if (event & ARM_USART_EVENT_RECEIVE_COMPLETE)
	{
		/* We are using polling method so interrupt events not used*/
	}

	if (event & ARM_USART_EVENT_RX_TIMEOUT)
	{
		/* We are using polling method so interrupt events not used*/
	}
}

/**
  \fn          int hardware_cfg(void)
  \brief       :UART hardware pin initialization (if printf redirection to UART is chosen):
  -  PIN-MUX configuration for UART receiver
  -  PIN-MUX configuration for UART transmitter
  \param[in]   none
  \return      ARM_DRIVER_OK: success; 0: failure
  */
int hardware_cfg(void)
{
	int ret = 0;

	/* Configure GPIO Pin : P3_1 as UART4 RX_B */
	ret = PINMUX_Config (PRINTF_REDIRECT_UART_RX_PORT, PRINTF_REDIRECT_UART_RX_PIN_NO, PINMUX_ALTERNATE_FUNCTION_1);
	if(ret != ARM_DRIVER_OK)
	{
		printf("\r\n Error: UART PINMUX as receiver failed.\r\n");
		return ret;
	}

	/* Configure GPIO Pin : P3_2 as UART4 TX_B */
	ret = PINMUX_Config (PRINTF_REDIRECT_UART_TX_PORT, PRINTF_REDIRECT_UART_TX_PIN_NO, PINMUX_ALTERNATE_FUNCTION_1);
	if(ret != ARM_DRIVER_OK)
	{
		printf("\r\n Error: UART PINMUX as transmitter failed.\r\n");
		return ret;
	}

	return ARM_DRIVER_OK;
}
#endif

/* Main entry point */
int main()
{
	int ret = 0;

#if PRINTF_REDIRECT
	/* Initialize UART4 hardware pins using PinMux driver if printf redirection to UART is selected */
	ret = hardware_cfg();
	if(ret != ARM_DRIVER_OK)
	{
		/* Error in hardware configuration */
		printf("\r\n Error: Hardware configuration failed.\r\n");
	}

	/* Initialize UART driver */
	ret = USARTdrv->Initialize(UART_callback);
	if(ret != ARM_DRIVER_OK)
	{
		/* Error in UART Initialize. */
		printf("\r\n ERROR: Initialize UART failed.\r\n");
		return 0;
	}

	/* Power up UART peripheral */
	ret = USARTdrv->PowerControl(ARM_POWER_FULL);
	if(ret != ARM_DRIVER_OK)
	{
		/* Error in UART Power Up. */
		printf("ERROR: UART power ON failed.\r\n");
		goto error_UART_uninitialize;
	}

	/* Configure UART to 115200 Bits/sec */
	ret = USARTdrv->Control(ARM_USART_MODE_ASYNCHRONOUS |
	                          ARM_USART_DATA_BITS_8       |
	                          ARM_USART_PARITY_NONE       |
	                          ARM_USART_STOP_BITS_1       |
	                          ARM_USART_FLOW_CONTROL_NONE, 115200);
	if(ret != ARM_DRIVER_OK)
	{
		/* Error in UART Control. */
		printf("ERROR: UART configuration failed.\r\n");
		goto error_UART_poweroff;
	}

	/* Enable UART Receiver and Transmitter lines */
	ret =  USARTdrv->Control(ARM_USART_CONTROL_TX, 1);
	if(ret != ARM_DRIVER_OK)
	{
		/* Error in UART Control TX. */
		printf("ERROR: UART transmitter configuration failed.\r\n");
		goto error_UART_poweroff;
	}

	ret =  USARTdrv->Control(ARM_USART_CONTROL_RX, 1);
	if(ret != ARM_DRIVER_OK)
	{
		/* Error in UART Control RX. */
		printf("ERROR: UART receiver configuration failed.\r\n");
		goto error_UART_poweroff;
	}
#endif

	ret = DPHY_External_Loopback_Test(DPHY_PLL_Frequency, DPHY_Loopback_Test_Runtime);
	if(ret == TPASS)
		printf("loopback test passed.\r\n");
	else
		printf("loopback test failed.\r\n");

	return 0;

#if PRINTF_REDIRECT
error_UART_poweroff:
	/* Received error Power off UART peripheral */
	ret = USARTdrv->PowerControl(ARM_POWER_OFF);
	if(ret != ARM_DRIVER_OK)
	{
		/* Error in UART Power OFF. */
		printf("ERROR: Could not power OFF UART\n");
	}

error_UART_uninitialize:
	/* Received error Un-initialize UART driver */
	ret = USARTdrv->Uninitialize();
	if(ret != ARM_DRIVER_OK)
	{
		/* Error in UART Uninitialize. */
		printf("ERROR: Could not uninitialize UART\n");
	}
#endif

	return 0;
}

/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
