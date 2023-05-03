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
 * @file     : Parallel_Display_Test_Baremetal.c
 * @author   : Chandra Bhushan Singh
 * @email    : chandrabhushan.singh@alifsemi.com
 * @version  : V1.0.0
 * @date     : 28-Feb-2023
 * @brief    : Baremetal demo application code for parallel display
 * @bug      : None.
 * @Note     : None.
 ******************************************************************************/

/* System Includes */
#include <stdio.h>
#include <string.h>

/* PINMUX Driver */
#include "Driver_PINMUX_AND_PINPAD.h"

#include <RTE_Device.h>
#include <RTE_Components.h>
#include CMSIS_device_header

/* include the CDC200 driver */
#include "Driver_CDC200.h"

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

#define DIMAGE_X       (RTE_PANEL_HACTIVE_TIME)
#define DIMAGE_Y       (RTE_PANEL_VACTIVE_LINE)

#if RTE_CDC200_PIXEL_FORMAT == 0
/* ARGB8888 32-bit Format (4-bytes) */
#define PIXEL_BYTES    (4)

#elif RTE_CDC200_PIXEL_FORMAT == 1
/*  RGB888 24-bit Format (3-bytes) */
#define PIXEL_BYTES    (3)

#elif RTE_CDC200_PIXEL_FORMAT == 2
/*  RGB565  16-bit Format (2-bytes) */
#define PIXEL_BYTES    (2)

#endif

static uint8_t lcd_image[DIMAGE_Y][DIMAGE_X][PIXEL_BYTES] __attribute__((section("lcd_frame_buf"))) = {0};

/* CDC200 driver instance */
extern ARM_DRIVER_CDC200 Driver_CDC200;
static ARM_DRIVER_CDC200 *CDCdrv = &Driver_CDC200;

/**
 *    @func         : void display_callback()
 *    @brief        : Parallel display demo callback
 *                  - normally is not called
 *    @return       : NONE
*/
static void display_callback(uint32_t event)
{
	if(event & ARM_CDC_DSI_ERROR_EVENT)
	{
		/* Transfer Error: Received Hardware error */
		while(1);
	}
}

/**
  \fn          int cdc200_pinmux(void)
  \brief       cdc hardware pin initialization:
  - PIN-MUX configuration
  \param[in]   none
  \return      0:success; -1:failure
  */
int cdc200_pinmux(void)
{
	int ret;

	/* Configure Pin : P1_0 as CDC_PCLK */
	ret = PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_0, PINMUX_ALTERNATE_FUNCTION_5);
	if(ret != ARM_DRIVER_OK)
	{
		printf("\r\n Error: cdc200 PINMUX failed.\r\n");
		return -1;
	}

	/* Configure Pin : P1_1 as CDC_D0 */
	ret = PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_1, PINMUX_ALTERNATE_FUNCTION_5);
	if(ret != ARM_DRIVER_OK)
	{
		printf("\r\n Error: cdc200 PINMUX failed.\r\n");
		return -1;
	}

	/* Configure Pin : P1_2 as CDC_D1 */
	ret = PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_2, PINMUX_ALTERNATE_FUNCTION_5);
	if(ret != ARM_DRIVER_OK)
	{
		printf("\r\n Error: cdc200 PINMUX failed.\r\n");
		return -1;
	}

	/* Configure Pin : P1_3 as CDC_D2 */
	ret = PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_3, PINMUX_ALTERNATE_FUNCTION_5);
	if(ret != ARM_DRIVER_OK)
	{
		printf("\r\n Error: cdc200 PINMUX failed.\r\n");
		return -1;
	}

	/* Configure Pin : P1_4 as CDC_D3 */
	ret = PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_4, PINMUX_ALTERNATE_FUNCTION_5);
	if(ret != ARM_DRIVER_OK)
	{
		printf("\r\n Error: cdc200 PINMUX failed.\r\n");
		return -1;
	}

	/* Configure Pin : P1_5 as CDC_D4 */
	ret = PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_5, PINMUX_ALTERNATE_FUNCTION_5);
	if(ret != ARM_DRIVER_OK)
	{
		printf("\r\n Error: cdc200 PINMUX failed.\r\n");
		return -1;
	}

	/* Configure Pin : P1_6 as CDC_D5 */
	ret = PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_6, PINMUX_ALTERNATE_FUNCTION_5);
	if(ret != ARM_DRIVER_OK)
	{
		printf("\r\n Error: cdc200 PINMUX failed.\r\n");
		return -1;
	}

	/* Configure Pin : P1_7 as CDC_D6 */
	ret = PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_7, PINMUX_ALTERNATE_FUNCTION_5);
	if(ret != ARM_DRIVER_OK)
	{
		printf("\r\n Error: cdc200 PINMUX failed.\r\n");
		return -1;
	}

	/* Configure Pin : P1_8 as CDC_D7 */
	ret = PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_8, PINMUX_ALTERNATE_FUNCTION_5);
	if(ret != ARM_DRIVER_OK)
	{
		printf("\r\n Error: cdc200 PINMUX failed.\r\n");
		return -1;
	}

	/* Configure Pin : P1_9 as CDC_D8 */
	ret = PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_9, PINMUX_ALTERNATE_FUNCTION_5);
	if(ret != ARM_DRIVER_OK)
	{
		printf("\r\n Error: cdc200 PINMUX failed.\r\n");
		return -1;
	}

	/* Configure Pin : P1_10 as CDC_D9 */
	ret = PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_10, PINMUX_ALTERNATE_FUNCTION_5);
	if(ret != ARM_DRIVER_OK)
	{
		printf("\r\n Error: cdc200 PINMUX failed.\r\n");
		return -1;
	}

	/* Configure Pin : P1_11 as CDC_D10 */
	ret = PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_11, PINMUX_ALTERNATE_FUNCTION_5);
	if(ret != ARM_DRIVER_OK)
	{
		printf("\r\n Error: cdc200 PINMUX failed.\r\n");
		return -1;
	}

	/* Configure Pin : P1_12 as CDC_D11 */
	ret = PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_12, PINMUX_ALTERNATE_FUNCTION_6);
	if(ret != ARM_DRIVER_OK)
	{
		printf("\r\n Error: cdc200 PINMUX failed.\r\n");
		return -1;
	}

	/* Configure Pin : P1_13 as CDC_D12 */
	ret = PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_13, PINMUX_ALTERNATE_FUNCTION_6);
	if(ret != ARM_DRIVER_OK)
	{
		printf("\r\n Error: cdc200 PINMUX failed.\r\n");
		return -1;
	}

	/* Configure Pin : P1_14 as CDC_D13 */
	ret = PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_14, PINMUX_ALTERNATE_FUNCTION_5);
	if(ret != ARM_DRIVER_OK)
	{
		printf("\r\n Error: cdc200 PINMUX failed.\r\n");
		return -1;
	}

	/* Configure Pin : P1_15 as CDC_D14 */
	ret = PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_15, PINMUX_ALTERNATE_FUNCTION_5);
	if(ret != ARM_DRIVER_OK)
	{
		printf("\r\n Error: cdc200 PINMUX failed.\r\n");
		return -1;
	}

	/* Configure Pin : P1_16 as CDC_D15 */
	ret = PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_16, PINMUX_ALTERNATE_FUNCTION_5);
	if(ret != ARM_DRIVER_OK)
	{
		printf("\r\n Error: cdc200 PINMUX failed.\r\n");
		return -1;
	}

	/* Configure Pin : P1_17 as CDC_D16 */
	ret = PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_17, PINMUX_ALTERNATE_FUNCTION_5);
	if(ret != ARM_DRIVER_OK)
	{
		printf("\r\n Error: cdc200 PINMUX failed.\r\n");
		return -1;
	}

	/* Configure Pin : P1_18 as CDC_D17 */
	ret = PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_18, PINMUX_ALTERNATE_FUNCTION_5);
	if(ret != ARM_DRIVER_OK)
	{
		printf("\r\n Error: cdc200 PINMUX failed.\r\n");
		return -1;
	}

	/* Configure Pin : P1_19 as CDC_D18 */
	ret = PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_19, PINMUX_ALTERNATE_FUNCTION_5);
	if(ret != ARM_DRIVER_OK)
	{
		printf("\r\n Error: cdc200 PINMUX failed.\r\n");
		return -1;
	}

	/* Configure Pin : P1_20 as CDC_D19 */
	ret = PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_20, PINMUX_ALTERNATE_FUNCTION_5);
	if(ret != ARM_DRIVER_OK)
	{
		printf("\r\n Error: cdc200 PINMUX failed.\r\n");
		return -1;
	}

	/* Configure Pin : P1_21 as CDC_D20 */
	ret = PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_21, PINMUX_ALTERNATE_FUNCTION_6);
	if(ret != ARM_DRIVER_OK)
	{
		printf("\r\n Error: cdc200 PINMUX failed.\r\n");
		return -1;
	}

	/* Configure Pin : P1_22 as CDC_D21 */
	ret = PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_22, PINMUX_ALTERNATE_FUNCTION_6);
	if(ret != ARM_DRIVER_OK)
	{
		printf("\r\n Error: cdc200 PINMUX failed.\r\n");
		return -1;
	}

	/* Configure Pin : P1_23 as CDC_D22 */
	ret = PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_23, PINMUX_ALTERNATE_FUNCTION_5);
	if(ret != ARM_DRIVER_OK)
	{
		printf("\r\n Error: cdc200 PINMUX failed.\r\n");
		return -1;
	}

	/* Configure Pin : P1_24 as CDC_D23 */
	ret = PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_24, PINMUX_ALTERNATE_FUNCTION_5);
	if(ret != ARM_DRIVER_OK)
	{
		printf("\r\n Error: cdc200 PINMUX failed.\r\n");
		return -1;
	}

	/* Configure Pin : P1_25 as CDC_DE */
	ret = PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_25, PINMUX_ALTERNATE_FUNCTION_5);
	if(ret != ARM_DRIVER_OK)
	{
		printf("\r\n Error: cdc200 PINMUX failed.\r\n");
		return -1;
	}

	/* Configure Pin : P1_1 as CDC_HSYNC */
	ret = PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_26, PINMUX_ALTERNATE_FUNCTION_5);
	if(ret != ARM_DRIVER_OK)
	{
		printf("\r\n Error: cdc200 PINMUX failed.\r\n");
		return -1;
	}

	/* Configure Pin : P1_1 as CDC_VSYNC */
	ret = PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_27, PINMUX_ALTERNATE_FUNCTION_5);
	if(ret != ARM_DRIVER_OK)
	{
		printf("\r\n Error: cdc200 PINMUX failed.\r\n");
		return -1;
	}

	return 0;
}

/**
 *    @func         : void Paralle_Display_Demo()
 *    @brief        : Parallel display demo
 *                  - initialize the CDC200 controller
 *                  - initialize the DSI controller and D-PHY
 *                  - initialize the LCD panel
 *                  - write various colors to the memory address.
 *    @return       : NONE
*/
static void Parallel_Display_Demo()
{

	int32_t ret       = 0;
	ARM_DRIVER_VERSION version;

	/* Hardware initialization for CDC */
	ret = cdc200_pinmux();
	if(ret != 0)
	{
		printf("\r\n Error: CDC200 Hardware Initialize failed.\r\n");
		return;
	}

	printf("\r\n >>> CDC demo starting up!!! <<< \r\n");

	version = CDCdrv->GetVersion();
	printf("\r\n CDC version api:%X driver:%X...\r\n",version.api, version.drv);

	/* Initialize CDC driver */
	ret = CDCdrv->Initialize(display_callback);
	if(ret != ARM_DRIVER_OK){
		printf("\r\n Error: CDC init failed\n");
		return;
	}

	/* Power control CDC */
	ret = CDCdrv->PowerControl(ARM_POWER_FULL);
	if(ret != ARM_DRIVER_OK){
		printf("\r\n Error: CDC Power up failed\n");
		goto error_uninitialize;
	}

	/* configure CDC controller */
	ret = CDCdrv->Control(CDC200_CONFIGURE_DISPLAY, (uint32_t)lcd_image);
	if(ret != ARM_DRIVER_OK){
		printf("\r\n Error: CDC controller configuration failed\n");
		goto error_uninitialize;
	}

	printf(">>> Allocated memory buffer Address is 0x%X <<<\n",(uint32_t)lcd_image);

	/* Start CDC */
	ret = CDCdrv->Start();
	if(ret != ARM_DRIVER_OK){
		printf("\r\n Error: CDC Start failed\n");
		goto error_poweroff;
	}

	while(1){
		memset(lcd_image, 0x00, sizeof(lcd_image));
		PMU_delay_loop_us(2 * 1000 * 1000);
		memset(lcd_image, 0xFF, sizeof(lcd_image));
		PMU_delay_loop_us(2 * 1000 * 1000);
	}

	/* Stop CDC */
	ret = CDCdrv->Stop();
	if(ret != ARM_DRIVER_OK){
		printf("\r\n Error: CDC Stop failed\n");
		goto error_poweroff;
	}

error_poweroff:

	/* Power off ADC peripheral */
	ret = CDCdrv->PowerControl(ARM_POWER_OFF);
	if(ret != ARM_DRIVER_OK)
	{
		printf("\r\n Error: CDC Power OFF failed.\r\n");
	}

error_uninitialize:

	/* Un-initialize ADC driver */
	ret = CDCdrv->Uninitialize();
	if(ret != ARM_DRIVER_OK)
	{
		printf("\r\n Error: CDC Uninitialize failed.\r\n");
	}

	printf("\r\n XXX CDC demo exiting XXX...\r\n");
}

/* Define main entry point.  */
int main()
{
	/* Enter the demo Application.  */
	Parallel_Display_Demo();
	return 0;
}
