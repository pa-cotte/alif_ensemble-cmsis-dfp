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
 * @file     FOCUS_LCD_PANEL_800_480.c
 * @author   Girish BN
 * @email    girish.bn@alifsemi.com
 * @version  V1.0.0
 * @date     30-Sep-2021
 * @brief    Display driver source file.
 * @bug      None.
 * @Note     None
 ******************************************************************************/

#include "display.h"
#include "Driver_CDC200.h"
#include "Driver_PINMUX_AND_PINPAD.h"

const display_panel focus_lcd =
	{
		.height         = 480,
		.width          = 800,
		.hsync          = 1,
		.vsync          = 1,
		.h_back_porch   = 46,
		.v_back_porch   = 23,
		.h_front_porch  = 210,
		.v_front_porch  = 22
	};

void pin_mux_config(void)
{
	PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_0, PINMUX_ALTERNATE_FUNCTION_5);

	PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_1, PINMUX_ALTERNATE_FUNCTION_5);

	PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_2, PINMUX_ALTERNATE_FUNCTION_5);

	PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_3, PINMUX_ALTERNATE_FUNCTION_5);

	PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_4, PINMUX_ALTERNATE_FUNCTION_5);

	PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_5, PINMUX_ALTERNATE_FUNCTION_5);

	PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_6, PINMUX_ALTERNATE_FUNCTION_5);

	PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_7, PINMUX_ALTERNATE_FUNCTION_5);

	PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_8, PINMUX_ALTERNATE_FUNCTION_5);

	PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_9, PINMUX_ALTERNATE_FUNCTION_5);

	PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_10, PINMUX_ALTERNATE_FUNCTION_5);

	PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_11, PINMUX_ALTERNATE_FUNCTION_5);

	PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_12, PINMUX_ALTERNATE_FUNCTION_6);

	PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_13, PINMUX_ALTERNATE_FUNCTION_6);

	PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_14, PINMUX_ALTERNATE_FUNCTION_5);

	PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_15, PINMUX_ALTERNATE_FUNCTION_5);

	PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_16, PINMUX_ALTERNATE_FUNCTION_5);

	PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_17, PINMUX_ALTERNATE_FUNCTION_5);

	PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_18, PINMUX_ALTERNATE_FUNCTION_5);

	PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_19, PINMUX_ALTERNATE_FUNCTION_5);

	PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_20, PINMUX_ALTERNATE_FUNCTION_5);

	PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_21, PINMUX_ALTERNATE_FUNCTION_6);

	PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_22, PINMUX_ALTERNATE_FUNCTION_6);

	PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_23, PINMUX_ALTERNATE_FUNCTION_5);

	PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_24, PINMUX_ALTERNATE_FUNCTION_5);

	PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_25, PINMUX_ALTERNATE_FUNCTION_5);

	PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_26, PINMUX_ALTERNATE_FUNCTION_5);

	PINMUX_Config (PORT_NUMBER_1, PIN_NUMBER_27, PINMUX_ALTERNATE_FUNCTION_5);
}

/**
 \fn            int32_t graphics_setup (uint32_t image_buff_address, uint32_t image_format);
 \brief         Configures the graphics to display.
 \param[in]     image_buff_address: Image stored memory starting address.
 \param[in]     image_format: Image standard or type.
 \param[out]    execution status.
 */
int32_t graphics_setup (uint32_t image_buff_address, uint32_t image_format)
{
	pin_mux_config();

	return display_controller_setup(image_buff_address, image_format, &focus_lcd);
}
