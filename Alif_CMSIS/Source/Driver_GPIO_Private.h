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
 * @file     Driver_GPIO_Private.h
 * @author   Girish BN, Manoj A Murudi
 * @email    girish.bn@alifsemi.com, manoj.murudi@alifsemi.com
 * @version  V1.0.0
 * @date     29-March-2023
 * @brief    Header file for GPIO.
 * @bug      None.
 * @Note	 None
 ******************************************************************************/

#ifndef DRIVER_GPIO_PRIVATE_H_
#define DRIVER_GPIO_PRIVATE_H_

#ifdef  __cplusplus
extern "C"
{
#endif

#include "RTE_Device.h"
#include "RTE_Components.h"
#include CMSIS_device_header

#include "Driver_GPIO.h"
#include "gpio.h"

#define GPIO_PORT_MAX_PIN_NUMBER                0x8U     /* Number of pins in each port */

#define GPIO_CTRL_DB_CKEN_POS                   0x12U    /* Position of Debounce clock enable bit */

#define ARM_GPIO_BIT_IRQ_POLARITY_Pos           0U       ///< bits - 0
#define ARM_GPIO_BIT_IRQ_POLARITY_Msk           (1U << ARM_GPIO_BIT_IRQ_POLARITY_Pos)
#define ARM_GPIO_BIT_IRQ_POLARITY(x)            (((x)& ARM_GPIO_BIT_IRQ_POLARITY_Msk) >> ARM_GPIO_BIT_IRQ_POLARITY_Pos)

#define ARM_GPIO_BIT_IRQ_BOTH_EDGE_Pos          1U       ///< bits - 1
#define ARM_GPIO_BIT_IRQ_BOTH_EDGE_Msk          (1U << ARM_GPIO_BIT_IRQ_BOTH_EDGE_Pos)
#define ARM_GPIO_BIT_IRQ_BOTH_EDGE(x)           (((x)& ARM_GPIO_BIT_IRQ_BOTH_EDGE_Msk) >> ARM_GPIO_BIT_IRQ_BOTH_EDGE_Pos)

#define ARM_GPIO_BIT_IRQ_SENSITIVE_Pos          2U       ///< bits - 2
#define ARM_GPIO_BIT_IRQ_SENSITIVE_Msk          (1U << ARM_GPIO_BIT_IRQ_SENSITIVE_Pos)
#define ARM_GPIO_BIT_IRQ_SENSITIVE(x)           (((x)& ARM_GPIO_BIT_IRQ_SENSITIVE_Msk) >> ARM_GPIO_BIT_IRQ_SENSITIVE_Pos)

#define GPIO0_NUM                               0U
#define GPIO1_NUM                               1U
#define GPIO2_NUM                               2U
#define GPIO3_NUM                               3U
#define GPIO4_NUM                               4U
#define GPIO5_NUM                               5U
#define GPIO6_NUM                               6U
#define GPIO7_NUM                               7U
#define GPIO8_NUM                               8U
#define GPIO9_NUM                               9U
#define GPIO10_NUM                              10U
#define GPIO11_NUM                              11U
#define GPIO12_NUM                              12U
#define GPIO13_NUM                              13U
#define GPIO14_NUM                              14U
#define GPIO15_NUM                              15U

#define CLKCTL_PER_SLV_GPIO_CTRL_BASE           0x4902F080UL

/**
 * enum GPIO_FLAG_TYPE.
 * GPIO driver status flags.
 */
typedef enum _GPIO_FLAG_TYPE
{
    GPIO_DRV_FLAG_INITIALIZED    = (1U << 0U),     /* GPIO Driver is Initialized */
    GPIO_DRV_FLAG_POWER_DONE     = (1U << 1U),     /* GPIO Driver is Powered     */
} GPIO_FLAG_TYPE;

typedef struct _GPIO_DRV_STATE {
    uint32_t initialized : 1; /* Driver Initialized*/
    uint32_t powered     : 1; /* Driver powered */
    uint32_t reserved    : 30;/* Reserved */
} GPIO_DRV_STATE;

/**
  * @brief GPIO Resources
  */
typedef struct _GPIO_RESOURCES {
    LPGPIO_Type         *reg_base;                               /**< GPIO PORT Base Address>**/
    IRQn_Type           IRQ_base_num;                            /**< GPIO PORT IRQ base Num>**/
    uint16_t            db_clkdiv;                               /**< GPIO PORT debounce clk divisor: only for GPIO 0-14 >**/
    uint8_t             gpio_id;                                 /**< GPIO instance >*/
    uint8_t             IRQ_priority[GPIO_PORT_MAX_PIN_NUMBER];  /**< GPIO PIN IRQ priority >**/
    GPIO_DRV_STATE      state;                                   /**< GPIO PORT status flag >**/
    ARM_GPIO_SignalEvent_t cb_event[GPIO_PORT_MAX_PIN_NUMBER];   /**< GPIO Call back function >*/
} GPIO_RESOURCES;

/**
  \fn          static void inline GPIO_Debounce_Enable (GPIO_RESOURCES *GPIO)
  \brief       Enable Debounce clock from EXPMST0.
  \param       GPIO     Pointer GPIO resource structure
  \return      none
*/
static void inline GPIO_Debounce_Enable (GPIO_RESOURCES *GPIO)
{
    volatile uint32_t *gpio_ctrl = ((volatile uint32_t*)CLKCTL_PER_SLV_GPIO_CTRL_BASE) + GPIO->gpio_id;

    /* config debounce clock divisor */
    *gpio_ctrl |= GPIO->db_clkdiv;

    /* enable EXPMST0 GPIO de-bounce clock. */
    *gpio_ctrl |= (1U << GPIO_CTRL_DB_CKEN_POS);
}

/**
  \fn          static inline void GPIO_Debounce_Disable (GPIO_RESOURCES *GPIO)
  \brief       Disable Debounce clock from EXPMST0.
  \param       GPIO     Pointer GPIO resource structure
  \param       pin_no   pin number
  \return      none
*/
static void inline GPIO_Debounce_Disable (GPIO_RESOURCES *GPIO)
{
    volatile uint32_t *gpio_ctrl = ((volatile uint32_t*)CLKCTL_PER_SLV_GPIO_CTRL_BASE) + GPIO->gpio_id;

    /* disable EXPMST0 GPIO de-bounce clock. */
    *gpio_ctrl &= ~(1U << GPIO_CTRL_DB_CKEN_POS);
}

#ifdef  __cplusplus
}
#endif

#endif /* DRIVER_GPIO_PRIVATE_H_ */

/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
