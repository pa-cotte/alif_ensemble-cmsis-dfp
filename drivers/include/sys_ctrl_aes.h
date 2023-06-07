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
 * @file     sys_ctrl_aes.h
 * @author   Silesh C V
 * @email    silesh@alifsemi.com
 * @version  V1.0.0
 * @date     30-May-2023
 * @brief    AES control.
 * @bug      None.
 * @Note     None
 ******************************************************************************/

#ifndef SYS_CTRL_AES_H
#define SYS_CTRL_AES_H

#include <stdint.h>
#include "peripheral_types.h"

#ifdef  __cplusplus
extern "C"
{
#endif
static inline void set_aes_rxds_delay(OSPI_INSTANCE instance, uint8_t rxds_delay)
{
    switch(instance)
    {
    case OSPI_INSTANCE_0:
        AES0->AES_RXDS_DELAY = rxds_delay;
        break;
    case OSPI_INSTANCE_1:
        AES1->AES_RXDS_DELAY = rxds_delay;
        break;
    }
}

#ifdef  __cplusplus
}
#endif
#endif /* SYS_CTRL_AES_H */
