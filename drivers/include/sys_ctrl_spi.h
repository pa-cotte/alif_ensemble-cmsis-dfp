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
 * @file     sys_ctrl_spi.h
 * @author   Manoj A Murudi
 * @email    manoj.murudi@alifsemi.com
 * @version  V1.0.0
 * @date     24-04-2023
 * @brief    SPI system control Specific Header file.
 ******************************************************************************/

#ifndef SYS_CTRL_SPI_H_
#define SYS_CTRL_SPI_H_

#ifdef  __cplusplus
extern "C"
{
#endif

#include "peripheral_types.h"

/**
  \fn          static inline void ctrl_spi_clk (const SPI_RESOURCES *SPI, bool enable)
  \brief       Enable/Disable SPI input clock
  \param[in]   instance  spi instance
  \param[in]   enable    Enable/Disable control
  \return      none
*/
static inline void ctrl_spi_clk (SPI_INSTANCE instance, bool enable)
{
    switch (instance)
    {
        case SPI_INSTANCE_0:
        {
            if (enable)
            {
                CLKCTL_PER_SLV->SSI_CTRL |= (SSI_CTRL_SS_IN_VAL_0 | SSI_CTRL_SS_IN_SEL_0);
            }
            else
            {
                CLKCTL_PER_SLV->SSI_CTRL &= ~(SSI_CTRL_SS_IN_VAL_0 | SSI_CTRL_SS_IN_SEL_0);
            }
            break;
        }
        case SPI_INSTANCE_1:
        {
            if (enable)
            {
                CLKCTL_PER_SLV->SSI_CTRL |= (SSI_CTRL_SS_IN_VAL_1 | SSI_CTRL_SS_IN_SEL_1);
            }
            else
            {
                CLKCTL_PER_SLV->SSI_CTRL &= ~(SSI_CTRL_SS_IN_VAL_1 | SSI_CTRL_SS_IN_SEL_1);
            }
            break;
        }
        case SPI_INSTANCE_2:
        {
            if (enable)
            {
                CLKCTL_PER_SLV->SSI_CTRL |= (SSI_CTRL_SS_IN_VAL_2 | SSI_CTRL_SS_IN_SEL_2);
            }
            else
            {
                CLKCTL_PER_SLV->SSI_CTRL &= ~(SSI_CTRL_SS_IN_VAL_2 | SSI_CTRL_SS_IN_SEL_2);
            }
            break;
        }
        case SPI_INSTANCE_3:
        {
            if (enable)
            {
                CLKCTL_PER_SLV->SSI_CTRL |= (SSI_CTRL_SS_IN_VAL_3 | SSI_CTRL_SS_IN_SEL_3);
            }
            else
            {
                CLKCTL_PER_SLV->SSI_CTRL &= ~(SSI_CTRL_SS_IN_VAL_3 | SSI_CTRL_SS_IN_SEL_3);
            }
            break;
        }
    }
}

#ifdef __cplusplus
}
#endif
#endif /* SYS_CTRL_SPI_H_ */
