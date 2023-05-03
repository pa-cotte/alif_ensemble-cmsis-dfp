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
 * @file     sys_ctrl_dac.h
 * @author   Nisarga A M
 * @email    nisarga.am@alifsemi.com
 * @version  V1.0.0
 * @date     27-Apr-2023
 * @brief    System Control Device information for DAC.
 * @bug      None.
 * @Note     None
 ******************************************************************************/

#ifndef SYS_CTRL_DAC_H_
#define SYS_CTRL_DAC_H_

#ifdef  __cplusplus
extern "C"
{
#endif

#include "peripheral_types.h"

/**
  \fn          static inline void DacCoreClkControl(const DAC_RESOURCES *DAC, bool enable)
  \brief       Enable/Disable DAC input clock
  \param[in]   DAC       Pointer to the DAC_RESOURCES structure
  \param[in]   enable    Enable/Disable control
  \return      none
*/
static inline void DacCoreClkControl(const DAC_RESOURCES *DAC, bool enable)
{
    switch (DAC->drv_instance)
    {
        case DAC_INSTANCE_0:
        {
            if (enable)
            {
                CLKCTL_PER_SLV->DAC_CTRL |= DAC_CTRL_DAC0_CKEN;
            }
            else {
                CLKCTL_PER_SLV->DAC_CTRL &= ~(DAC_CTRL_DAC0_CKEN);
            }
            break;
        }

        case DAC_INSTANCE_1:
        {
            if (enable)
            {
                CLKCTL_PER_SLV->DAC_CTRL |= DAC_CTRL_DAC1_CKEN;
            }
            else {
                CLKCTL_PER_SLV->DAC_CTRL &= ~(DAC_CTRL_DAC1_CKEN);
            }

            break;
        }
    }
}

#ifdef __cplusplus
}
#endif

#endif /* SYS_CTRL_DAC_H_ */
