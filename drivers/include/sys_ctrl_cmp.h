/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

#ifndef SYS_CTRL_CMP_H_
#define SYS_CTRL_CMP_H_

#ifdef  __cplusplus
extern "C"
{
#endif

#include "peripheral_types.h"

/**
  \fn          static inline void CmpCoreClkControl(const CMP_RESOURCES *CMP, bool enable)
  \brief       Enable/Disable CMP input clock
  \param[in]   CMP       Pointer to the CMP_RESOURCES structure
  \param[in]   enable    Enable/Disable CMP clock control
  \return      none
*/
static inline void CmpCoreClkControl(const CMP_RESOURCES *CMP, bool enable)
{
    switch(CMP->drv_instance)
    {
        case CMP_INSTANCE_0:
        {
            if (enable)
            {
                CLKCTL_PER_SLV->CMP_CTRL |= CMP_CTRL_CMP0_CLKEN;
            }
            else
            {
                CLKCTL_PER_SLV->CMP_CTRL &= ~(CMP_CTRL_CMP0_CLKEN);
            }
            break;
        }

        case CMP_INSTANCE_1:
        {
            if (enable)
            {
                CLKCTL_PER_SLV->CMP_CTRL |= CMP_CTRL_CMP1_CLKEN;
            }
            else
            {
                CLKCTL_PER_SLV->CMP_CTRL &= ~(CMP_CTRL_CMP1_CLKEN);
            }
            break;
        }

        case CMP_INSTANCE_2:
        {
            if (enable)
            {
                CLKCTL_PER_SLV->CMP_CTRL |= CMP_CTRL_CMP2_CLKEN;
            }
            else
            {
                CLKCTL_PER_SLV->CMP_CTRL &= ~(CMP_CTRL_CMP2_CLKEN);
            }
            break;
        }

        case CMP_INSTANCE_3:
        {
            if (enable)
            {
                CLKCTL_PER_SLV->CMP_CTRL |= CMP_CTRL_CMP3_CLKEN;
            }
            else
            {
                CLKCTL_PER_SLV->CMP_CTRL &= ~(CMP_CTRL_CMP3_CLKEN);
            }
            break;
        }
    }
}

#endif /* SYS_CTRL_CMP_H_ */
