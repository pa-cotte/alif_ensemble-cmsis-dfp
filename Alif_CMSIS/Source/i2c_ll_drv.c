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
 * @file     i2c_ll_drv.c
 * @author   Tanay Rami         | Prabhakar Kumar
 * @email    tanay@alifsemi.com | prabhakar.kumar@alifsemi.com
 * @version  V1.0.0
 * @date     20-June-2020       | 21-July-2022
 * @brief    CMSIS-Driver for i2c.
 *           This file contains lower level (HAL) apis for i2c.
 * @bug      None.
 * @Note	 None.
 ******************************************************************************/

/* Includes --------------------------------------------------------------------------- */
/* System Includes */
#include <stddef.h>
#include <stdint.h>
#include "stdio.h"

/* Project Includes */
#include "i2c_ll_drv.h"

/* Macros ----------------------------------------------------------------------------- */

/* enable transmit/receive interrupt */
#define I2C_ENABLE_TRANSMITTER_INT              (1U)    /* enable transmitter interrupt  */
#define I2C_ENABLE_RECEIVER_INT                 (2U)    /* enable receiver interrupt     */

/* disable transmit/receive interrupt */
#define I2C_DISABLE_TRANSMITTER_INT             (3U)    /* disable transmitter interrupt */
#define I2C_DISABLE_RECEIVER_INT                (4U)    /* disable receiver interrupt    */

/* Functions --------------------------------------------------------------------------- */

/**
 * @brief   Enable i2c device
 * @note    none
 * @param   i2c_reg_ptr    : Pointer to i2c register set structure
 * @retval  none
 */
static __inline void i2c_enable(i2c_reg_set_t *i2c_reg_ptr)
{
    i2c_reg_ptr->ic_enable = I2C_IC_ENABLE_I2C_ENABLE;
}

/**
 * @brief   Disable i2c device
 * @note    none
 * @param   i2c_reg_ptr    : Pointer to i2c register set structure
 * @retval  none
 */
static __inline void i2c_disable(i2c_reg_set_t *i2c_reg_ptr)
{
    i2c_reg_ptr->ic_enable = I2C_IC_ENABLE_I2C_DISABLE;
}

/**
 * @brief   check whether i2c is ready to transmit, 1 ready, 0 not ready
 * @note    none
 * @param   i2c_reg_ptr    : Pointer to i2c register set structure
 * @retval  1 ready to transmit, 0 not ready to transmit
 */
static __inline int32_t i2c_tx_ready(i2c_reg_set_t *i2c_reg_ptr)
{
    return ( (i2c_reg_ptr->ic_status & I2C_IC_STATUS_TRANSMIT_FIFO_NOT_FULL) ? 1 : 0);
}

/**
 * @brief   check whether i2c is ready to receive, 1 ready, 0 not ready
 * @note    none
 * @param   i2c_reg_ptr    : Pointer to i2c register set structure
 * @retval  1 ready to transmit, 0 not ready to transmit
 */
static __inline int32_t i2c_rx_ready(i2c_reg_set_t *i2c_reg_ptr)
{
    return ( (i2c_reg_ptr->ic_status & I2C_IC_STATUS_RECEIVE_FIFO_NOT_EMPTY) ? 1 : 0);
}

/**
 * @brief   write data to the TX FiFo buffer
 * @note    none
 * @param   i2c    : Pointer to i2c resources structure
 * @param   data   : 8-bit data to transmit + command R/W + next condition: Stop/Restart/None
 * @retval  none
 */
static __inline void i2c_write_data_cmd_next_condition_to_buffer(i2c_reg_set_t *i2c_reg_ptr,
                                                                 uint32_t      data)
{
    i2c_reg_ptr->ic_data_cmd = data;
}

/**
 * @brief   read data from RX FiFo Buffer
 * @note    none
 * @param   i2c_reg_ptr    : Pointer to i2c register set structure
 * @retval  received data (8-bit)
 */
static __inline uint32_t i2c_read_data_from_buffer(i2c_reg_set_t *i2c_reg_ptr)
{
    return ( (i2c_reg_ptr->ic_data_cmd) & 0xff );
}

/**
 * @brief   enable(unmask) i2c interrupt (0-mask  1-unmask)
 * @note    none
 * @param   i2c_reg_ptr : Pointer to i2c register set structure
 * @param   mask        : interrupt register bits which needs to be enable
 * @retval  none
 */
static __inline void i2c_unmask_interrupt(i2c_reg_set_t *i2c_reg_ptr, uint32_t mask)
{
    i2c_reg_ptr->ic_intr_mask |= mask;
}

/**
 * @brief   disable(mask) i2c interrupt	0-mask  1-unmask
 * @note    none
 * @param   i2c_reg_ptr : Pointer to i2c register set structure
 * @param   mask        : interrupt register bits which needs to be disable
 * @retval  none
 */
static __inline void i2c_mask_interrupt(i2c_reg_set_t *i2c_reg_ptr, uint32_t mask)
{
    i2c_reg_ptr->ic_intr_mask &= ~mask;
}

/**
 * @brief   clear all combined and individual i2c interrupt
 * @note    none
 * @param   i2c_reg_ptr    : Pointer to i2c register set structure
 * @retval  none
 */
static __inline void i2c_clear_all_interrupt(i2c_reg_set_t *i2c_reg_ptr)
{
    /* clear all combined and individual interrupt. */
    (void)i2c_reg_ptr->ic_clr_intr;
}

/** Set IC_CLK frequency by configuration the *CNT registers for different speed modes */
static __inline void i2c_set_scl_cnt(i2c_reg_set_t *i2c_reg_ptr, i2c_scl_cnt_t *scl_cnt)
{
    i2c_disable(i2c_reg_ptr);

    i2c_reg_ptr->ic_ss_scl_hcnt = scl_cnt->standard_speed_scl_hcnt;
    i2c_reg_ptr->ic_ss_scl_lcnt = scl_cnt->standard_speed_scl_lcnt;
    i2c_reg_ptr->ic_fs_scl_hcnt = scl_cnt->fast_speed_scl_hcnt;
    i2c_reg_ptr->ic_fs_scl_lcnt = scl_cnt->fast_speed_scl_lcnt;
    i2c_reg_ptr->ic_hs_scl_hcnt = scl_cnt->high_speed_scl_hcnt;
    i2c_reg_ptr->ic_hs_scl_lcnt = scl_cnt->high_speed_scl_lcnt;
    i2c_enable(i2c_reg_ptr);

}

/** Set spike suppression configuration */
static __inline void i2c_set_spike_len(i2c_reg_set_t *i2c_reg_ptr, i2c_spike_length_t *spklen)
{
    i2c_disable(i2c_reg_ptr);

    i2c_reg_ptr->ic_fs_spklen = spklen->fs_spike_length;
    i2c_reg_ptr->ic_hs_spklen = spklen->hs_spike_length;
    i2c_enable(i2c_reg_ptr);

}

void i2c_calc_spike(uint32_t clk_khz, i2c_spike_length_t *spklen)
{
    /* Reference has take from the databook section 2.15 */
    uint32_t clk_ns;
    spklen->fs_spike_length = 0;
    spklen->hs_spike_length = 0;

    if (clk_khz <= 1000000)
    {
        clk_ns = 1000000 / clk_khz;
        spklen->fs_spike_length = I2C_FS_SPIKE_LENGTH_NS/clk_ns;
        if ((I2C_FS_SPIKE_LENGTH_NS % clk_ns) != 0) {
            spklen->fs_spike_length += 1;
        }

        spklen->hs_spike_length = I2C_HS_SPIKE_LENGTH_NS/clk_ns;
        if ((I2C_HS_SPIKE_LENGTH_NS % clk_ns) != 0) {
            spklen->hs_spike_length += 1;
        }
    }
    else {
        clk_ns = clk_khz / 1000000;
        spklen->fs_spike_length = I2C_FS_SPIKE_LENGTH_NS*clk_ns;
        spklen->hs_spike_length = I2C_HS_SPIKE_LENGTH_NS*clk_ns;
    }

}

void i2c_calc_sclcnt(uint32_t clk_khz, uint32_t caploading, i2c_scl_cnt_t *scl_cnt, i2c_spike_length_t *spklen)
{

    uint32_t clk_ns;
    clk_ns = 1000000 / clk_khz;

    if (clk_khz <= 1000000) {
        /* Calculate count value for standard speed */
        scl_cnt->standard_speed_scl_hcnt = I2C_MIN_SS_HIGH_TIME_NS/clk_ns;
        if ((I2C_MIN_SS_HIGH_TIME_NS % clk_ns) != 0) {
            scl_cnt->standard_speed_scl_hcnt += 1;
        }
        scl_cnt->standard_speed_scl_lcnt = I2C_MIN_SS_LOW_TIME_NS/clk_ns;
        if ((I2C_MIN_SS_LOW_TIME_NS % clk_ns) != 0) {
            scl_cnt->standard_speed_scl_lcnt += 1;
        }
        /* Calculate count value for fast speed */
        scl_cnt->fast_speed_scl_hcnt = I2C_MIN_FS_HIGH_TIME_NS/clk_ns;
        if ((I2C_MIN_FS_HIGH_TIME_NS % clk_ns) != 0) {
            scl_cnt->fast_speed_scl_hcnt += 1;
        }
        scl_cnt->fast_speed_scl_lcnt = I2C_MIN_FS_LOW_TIME_NS/clk_ns;
        if ((I2C_MIN_FS_LOW_TIME_NS % clk_ns) != 0) {
            scl_cnt->fast_speed_scl_lcnt += 1;
        }
        /* Calculate count value for high speed */
        if (caploading == I2C_CAP_LOADING_100PF) {
            scl_cnt->high_speed_scl_hcnt = I2C_MIN_HS_100PF_HIGH_TIME_NS/clk_ns;
            if ((I2C_MIN_HS_100PF_HIGH_TIME_NS % clk_ns) != 0) {
                scl_cnt->high_speed_scl_hcnt += 1;
            }
            scl_cnt->high_speed_scl_lcnt = I2C_MIN_HS_100PF_LOW_TIME_NS/clk_ns;
            if ((I2C_MIN_HS_100PF_LOW_TIME_NS % clk_ns) != 0) {
                scl_cnt->high_speed_scl_lcnt += 1;
            }
        } else {
            scl_cnt->high_speed_scl_hcnt = I2C_MIN_HS_400PF_HIGH_TIME_NS/clk_ns;
            if ((I2C_MIN_HS_400PF_HIGH_TIME_NS % clk_ns) != 0) {
                scl_cnt->high_speed_scl_hcnt += 1;
            }
            scl_cnt->high_speed_scl_lcnt = I2C_MIN_HS_400PF_LOW_TIME_NS/clk_ns;
            if ((I2C_MIN_HS_400PF_LOW_TIME_NS % clk_ns) != 0) {
                scl_cnt->high_speed_scl_lcnt += 1;
            }
        }
    } else {
        /* Calculate count value for standard speed */
        scl_cnt->standard_speed_scl_hcnt = I2C_MIN_SS_HIGH_TIME_NS*clk_ns;
        scl_cnt->standard_speed_scl_lcnt = I2C_MIN_SS_LOW_TIME_NS*clk_ns;
        /* Calculate count value for fast speed */
        scl_cnt->fast_speed_scl_hcnt = I2C_MIN_FS_HIGH_TIME_NS*clk_ns;
        scl_cnt->fast_speed_scl_lcnt = I2C_MIN_FS_LOW_TIME_NS*clk_ns;
        /* Calculate count value for high speed */
        if (caploading == I2C_CAP_LOADING_100PF) {
            scl_cnt->high_speed_scl_hcnt = I2C_MIN_HS_100PF_HIGH_TIME_NS*clk_ns;
            scl_cnt->high_speed_scl_lcnt = I2C_MIN_HS_100PF_LOW_TIME_NS*clk_ns;
        } else {
            scl_cnt->high_speed_scl_hcnt = I2C_MIN_HS_400PF_HIGH_TIME_NS*clk_ns;
            scl_cnt->high_speed_scl_lcnt = I2C_MIN_HS_400PF_LOW_TIME_NS*clk_ns;
        }
    }
    if (scl_cnt->standard_speed_scl_hcnt < I2C_MIN_SS_SCL_HCNT(spklen->fs_spike_length)) {
        scl_cnt->standard_speed_scl_hcnt = I2C_MIN_SS_SCL_HCNT(spklen->fs_spike_length);
    }
    if (scl_cnt->standard_speed_scl_lcnt < I2C_MIN_SS_SCL_LCNT(spklen->fs_spike_length)) {
        scl_cnt->standard_speed_scl_lcnt = I2C_MIN_SS_SCL_LCNT(spklen->fs_spike_length);
    }
    if (scl_cnt->fast_speed_scl_hcnt < I2C_MIN_FS_SCL_HCNT(spklen->fs_spike_length)) {
        scl_cnt->fast_speed_scl_hcnt = I2C_MIN_FS_SCL_HCNT(spklen->fs_spike_length);
    }
    if (scl_cnt->fast_speed_scl_lcnt < I2C_MIN_FS_SCL_LCNT(spklen->fs_spike_length)) {
        scl_cnt->fast_speed_scl_lcnt = I2C_MIN_FS_SCL_LCNT(spklen->fs_spike_length);
    }
    if (scl_cnt->high_speed_scl_hcnt < I2C_MIN_HS_SCL_HCNT(spklen->hs_spike_length)) {
        scl_cnt->high_speed_scl_hcnt = I2C_MIN_HS_SCL_HCNT(spklen->hs_spike_length);
    }
    if (scl_cnt->high_speed_scl_lcnt < I2C_MIN_HS_SCL_LCNT(spklen->hs_spike_length)) {
        scl_cnt->high_speed_scl_lcnt = I2C_MIN_HS_SCL_LCNT(spklen->hs_spike_length);
    }

}

/**
 * @brief    enable master transmit and receive interrupt
 * @note     only one combined interrupt is available for TX/RX
 *           so if any one is enable then no need to enable interrupt again.
 * @param    i2c    : Pointer to i2c resources structure
 * @param    arg    : I2C_ENABLE_TRANSMITTER_INT or I2C_ENABLE_RECEIVER_INT
 * @retval   none
 */
static void i2c_enable_master_interrupt(i2c_resources_t *i2c, uint32_t arg)
{
    i2c_info_t    *i2c_info_ptr = i2c->info;
    i2c_reg_set_t *i2c_reg_ptr  = (i2c_reg_set_t *)i2c->reg_base;

    /* pass only if parameter is TX/RX enable interrupt */
    if( (arg != I2C_ENABLE_TRANSMITTER_INT) && (arg != I2C_ENABLE_RECEIVER_INT) )
        return ;

    /* enable transmitter interrupt */
    if(arg == I2C_ENABLE_TRANSMITTER_INT)
    {
        i2c_unmask_interrupt(i2c_reg_ptr, I2C_IC_INT_MST_TX_ENABLE);

        /* set TX interrupt enable flag */
        i2c_info_ptr->int_status |= I2C_FLAG_TX_INT_ENABLE;
    }

    /* enable receiver interrupt */
    if(arg == I2C_ENABLE_RECEIVER_INT)
    {
        i2c_unmask_interrupt(i2c_reg_ptr, I2C_IC_INT_MST_RX_ENABLE);

        /* set RX interrupt enable flag */
        i2c_info_ptr->int_status |= I2C_FLAG_RX_INT_ENABLE;
    }
}

/**
 * @brief   disable master transmit and receive interrupt
 * @note    only one combined interrupt is available for TX/RX
 *          so if both TX and RX are disable then only disable interrupt.
 * @param   i2c         : Pointer to i2c resources structure
 * @param   arg         : I2C_DISABLE_TRANSMITTER_INT or I2C_DISABLE_RECEIVER_INT
 * @retval  none
 */
static void i2c_disable_master_interrupt(i2c_resources_t *i2c, uint32_t arg)
{
    i2c_info_t      *i2c_info_ptr = i2c->info;
    i2c_reg_set_t   *i2c_reg_ptr  = (i2c_reg_set_t *)i2c->reg_base;

    /* Pass only if parameter is TX/RX disable interrupt */
    if( (arg != I2C_DISABLE_TRANSMITTER_INT) && (arg != I2C_DISABLE_RECEIVER_INT) )
        return;

    /* Disable transmit interrupt */
    if(arg == I2C_DISABLE_TRANSMITTER_INT)
    {
        i2c_mask_interrupt(i2c_reg_ptr, I2C_IC_INT_MST_TX_ENABLE);

        /* Reset TX interrupt enable flag */
        i2c_info_ptr->int_status &= ~I2C_FLAG_TX_INT_ENABLE;
    }

    /* Disable receiver interrupt */
    if(arg == I2C_DISABLE_RECEIVER_INT)
    {
        i2c_mask_interrupt(i2c_reg_ptr, I2C_IC_INT_MST_RX_ENABLE);

        /* Reset RX interrupt enable flag */
        i2c_info_ptr->int_status &= ~I2C_FLAG_RX_INT_ENABLE;
    }
}

static void i2c_enable_slave_interrupt(i2c_resources_t *i2c, uint32_t arg)
{
    i2c_info_t    *i2c_info_ptr = i2c->info;
    i2c_reg_set_t *i2c_reg_ptr  = (i2c_reg_set_t *)i2c->reg_base;

    /* pass only if parameter is TX/RX enable interrupt */
    if( (arg != I2C_ENABLE_TRANSMITTER_INT) && (arg != I2C_ENABLE_RECEIVER_INT) )
        return ;

    /* enable transmitter interrupt */
    if(arg == I2C_ENABLE_TRANSMITTER_INT)
    {
        i2c_unmask_interrupt(i2c_reg_ptr, I2C_IC_INT_SLV_TX_ENABLE);

        /* set TX interrupt enable flag */
        i2c_info_ptr->int_status |= I2C_FLAG_TX_INT_ENABLE;
    }

    /* enable receiver interrupt */
    if(arg == I2C_ENABLE_RECEIVER_INT)
    {
        i2c_unmask_interrupt(i2c_reg_ptr, I2C_IC_INT_SLV_RX_ENABLE);

        /* set RX interrupt enable flag */
        i2c_info_ptr->int_status |= I2C_FLAG_RX_INT_ENABLE;
    }
}

static void i2c_disable_slave_interrupt(i2c_resources_t *i2c, uint32_t arg)
{
    i2c_info_t      *i2c_info_ptr = i2c->info;
    i2c_reg_set_t   *i2c_reg_ptr  = (i2c_reg_set_t *)i2c->reg_base;

    /* Pass only if parameter is TX/RX disable interrupt */
    if( (arg != I2C_DISABLE_TRANSMITTER_INT) && (arg != I2C_DISABLE_RECEIVER_INT) )
        return;

    /* Disable transmit interrupt */
    if(arg == I2C_DISABLE_TRANSMITTER_INT)
    {
        i2c_mask_interrupt(i2c_reg_ptr, I2C_IC_INT_SLV_TX_ENABLE);

        /* Reset TX interrupt enable flag */
        i2c_info_ptr->int_status &= ~I2C_FLAG_TX_INT_ENABLE;
    }

    /* Disable receiver interrupt */
    if(arg == I2C_DISABLE_RECEIVER_INT)
    {
        i2c_mask_interrupt(i2c_reg_ptr, I2C_IC_INT_SLV_RX_ENABLE);

        /* Reset RX interrupt enable flag */
        i2c_info_ptr->int_status &= ~I2C_FLAG_RX_INT_ENABLE;
    }
}

/**
 * @brief    enable master or slave transmit and receive interrupt
 * @note     only one combined interrupt is available for TX/RX
 *           so if any one is enable then no need to enable interrupt again.
 * @note     slave is not implemented.
 * @param    i2c    : Pointer to i2c resources structure
 * @param    arg    : I2C_ENABLE_TRANSMITTER_INT or I2C_ENABLE_RECEIVER_INT
 * @retval   none
 */
static void i2c_enable_master_or_slave_interrupt(i2c_resources_t *i2c, uint32_t arg)
{
    i2c_info_t    *i2c_info_ptr = i2c->info;
    i2c_reg_set_t *i2c_reg_ptr  = (i2c_reg_set_t *)i2c->reg_base;

    if (i2c_info_ptr->mode == I2C_MASTER_MODE)
    {
        /* master */
        i2c_enable_master_interrupt(i2c, arg);
    }
    else
    {
        /* slave */
        i2c_enable_slave_interrupt(i2c, arg);
    }

    /* only one combined interrupt is available for TX/RX
     * so if any one is enable then no need to enable interrupt again.
     */

    /* if TX and RX both are disable then only enable interrupt.*/
    if ((i2c_info_ptr->int_status & I2C_FLAG_TX_OR_RX_INT_ENABLE) == 0) /* TX and RX both are disable. */
    {
        /* anyone should be enable either TX or RX */
        if ( (i2c_info_ptr->int_status & I2C_FLAG_TX_INT_ENABLE) ||
             (i2c_info_ptr->int_status & I2C_FLAG_RX_INT_ENABLE) )
        {
            if (i2c->irq_num != I2C_INVALID_INT_NUM)
            {
                /* NVICSetpriority */
                if (i2c->irq_priority != I2C_INVALID_PRIORITY)
                {
                    NVIC_SetPriority(i2c->irq_num, i2c->irq_priority);
                    if (NVIC_GetPriority(i2c->irq_num) != i2c->irq_priority)
                    {
                         return; /* error */
                    }
                }

                /* enable the NVIC interrupt. */
                NVIC_EnableIRQ(i2c->irq_num);
            }

            /* set the global flag as TX or RX interrupt enable. */
            i2c_info_ptr->int_status |= I2C_FLAG_TX_OR_RX_INT_ENABLE;
        }
    }
    /* else interrupt is already enabled. */
}

/**
 * @brief   disable master or slave transmit and receive interrupt
 * @note    only one combined interrupt is available for TX/RX
 *          so if both TX and RX are disable then only disable interrupt.
 * @note    slave is not implemented.
 * @param   i2c    : Pointer to i2c resources structure
 * @param   arg    : I2C_DISABLE_TRANSMITTER_INT or I2C_DISABLE_RECEIVER_INT
 * @retval  none
 */
static void i2c_disable_master_or_slave_interrupt(i2c_resources_t *i2c, uint32_t arg)
{
    i2c_info_t     *i2c_info_ptr = i2c->info;
    i2c_reg_set_t  *i2c_reg_ptr  = (i2c_reg_set_t *)i2c->reg_base;

    if (i2c_info_ptr->mode == I2C_MASTER_MODE)
    {
        /* master */
        i2c_disable_master_interrupt(i2c, arg);
    }
    else
    {
        /* slave */
        i2c_disable_slave_interrupt(i2c, arg);
    }

    /*only one combined interrupt is available for TX/RX
     * so if both TX and RX are disable then only disable interrupt
     */

    /* disable if anyone TX or RX interrupt is already enabled. */
    if (i2c_info_ptr->int_status & I2C_FLAG_TX_OR_RX_INT_ENABLE)
    {
        /* both TX and RX flag should be disable then only disable interrupt. */
        if ( ((i2c_info_ptr->int_status & I2C_FLAG_TX_INT_ENABLE) == 0) &&
             ((i2c_info_ptr->int_status & I2C_FLAG_RX_INT_ENABLE) == 0) )
        {
            if (i2c->irq_num != I2C_INVALID_INT_NUM)
            {
                /* disable the NVIC interrupt. */
                NVIC_DisableIRQ(i2c->irq_num);
            }

            /* set the global flag as TX or RX interrupt is enable. */
            i2c_info_ptr->int_status &= ~I2C_FLAG_TX_OR_RX_INT_ENABLE;
        }
    }
}

/**
 * @brief   disable both transmit and receive interrupt for master or slave
 * @note    only one combined interrupt is available for TX/RX
 *          so if both TX and RX are disable then only disable interrupt.
 * @note    slave is not implemented.
 * @param   i2c    : Pointer to i2c resources structure
 * @retval  none
 */
static void i2c_disable_interrupt(i2c_resources_t *i2c)
{
    i2c_info_t *i2c_info_ptr = i2c->info;

    /* disable i2c transmit interrupt  */
    i2c_disable_master_or_slave_interrupt(i2c, I2C_DISABLE_TRANSMITTER_INT);

    /* disable i2c receive interrupt  */
    i2c_disable_master_or_slave_interrupt(i2c, I2C_DISABLE_RECEIVER_INT);

    /* disable i2c interrupt */
    if (i2c->irq_num != I2C_INVALID_INT_NUM)
    {
        /* disable NVIC interrupt. */
        NVIC_DisableIRQ(i2c->irq_num);
    }

    /* reset the global flag for interrupt. */
    i2c->info->int_status &= ~(I2C_FLAG_TX_OR_RX_INT_ENABLE|I2C_FLAG_TX_INT_ENABLE|I2C_FLAG_RX_INT_ENABLE);
}

/**
 * @brief   check any error for master
 * @note    none
 * @param   i2c    : Pointer to i2c resources structure
 * @retval  none
 */
static int32_t i2c_master_check_error (i2c_resources_t *i2c)
{
    uint32_t status;
    int32_t ercd = I2C_ERR_NONE;

    i2c_info_t      *i2c_info_ptr = i2c->info;
    i2c_reg_set_t   *i2c_reg_ptr  = (i2c_reg_set_t *)i2c->reg_base;

    status = i2c_reg_ptr->ic_raw_intr_stat;

    /*transmit abort*/
    if (status & I2C_IC_INTR_STAT_TX_ABRT)
    {

        status = i2c_reg_ptr->ic_tx_abrt_source;

        if (status & I2C_MST_ABRT_LOST_BUS)
        {
            ercd = I2C_ERR_LOST_BUS;
        }
        else if (status & I2C_MST_ABRT_ADDR_NOACK)
        {
            ercd = I2C_ERR_ADDR_NOACK;
        }
        /* master got ack from slave for 7/10bit addr then
         * mastrer sends data, no ack for the data */
        else if (status & I2C_MST_ABRT_DATA_NOACK)
        {
            ercd = I2C_ERR_DATA_NOACK;
        }
        else
        {
            ercd = I2C_ERR_UNDEF;
        }

        status = i2c_reg_ptr->ic_clr_tx_abrt;
    }
    else
    {
        /*during transmit set once TX_fifo is at max buffer_length and processor sends another i2c cmd by writing to IC_DATA_CMD */
        if (status & I2C_IC_INTR_STAT_TX_OVER)
        {
            i2c_info_ptr->tx_over ++;

            /* clear reg */
            status = i2c_reg_ptr->ic_clr_tx_over;
        }

        /* RX_fifo full and received one more byte | RX_fifo is empty and trying to read from ic_data_cmd */
        if (status & (I2C_IC_INTR_STAT_RX_OVER|I2C_IC_INTR_STAT_RX_UNDER))
        {
            i2c_info_ptr->rx_over ++;

            /* clear reg */
            status = i2c_reg_ptr->ic_clr_rx_over;
            status = i2c_reg_ptr->ic_clr_rx_under;
        }
    }
    return ercd;
}
/**
 * @brief   check any error for slave
 * @note    none
 * @param   i2c    : Pointer to i2c resources structure
 * @retval  none
 */
static int32_t i2c_slave_check_error (i2c_resources_t *i2c)
{
    uint32_t status;
    int32_t ercd = I2C_ERR_NONE;

    i2c_info_t      *i2c_info_ptr = i2c->info;
    i2c_reg_set_t   *i2c_reg_ptr  = (i2c_reg_set_t *)i2c->reg_base;

    status = i2c_reg_ptr->ic_raw_intr_stat;

    if (status & I2C_IC_INTR_STAT_START_DET)
    {
        status = i2c_reg_ptr->ic_clr_start_det;
    }

    if (status & I2C_IC_INTR_STAT_STOP_DET)
    {
        status = i2c_reg_ptr->ic_clr_stop_det;
    }

    if (status & I2C_IC_INTR_STAT_GEN_CALL)
    {
        status = i2c_reg_ptr->ic_clr_gen_call;
    }

    if (status & I2C_IC_INTR_STAT_TX_OVER)
    {
        i2c_info_ptr->tx_over ++;
        status = i2c_reg_ptr->ic_clr_tx_over;
    }

    if (status & (I2C_IC_INTR_STAT_RX_OVER|I2C_IC_INTR_STAT_RX_UNDER))
    {
        i2c_info_ptr->rx_over ++;
        status = i2c_reg_ptr->ic_clr_rx_over;
        status = i2c_reg_ptr->ic_clr_rx_under;
    }
    if (status & I2C_IC_INTR_STAT_TX_ABRT)
    {
        status = i2c_reg_ptr->ic_tx_abrt_source;
        if(status & I2C_SLV_ABRT_LOST_BUS)
        {
            ercd = I2C_ERR_LOST_BUS;
        }

        status = i2c_reg_ptr->ic_clr_tx_abrt;
    }

    return ercd;
}

/**
 * @brief    enable i2c device
 * @note     none
 * @param    i2c    : Pointer to i2c resources structure
 * @retval   none
 */
static void i2c_enable_device(i2c_resources_t *i2c)
{
    i2c_info_t     *i2c_info_ptr = i2c->info;
    i2c_reg_set_t  *i2c_reg_ptr  = (i2c_reg_set_t *)i2c->reg_base;
    i2c_enable(i2c_reg_ptr);
}

/**
 * @brief    disable i2c device
 * @note     none
 * @param    i2c    : Pointer to i2c resources structure
 * @retval   none
 */
static void i2c_disable_device(i2c_resources_t *i2c)
{
    uint32_t i;
    i2c_info_t      *i2c_info_ptr = i2c->info;
    i2c_reg_set_t   *i2c_reg_ptr  = (i2c_reg_set_t *)i2c->reg_base;

    for (i=0; i < I2C_DISABLE_MAX_TIME_OUT_CNT; i++)
    {
        i2c_disable(i2c_reg_ptr);

        /* it requires some delay while disabling i2c */
        /* check whether i2c is disabled or not. */
        if ((i2c_reg_ptr->ic_enable_status & I2C_IC_ENABLE_STATUS_IC_EN) == 0)
        {
            /* i2c is disabled now. */
            break;
        }
    }
}

/**
 * @brief    reset i2c device
 * @note     none
 * @param    i2c    : Pointer to i2c resources structure
 * @retval   none
 */
void i2c_reset_device(i2c_resources_t *i2c)
{
    i2c_info_t     *i2c_info_ptr = i2c->info;
    i2c_reg_set_t  *i2c_reg_ptr  = (i2c_reg_set_t *)i2c->reg_base;

    i2c_disable_device(i2c);
    i2c_clear_all_interrupt(i2c_reg_ptr);

    i2c_info_ptr->next_cond = I2C_MODE_STOP;
    i2c_info_ptr->cur_state = I2C_FREE;
    i2c_info_ptr->err_state = I2C_ERR_NONE;
    i2c_info_ptr->tx_over = 0;
    i2c_info_ptr->rx_over = 0;

    i2c_enable_device(i2c);
}

/**
 * @brief   get i2c bus speed
 * @note    implemented only ARM_I2C_BUS_SPEED_STANDARD
 * @param   i2c_bus_speed    : i2c bus speed
 *          ARM_I2C_BUS_SPEED_STANDARD /
 *          I2C_IC_CON_SPEED_FAST / ARM_I2C_BUS_SPEED_FAST_PLUS /
 *          ARM_I2C_BUS_SPEED_HIGH
 * @retval  none
 */
static __inline int32_t i2c_get_bus_speed(uint32_t i2c_bus_speed)
{
    int32_t speed;

    switch (i2c_bus_speed)
    {
        case ARM_I2C_BUS_SPEED_STANDARD:
            /* Standard Speed (100kHz) */
            speed = I2C_IC_CON_SPEED_STANDARD;
            break;

        case ARM_I2C_BUS_SPEED_FAST:
            /* Fast Speed (400kHz) */
            speed = I2C_IC_CON_SPEED_FAST;
            break;

        case ARM_I2C_BUS_SPEED_FAST_PLUS:
            /* Fast+ Speed (1MHz) */
            speed = I2C_IC_CON_SPEED_FAST;
            break;

        case ARM_I2C_BUS_SPEED_HIGH:
            /* Fast+ Speed (3.4MHz) */
            speed = I2C_IC_CON_SPEED_HIGH;
            break;

        default:
            return ARM_DRIVER_ERROR_UNSUPPORTED;
    }
    return speed;
}

/**
 * @brief   initialize i2c master
 * @note    none
 * @param   i2c             : Pointer to i2c resources structure
 * @param   i2c_bus_speed   : i2c bus speed
 *          ARM_I2C_BUS_SPEED_STANDARD /
 *          I2C_IC_CON_SPEED_FAST /
 *          ARM_I2C_BUS_SPEED_FAST_PLUS /
 *          ARM_I2C_BUS_SPEED_HIGH
 * @retval  none
 */
int32_t i2c_master_init(i2c_resources_t *i2c, uint32_t i2c_bus_speed)
{
    uint32_t ic_con_reg_value = 0;
    uint32_t speed = 0;

    uint32_t temp_ic_con_reg_value = 0;
    uint32_t temp_ic_tar = 0;
    uint32_t temp;

    i2c_info_t      *i2c_info_ptr = i2c->info;
    i2c_reg_set_t   *i2c_reg_ptr  = (i2c_reg_set_t *)i2c->reg_base;

    uint32_t addr_mode = i2c_info_ptr->addr_mode;
    uint32_t tar_addr  = i2c_info_ptr->tar_addr;

    i2c_disable(i2c_reg_ptr);

    /* disable all i2c interrupt */
    i2c_reg_ptr->ic_intr_mask = I2C_IC_INT_DISABLE_ALL;

    /* Set to 7bit addressing and update target address */
    i2c_reg_ptr->ic_tar = (tar_addr & I2C_IC_TAR_10BIT_ADDR_MASK) | I2C_IC_TAR_SPECIAL | I2C_IC_TAR_GC_OR_START;

    speed = i2c_get_bus_speed(i2c_bus_speed);
    if(speed == ARM_DRIVER_ERROR_UNSUPPORTED)
    {
        return ARM_DRIVER_ERROR_UNSUPPORTED;
    }

    /* master mode, restart enabled */
    ic_con_reg_value = speed | I2C_IC_CON_ENABLE_MASTER_MODE | I2C_IC_CON_MASTER_RESTART_EN;

    /* Set final IC_CON value */
    i2c_reg_ptr->ic_con = ic_con_reg_value;

    /* FIFO threshold settings */
    i2c_reg_ptr->ic_tx_tl = I2C_IC_TX_TL_TX_FIFO_THRESHOLD_LVL;
    i2c_reg_ptr->ic_rx_tl = I2C_IC_RX_TL_RX_FIFO_THRESHOLD_LVL;

    /* Clock Settings */
    i2c_set_spike_len(i2c_reg_ptr, &(i2c_info_ptr->i2c_spklen));

    i2c_set_scl_cnt(i2c_reg_ptr, &(i2c_info_ptr->i2c_scl_cnt));

    /* Master code settings */
    /* only in High speed master mode */
    i2c_reg_ptr->ic_hs_maddr = 0;

    i2c_enable(i2c_reg_ptr);

    /* I2C mode set */
    i2c_info_ptr->mode = I2C_MASTER_MODE;

    return ARM_DRIVER_OK;
}
/**
 * @brief   initialize i2c slave
 * @note    none
 * @param   i2c             : Pointer to i2c resources structure
 * @param   i2c_slave_addr  : i2c slave address
 * @retval  ARM_DRIVER_OK   : success
 */
int32_t i2c_slave_init(i2c_resources_t *i2c, uint32_t i2c_slave_addr)
{
    int32_t ret = ARM_DRIVER_OK;
    int32_t ic_con_reg = 0;

    i2c_reg_set_t   *i2c_reg_ptr  = (i2c_reg_set_t *)i2c->reg_base;

    /* Disable i2c */
    i2c_disable(i2c_reg_ptr);

    /* Disable All i2c interrupt */
    i2c_reg_ptr->ic_intr_mask = I2C_IC_INT_DISABLE_ALL;

    /* Set slave address as a slave */
    i2c_reg_ptr->ic_sar = i2c_slave_addr & I2C_IC_SAR_10BIT_ADDR_MASK;

    ic_con_reg |= i2c_get_bus_speed(ARM_I2C_BUS_SPEED_STANDARD);
    if(ic_con_reg == ARM_I2C_BUS_SPEED_HIGH)
         return ARM_DRIVER_ERROR;

    ic_con_reg = I2C_IC_CON_ENA_SLAVE_MODE | I2C_IC_CON_MASTER_RESTART_EN | I2C_SLAVE_7BIT_ADDR_MODE ;

    /* slave, mode for 7 bit address */
    i2c_reg_ptr->ic_con = ic_con_reg;

    /* I2C mode set */
    i2c->info->flags = I2C_SLAVE_MODE;

    /* FIFO threshold settings */
    i2c_reg_ptr->ic_tx_tl = I2C_IC_TX_TL_TX_FIFO_THRESHOLD_LVL;
    i2c_reg_ptr->ic_rx_tl = I2C_IC_RX_TL_RX_FIFO_THRESHOLD_LVL;

    /* Enable i2c */
    i2c_enable(i2c_reg_ptr);

    return ret;
}

/**
 * @brief   abort transmit
 * @note    used in interrupt method only
 * @param   i2c    : Pointer to i2c resources structure
 * @retval  none
 */
void i2c_abort_transmit(i2c_resources_t *i2c)
{
    i2c_info_t      *i2c_info_ptr = i2c->info;
    i2c_reg_set_t   *i2c_reg_ptr  = (i2c_reg_set_t *)i2c->reg_base;

    /* if tx interrupt flag is enable then only disable transmit interrupt */
    if (i2c_info_ptr->int_status & I2C_FLAG_TX_INT_ENABLE)
    {
        i2c_disable_master_or_slave_interrupt(i2c, I2C_DISABLE_TRANSMITTER_INT);
        i2c_info_ptr->cur_state = I2C_FREE;

        /* Reset the tx_buffer */
        i2c_info_ptr->transfer.tx_total_num = 0U;
    }

    /* clear busy status bit. */
    i2c_info_ptr->status.busy = 0U;
}

/**
 * @brief   abort receive
 * @note    used in interrupt method only,
 * @param   i2c    : Pointer to i2c resources structure
 * @retval  none
 */
void i2c_abort_receive(i2c_resources_t *i2c)
{
    i2c_info_t      *i2c_info_ptr = i2c->info;
    i2c_reg_set_t   *i2c_reg_ptr  = (i2c_reg_set_t *)i2c->reg_base;

    if (i2c_info_ptr->int_status & I2C_FLAG_RX_INT_ENABLE)
    {
        i2c_disable_master_or_slave_interrupt(i2c, I2C_DISABLE_RECEIVER_INT);
        i2c_info_ptr->cur_state = I2C_FREE;

        /* Reset the rx_buffer */
        i2c_info_ptr->transfer.rx_total_num = 0U;
    }

    /* clear busy status bit. */
    i2c_info_ptr->status.busy = 0U;
}

/**
 * @brief    set i2c target address for slave device in master mode
 * @note     implemented only 7-bit addr mode
 * @param    i2c        : Pointer to i2c resources structure
 * @param    address    : i2c 7-bit or 10-bit slave address
 * @retval   none
 */
static __inline void i2c_set_target_addr(i2c_resources_t *i2c,
                                         uint32_t        address)
{
    i2c_info_t      *i2c_info_ptr = i2c->info;
    i2c_reg_set_t   *i2c_reg_ptr  = (i2c_reg_set_t *)i2c->reg_base;
    uint32_t temp_read = 0;

    if (address == i2c_info_ptr->tar_addr)
    {
        /* target address is already set. */
        return;
    }

    /* update the tar reg in info structure. */
    i2c_info_ptr->tar_addr = address;
    i2c_disable(i2c_reg_ptr);

    /* clear the 10bit(0-9) of the ic_tar target register. */
    i2c_reg_ptr->ic_tar &= ~(I2C_IC_TAR_10BIT_ADDR_MASK);

    /* update the 10bit(0-9) of the ic_tar target register as per our address. */
    i2c_reg_ptr->ic_tar |= (I2C_IC_TAR_10BIT_ADDR_MASK & address);
    i2c_enable(i2c_reg_ptr);

    temp_read = i2c_reg_ptr->ic_tar;
}

/**
 * @brief   i2c master transmit data using interrupt method
 * @note    implemented only 7-bit slave addr.
 * @param   i2c                     : Pointer to i2c resources structure
 * @param   addr                    : 7-bit/10-bit slave address
 * @param   data                    : Pointer to input data which needs to be transmit
 * @param   num                     : total length of input data
 * @param   xfer_pending            : whether transfer is completed(stopped)-0 or pending(Restart)-1
 * @retval  ARM_DRIVER_ERROR_BUSY   : error previous transfer operation is in process
 * @retval  ARM_DRIVER_OK           : success
 */
int32_t i2c_enable_master_transmit_interrupt (i2c_resources_t *i2c,
                                              uint32_t        addr,
                                              const uint8_t   *data,
                                              uint32_t        num,
                                              bool            xfer_pending)
{
	int ret = ARM_DRIVER_OK;

    i2c_info_t      *i2c_info_ptr = i2c->info;
    i2c_reg_set_t   *i2c_reg_ptr  = (i2c_reg_set_t *)i2c->reg_base;

    if (i2c_info_ptr->status.busy)
    {
        /* Transfer operation in progress */
        return ARM_DRIVER_ERROR_BUSY;
    }

    /* fill the i2c transfer structure as per user detail */
    i2c_info_ptr->transfer.tx_buf        = (uint8_t *)data;
    i2c_info_ptr->transfer.tx_total_num  = num;
    i2c_info_ptr->transfer.tx_curr_cnt   = 0U;
    i2c_info_ptr->transfer.curr_cnt      = 0U;
    i2c_info_ptr->pending = xfer_pending;
    i2c_info_ptr->cur_state = I2C_IN_TX;

    /* Update driver status \ref ARM_I2C_STATUS */
    i2c_info_ptr->status.busy             = 1;
    i2c_info_ptr->status.mode             = I2C_MASTER_MODE;
    i2c_info_ptr->status.direction        = I2C_DIR_TRANSMITTER;
    i2c_info_ptr->status.arbitration_lost = 0;
    i2c_info_ptr->status.bus_error        = 0;

    i2c_info_ptr->tx_over = 0U;

    /* set target address */
    i2c_set_target_addr(i2c, addr);

    i2c_enable_master_or_slave_interrupt(i2c, I2C_ENABLE_TRANSMITTER_INT);

    return ret;
}

/**
 * @brief    i2c master receive data using interrupt method
 * @note     implemented only 7-bit slave addr.
 * @param    i2c                    : Pointer to i2c resources structure
 * @param    addr                   : 7-bit/10-bit slave address
 * @param    data                   : Pointer to input data which needs to be transmit
 * @param    num                    : total length of input data
 * @param    xfer_pending           : whether transfer is completed(stopped)-0 or pending(Restart)-1
 * @retval   ARM_DRIVER_ERROR_BUSY  : error previous transfer operation is in process
 * @retval   ARM_DRIVER_OK          : success
 */
int32_t i2c_enable_master_receive_interrupt (i2c_resources_t  *i2c,
                                             uint32_t         addr,
                                             uint8_t          *data,
                                             uint32_t         num,
                                             bool             xfer_pending)
{
    i2c_info_t     *i2c_info_ptr = i2c->info;
    i2c_reg_set_t  *i2c_reg_ptr  = (i2c_reg_set_t *)i2c->reg_base;

    if (i2c_info_ptr->status.busy)
    {
        /* Transfer operation in progress */
        return ARM_DRIVER_ERROR_BUSY;
    }

    /* fill the i2c transfer structure as per user detail */
    i2c_info_ptr->transfer.rx_buf           = (uint8_t *)data;
    i2c_info_ptr->transfer.rx_total_num     = num;
    i2c_info_ptr->transfer.rx_curr_cnt      = 0U;
    i2c_info_ptr->transfer.curr_cnt         = 0U;
    i2c_info_ptr->transfer.rx_curr_tx_index = 0U;

    i2c_info_ptr->pending = xfer_pending;
    i2c_info_ptr->cur_state = I2C_IN_RX;

    /* Update driver status \ref ARM_I2C_STATUS */
    i2c_info_ptr->status.busy             = 1;
    i2c_info_ptr->status.mode             = I2C_MASTER_MODE;
    i2c_info_ptr->status.direction        = I2C_DIR_RECEIVER;
    i2c_info_ptr->status.arbitration_lost = 0;
    i2c_info_ptr->status.bus_error        = 0;

    i2c_info_ptr->rx_over = 0U;

    /* set target address */
    i2c_set_target_addr(i2c, addr);

    i2c_enable_master_or_slave_interrupt(i2c, I2C_ENABLE_RECEIVER_INT);

    return ARM_DRIVER_OK;
}
/**
 * @brief   i2c master transmit data using interrupt method
 * @note    implemented only 7-bit slave addr.
 * @param   i2c                     : Pointer to i2c resources structure
 * @param   addr                    : 7-bit/10-bit slave address
 * @param   data                    : Pointer to input data which needs to be transmit
 * @param   num                     : total length of input data
 * @retval  ARM_DRIVER_ERROR_BUSY   : error previous transfer operation is in process
 * @retval  ARM_DRIVER_OK           : success
 */
int32_t  i2c_enable_slave_transmit_interrupt (i2c_resources_t *i2c, const uint8_t *data, uint32_t num)
{
    i2c_info_t      *i2c_info_ptr = i2c->info;
    i2c_reg_set_t   *i2c_reg_ptr  = (i2c_reg_set_t *)i2c->reg_base;

    if (i2c_info_ptr->status.busy)
    {
        /* Transfer operation in progress */
        return ARM_DRIVER_ERROR_BUSY;
    }

    /* fill the i2c transfer structure as per user detail */
    i2c_info_ptr->transfer.tx_buf        = (uint8_t *)data;
    i2c_info_ptr->transfer.tx_total_num  = num;
    i2c_info_ptr->transfer.tx_curr_cnt   = 0U;
    i2c_info_ptr->transfer.curr_cnt      = 0U;
    i2c_info_ptr->cur_state = I2C_IN_TX;

    /* Update driver status \ref ARM_I2C_STATUS */
    i2c_info_ptr->status.busy             = 1;
    i2c_info_ptr->status.mode             = I2C_SLAVE_MODE;
    i2c_info_ptr->status.direction        = I2C_DIR_TRANSMITTER;
    i2c_info_ptr->status.bus_error        = 0;

    i2c_info_ptr->tx_over = 0U;

    i2c_enable_master_or_slave_interrupt(i2c, I2C_ENABLE_TRANSMITTER_INT);

  return ARM_DRIVER_OK;
}
/**
 * @brief    i2c master receive data using interrupt method
 * @note     implemented only 7-bit slave addr.
 * @param    i2c                    : Pointer to i2c resources structure
 * @param    addr                   : 7-bit/10-bit slave address
 * @param    data                   : Pointer to input data which needs to be transmit
 * @param    num                    : total length of input data
 * @retval   ARM_DRIVER_ERROR_BUSY  : error previous transfer operation is in process
 * @retval   ARM_DRIVER_OK          : success
 */
int32_t i2c_enable_slave_receive_interrupt(i2c_resources_t  *i2c, uint8_t *data, uint32_t num)
{
    i2c_info_t      *i2c_info_ptr = i2c->info;
    i2c_reg_set_t   *i2c_reg_ptr  = (i2c_reg_set_t *)i2c->reg_base;

    if (i2c_info_ptr->status.busy)
    {
        /* Transfer operation in progress */
        return ARM_DRIVER_ERROR_BUSY;
    }

    /* fill the i2c transfer structure as per user detail */
    i2c_info_ptr->transfer.rx_buf        = (uint8_t *)data;
    i2c_info_ptr->transfer.rx_total_num  = num;
    i2c_info_ptr->transfer.rx_curr_cnt   = 0U;
    i2c_info_ptr->transfer.curr_cnt      = 0U;
    i2c_info_ptr->cur_state = I2C_IN_RX;

    /* Update driver status \ref ARM_I2C_STATUS */
    i2c_info_ptr->status.busy             = 1;
    i2c_info_ptr->status.mode             = I2C_SLAVE_MODE;
    i2c_info_ptr->status.direction        = I2C_DIR_RECEIVER;
    i2c_info_ptr->status.bus_error        = 0;

    i2c_info_ptr->rx_over = 0U;

    i2c_enable_master_or_slave_interrupt(i2c, I2C_ENABLE_RECEIVER_INT);

  return ARM_DRIVER_OK;
}

/**
 * @brief    i2c slave transmit data using interrupt method
 * @note     implemented only 7-bit slave addr.
 * @param    i2c    : Pointer to i2c resources structure
 * @retval   callback event
 */
static int32_t i2c_slave_tansmit_data_isr(i2c_resources_t *i2c)
{
    int32_t ret = ARM_DRIVER_OK;

    i2c_info_t      *i2c_info_ptr = i2c->info;
    i2c_reg_set_t   *i2c_reg_ptr  = (i2c_reg_set_t *)i2c->reg_base;
    uint32_t i2c_int_status;
    uint32_t xmit_data = 0;
    uint32_t xmit_end  = 0;
    uint32_t event     = 0;

    i2c_int_status =(i2c_reg_ptr->ic_intr_stat);

    /* Slave error state check */
    i2c_info_ptr->err_state = i2c_slave_check_error(i2c);

    /* Transfer buffer has data to transmit */
    if(i2c_info_ptr->transfer.tx_buf)
    {
        /* Slave is Active */
        if(i2c_reg_ptr->ic_status & I2C_IC_STATUS_SLAVE_ACT)
        {
            /* checking FIFO is full ready to transmit data */
            while(i2c_tx_ready(i2c_reg_ptr))
            {
                   xmit_data = (uint32_t)( i2c_info_ptr->transfer.tx_buf[i2c_info_ptr->transfer.tx_curr_cnt] ) | I2C_IC_DATA_CMD_WRITE_REQ;

                    /* remaining byte send as a last condition none.*/
                   if (i2c_info_ptr->transfer.tx_curr_cnt >= (i2c_info_ptr->transfer.tx_total_num-1))
                   {
                       xmit_end = 1;
                   }

                   i2c_info_ptr->transfer.tx_curr_cnt++;
                   i2c_info_ptr->transfer.curr_cnt = i2c_info_ptr->transfer.tx_curr_cnt;

                   i2c_write_data_cmd_next_condition_to_buffer(i2c_reg_ptr, xmit_data);

                   if (xmit_end)
                   {
                       /* transmitted all the bytes, disable the transmit interrupt */
                       i2c_disable_master_or_slave_interrupt(i2c, I2C_DISABLE_TRANSMITTER_INT);
                       i2c_info_ptr->cur_state = I2C_FREE;

                       /* clear busy status bit. */
                       i2c_info_ptr->status.busy = 0U;

                       /* mark event as slave transmit complete successfully. */
                       event |= ARM_I2C_EVENT_TRANSFER_DONE;

                       break;
                   }/* (xmit_end) */

            } /* while(i2c_tx_ready(i2c_reg_ptr)) END*/

        }/* (i2c_reg_ptr->ic_status & I2C_IC_STATUS_SLAVE_ACT) END*/

    }/* (i2c_info_ptr->transfer.tx_buf) END*/

    if (i2c_int_status & I2C_IC_INTR_STAT_TX_OVER)
    {
        i2c_info_ptr->tx_over ++;
    }

    /* Slave Transmit Abort/bus error */
    if( i2c_info_ptr->err_state == I2C_ERR_LOST_BUS)
    {
        i2c_info_ptr->status.bus_error = 1U;

        i2c_disable_master_or_slave_interrupt(i2c, I2C_DISABLE_TRANSMITTER_INT);
        /* clear busy status bit. */
        i2c_info_ptr->status.busy = 0U;

        /* mark event as slave lost bus */
        event |= ARM_I2C_EVENT_BUS_ERROR;
        event |= ARM_I2C_EVENT_TRANSFER_DONE;
        event |= ARM_I2C_EVENT_TRANSFER_INCOMPLETE;
    }

    /* Clear Interrupt */
    i2c_int_status = i2c_reg_ptr->ic_clr_intr;

    return event;
}

/**
 * @brief    i2c slave receive data using interrupt method
 * @note     implemented only 7-bit slave addr.
 * @param    i2c    : Pointer to i2c resources structure
 * @retval   callback event
 */
static int32_t i2c_slave_receive_data_isr(i2c_resources_t *i2c)
{
    int32_t ret = ARM_DRIVER_OK;
    uint32_t i2c_int_status;
    uint32_t last_cond = 0;
    uint32_t xmit_data = 0;
    uint32_t event     = 0;

    i2c_info_t      *i2c_info_ptr = i2c->info;
    i2c_reg_set_t   *i2c_reg_ptr  = (i2c_reg_set_t *)i2c->reg_base;

    i2c_int_status = (i2c_reg_ptr->ic_intr_stat);

    /* Checking for the RX full interrupt */
    if (i2c_int_status & I2C_IC_INTR_STAT_RX_FULL)
    {
        /* Ready to receive data, FIFO has data */
        while (i2c_rx_ready(i2c_reg_ptr))
        {
            /* rx ready, data is available into data buffer read it. */
            i2c_info_ptr->transfer.rx_buf[i2c_info_ptr->transfer.rx_curr_cnt] = i2c_read_data_from_buffer(i2c_reg_ptr);

            i2c_info_ptr->transfer.rx_curr_cnt++;
            i2c_info_ptr->transfer.curr_cnt = i2c_info_ptr->transfer.rx_curr_cnt;

            /* received all the bytes? */
            if (i2c_info_ptr->transfer.rx_curr_cnt >= i2c_info_ptr->transfer.rx_total_num)
            {
                /* received all the bytes disable the RX interrupt and update callback event. */
                i2c_disable_master_or_slave_interrupt(i2c, I2C_DISABLE_RECEIVER_INT);
                i2c_info_ptr->cur_state = I2C_FREE;

                /* clear busy status bit. */
                i2c_info_ptr->status.busy = 0U;

                /* mark event as slave receive complete successfully. */
                event |= ARM_I2C_EVENT_TRANSFER_DONE;

                break;
            }/* received all the bytes */

        }/* while (i2c_rx_ready(i2c_reg_ptr)) END*/

    }/* (i2c_int_status & I2C_IC_INTR_STAT_RX_FULL) END */

    if (i2c_int_status & (I2C_IC_INTR_STAT_RX_OVER|I2C_IC_INTR_STAT_RX_UNDER))
    {
        i2c_info_ptr->rx_over++;
    }

    /* Clear Interrupt */
    i2c_int_status = i2c_reg_ptr->ic_clr_intr;

    return event;
}

/**
 * @brief    i2c master transmit data using interrupt method
 * @note     implemented only 7-bit slave addr.
 * @param    i2c    : Pointer to i2c resources structure
 * @retval   callback event
 */
static uint32_t i2c_master_transmit_data_isr(i2c_resources_t *i2c)
{
    i2c_info_t      *i2c_info_ptr = i2c->info;
    i2c_reg_set_t   *i2c_reg_ptr  = (i2c_reg_set_t *)i2c->reg_base;

    uint32_t i2c_int_status; /* i2c interrupt status */
    uint32_t last_cond = 0;
    uint32_t xmit_data = 0;
    uint32_t xmit_end  = 0;
    uint32_t event     = 0;

    if (i2c_info_ptr->pending)
    {
        /* transfer is pending set last command to Restart */
        last_cond = I2C_IC_DATA_CMD_RESTART;
    }
    else
    {
        /* transfer is completed (not pending) set last command to Stop. */
        last_cond = I2C_IC_DATA_CMD_STOP;
    }

    i2c_int_status = (i2c_reg_ptr->ic_intr_stat);

    if (i2c_info_ptr->transfer.tx_buf)
    {
        if (i2c_int_status & I2C_IC_INTR_STAT_TX_EMPTY)
        {
            xmit_end = 0;

            while (i2c_tx_ready(i2c_reg_ptr))
            {
                xmit_data = (uint32_t)( i2c_info_ptr->transfer.tx_buf[i2c_info_ptr->transfer.tx_curr_cnt] ) | I2C_IC_DATA_CMD_WRITE_REQ;

                /* send last byte separately with Restart or Stop condition
                 * remaining byte send as a last condition none.
                 */
                if (i2c_info_ptr->transfer.tx_curr_cnt == (i2c_info_ptr->transfer.tx_total_num-1))
                {
                    xmit_end = 1;
                    xmit_data |= last_cond;
                }
                else
                {
                    xmit_data |= I2C_IC_DATA_CMD_NONE;
                }

                i2c_info_ptr->transfer.tx_curr_cnt++;
                i2c_info_ptr->transfer.curr_cnt = i2c_info_ptr->transfer.tx_curr_cnt;

                i2c_write_data_cmd_next_condition_to_buffer(i2c_reg_ptr, xmit_data);

                if (xmit_end)
                {
                    /* transmitted all the bytes, disable the transmit interrupt */
                    i2c_disable_master_or_slave_interrupt(i2c, I2C_DISABLE_TRANSMITTER_INT);
                    i2c_info_ptr->cur_state = I2C_FREE;

                    /* clear busy status bit. */
                    i2c_info_ptr->status.busy = 0U;

                    /* mark event as master transmit complete successfully. */
                    event |= ARM_I2C_EVENT_TRANSFER_DONE;

                    break;
                }
            }
        }
        if (i2c_int_status & I2C_IC_INTR_STAT_TX_OVER)
        {
            i2c_info_ptr->tx_over ++;
        }

        if (i2c_int_status & I2C_IC_INTR_STAT_TX_ABRT)
        {
            i2c_info_ptr->err_state = i2c_master_check_error (i2c);

            if( i2c_info_ptr->err_state == I2C_ERR_LOST_BUS)
            {
                i2c_info_ptr->status.arbitration_lost = 1U;

                /* mark event as master lost arbitration. */
                event |= ARM_I2C_EVENT_ARBITRATION_LOST;
                event |= ARM_I2C_EVENT_TRANSFER_DONE;
                event |= ARM_I2C_EVENT_TRANSFER_INCOMPLETE;
            }
            if( i2c_info_ptr->err_state == I2C_ERR_ADDR_NOACK)
            {
                 /* mark event as slave not acknowledge 7bit/10bit addr. */
                event |= ARM_I2C_EVENT_ADDRESS_NACK;
                event |= ARM_I2C_EVENT_TRANSFER_DONE;
                event |= ARM_I2C_EVENT_TRANSFER_INCOMPLETE;
            }
            if( i2c_info_ptr->err_state == I2C_ERR_DATA_NOACK)
            {
                /* mark event as slave not acknowledge for the data. */
                event |= ARM_I2C_EVENT_TRANSFER_DONE;
                event |= ARM_I2C_EVENT_TRANSFER_INCOMPLETE;
            }
            if( i2c_info_ptr->err_state == I2C_ERR_UNDEF)
            {
                i2c_info_ptr->status.bus_error = 1U;

                event |= ARM_I2C_EVENT_BUS_ERROR;
                event |= ARM_I2C_EVENT_TRANSFER_DONE;
                event |= ARM_I2C_EVENT_TRANSFER_INCOMPLETE;
            }

            if (i2c_info_ptr->err_state != I2C_ERR_NONE)
            {
                i2c_disable_master_or_slave_interrupt(i2c, I2C_DISABLE_TRANSMITTER_INT);
                i2c_info_ptr->cur_state = I2C_FREE;

                /* clear busy status bit. */
                i2c_info_ptr->status.busy = 0U;
                event |= ARM_I2C_EVENT_TRANSFER_DONE;
                event |= ARM_I2C_EVENT_TRANSFER_INCOMPLETE;
            }
            }

    }
    else
    {
        i2c_disable_master_or_slave_interrupt(i2c, I2C_DISABLE_TRANSMITTER_INT);
        i2c_info_ptr->cur_state = I2C_FREE;
        /* clear busy status bit. */
        i2c_info_ptr->status.busy = 0U;
        event |= ARM_I2C_EVENT_TRANSFER_DONE;
        event |= ARM_I2C_EVENT_TRANSFER_INCOMPLETE;
    }
    /* Clear Interrupt */
    i2c_int_status = i2c_reg_ptr->ic_clr_intr;

    /* return the callback event. */
    return event;
}

/**
 * @brief    i2c master receive data using interrupt method
 * @note     implemented only 7-bit slave addr.
 * @param    i2c    : Pointer to i2c resources structure
 * @retval   callback event
 */
static uint32_t i2c_master_receive_data_isr(i2c_resources_t *i2c)
{
    i2c_info_t      *i2c_info_ptr = i2c->info;
    i2c_reg_set_t   *i2c_reg_ptr  = (i2c_reg_set_t *)i2c->reg_base;

    uint32_t i2c_int_status;
    uint32_t last_cond = 0;
    uint32_t xmit_data = 0;
    uint32_t event     = 0;

    if (i2c_info_ptr->pending)
    {
        /* transfer is pending set last command to Restart */
        last_cond = I2C_IC_DATA_CMD_RESTART;
    }
    else
    {
        /* transfer is completed (not pending) set last command to Stop. */
        last_cond = I2C_IC_DATA_CMD_STOP;
    }

    i2c_int_status = (i2c_reg_ptr->ic_intr_stat);

    if (i2c_info_ptr->transfer.rx_buf)
    {
        if (i2c_int_status & I2C_IC_INTR_STAT_TX_EMPTY)
        {
            while (i2c_tx_ready(i2c_reg_ptr))
            {

                /* completed sending all the read commands? */
                if (i2c_info_ptr->transfer.rx_curr_tx_index >= i2c_info_ptr->transfer.rx_total_num)
                {
                    /* added all the read commands to FIFO.
                     * now we have to read from i2c so disable TX interrupt. */
                    i2c_mask_interrupt(i2c_reg_ptr, I2C_IC_INTR_STAT_TX_EMPTY);
                    break;
                }
                xmit_data = I2C_IC_DATA_CMD_READ_REQ;

                /* send last byte separately with Restart or Stop condition
                 * remaining byte send as a last condition none.
                 */
                if (i2c_info_ptr->transfer.rx_curr_tx_index == (i2c_info_ptr->transfer.rx_total_num - 1) )
                {
                    /* last byte send with Restart/ Stop condition. */
                    xmit_data |= last_cond;
                }
                else
                {
                    xmit_data |= I2C_IC_DATA_CMD_NONE;
                }
                i2c_info_ptr->transfer.rx_curr_tx_index++;

                /* update read condition in data-buffer. */
                i2c_write_data_cmd_next_condition_to_buffer(i2c_reg_ptr, xmit_data);
            }
        }

        /* transmitted all the read condition, waiting for i2c to receive data from slave.
         * IC_INTR_STAT_RX_FULL set when i2c receives reaches or goes above RX_TL threshold (0 in our case) */
        if (i2c_int_status & I2C_IC_INTR_STAT_RX_FULL)
        {
            while (i2c_rx_ready(i2c_reg_ptr))
            {
                /* rx ready, data is available into data buffer read it. */
                i2c_info_ptr->transfer.rx_buf[i2c_info_ptr->transfer.rx_curr_cnt] = i2c_read_data_from_buffer(i2c_reg_ptr);

                i2c_info_ptr->transfer.rx_curr_cnt++;
                i2c_info_ptr->transfer.curr_cnt = i2c_info_ptr->transfer.rx_curr_cnt;

                /* received all the bytes */
                if (i2c_info_ptr->transfer.rx_curr_cnt >= i2c_info_ptr->transfer.rx_total_num)
                {
                    /* received all the bytes disable the RX interrupt and update callback event. */
                    i2c_disable_master_or_slave_interrupt(i2c, I2C_DISABLE_RECEIVER_INT);
                    i2c_info_ptr->cur_state = I2C_FREE;

                    /* clear busy status bit. */
                    i2c_info_ptr->status.busy = 0U;

                    /* mark event as master receive complete successfully. */
                    event |= ARM_I2C_EVENT_TRANSFER_DONE;

                    break;
                }
            }
        }

        if (i2c_int_status & (I2C_IC_INTR_STAT_RX_OVER|I2C_IC_INTR_STAT_RX_UNDER))
        {
            i2c_info_ptr->rx_over++;
        }

        /* we got Transfer abort check the error condition */
        if (i2c_int_status & I2C_IC_INTR_STAT_TX_ABRT)
        {

            i2c_info_ptr->err_state = i2c_master_check_error (i2c);

            if( i2c_info_ptr->err_state == I2C_ERR_LOST_BUS)
            {
                i2c_info_ptr->status.arbitration_lost = 1U;

                /* mark event as master lost arbitration. */
                event |= ARM_I2C_EVENT_ARBITRATION_LOST;
                event |= ARM_I2C_EVENT_TRANSFER_DONE;
                event |= ARM_I2C_EVENT_TRANSFER_INCOMPLETE;
            }
            if( i2c_info_ptr->err_state == I2C_ERR_ADDR_NOACK)
            {
                /* mark event as slave not acknowledge 7bit/10bit addr. */
                event |= ARM_I2C_EVENT_ADDRESS_NACK;
                event |= ARM_I2C_EVENT_TRANSFER_DONE;
                event |= ARM_I2C_EVENT_TRANSFER_INCOMPLETE;
            }
            if( i2c_info_ptr->err_state == I2C_ERR_DATA_NOACK)
            {
                /* mark event as slave not acknowledge for the data. */
                event |= ARM_I2C_EVENT_TRANSFER_DONE;
                event |= ARM_I2C_EVENT_TRANSFER_INCOMPLETE;
            }
            if( i2c_info_ptr->err_state == I2C_ERR_UNDEF)
            {
                /* any other error */
                i2c_info_ptr->status.bus_error = 1U;

                event |= ARM_I2C_EVENT_BUS_ERROR;
                event |= ARM_I2C_EVENT_TRANSFER_DONE;
                event |= ARM_I2C_EVENT_TRANSFER_INCOMPLETE;
            }

            if (i2c_info_ptr->err_state != I2C_ERR_NONE)
            {
                i2c_disable_master_or_slave_interrupt(i2c, I2C_DISABLE_RECEIVER_INT);
                i2c_info_ptr->cur_state = I2C_FREE;

                /* clear busy status bit. */
                i2c_info_ptr->status.busy = 0U;
                event |= ARM_I2C_EVENT_TRANSFER_DONE;
                event |= ARM_I2C_EVENT_TRANSFER_INCOMPLETE;
            }

        }
    }
    else
    {
        i2c_disable_master_or_slave_interrupt(i2c, I2C_DISABLE_RECEIVER_INT);
        i2c_info_ptr->cur_state = I2C_FREE;
        /* clear busy status bit. */
        i2c_info_ptr->status.busy = 0U;
        event |= ARM_I2C_EVENT_TRANSFER_DONE;
        event |= ARM_I2C_EVENT_TRANSFER_INCOMPLETE;
    }
    /* Clear Interrupt */

    i2c_int_status = i2c_reg_ptr->ic_clr_intr;

    /* return the callback event. */
    return event;
}

/**
 * @brief    i2c interrupt handler
 * @note     only one combined interrupt for
 *          -TX / RX
 *          -to abort the transfer call abort_transmit /abort_receive
 * @param    i2c    : Pointer to i2c resources structure
 * @retval   none
 */
void i2c_irq_handler (i2c_resources_t *i2c)
{
    /* callback event */
    uint32_t event = 0U;

    i2c_info_t      *i2c_info_ptr = i2c->info;
    i2c_reg_set_t   *i2c_reg_ptr  = (i2c_reg_set_t *)i2c->reg_base;

    /* Check for master mode */
    if (i2c_info_ptr->mode == I2C_MASTER_MODE)
    {
        if (i2c_info_ptr->cur_state == I2C_IN_TX)
        {
            event = i2c_master_transmit_data_isr(i2c);
        }
        else
        {
            event = i2c_master_receive_data_isr(i2c);
        }
    }/* master END */
    else /* Slave mode */
    {
        if(i2c_reg_ptr->ic_raw_intr_stat & I2C_IC_INTR_STAT_RD_REQ)
        {
            /* slave transmit*/
            event = i2c_slave_tansmit_data_isr(i2c);
        }
        else if(i2c_reg_ptr->ic_raw_intr_stat & I2C_IC_INTR_STAT_RX_FULL)
        {
           /* slave receive */
           event = i2c_slave_receive_data_isr(i2c);
        }
    }/* Slave mode END*/

    /* call the user callback if any event occurs */
    if ((event != 0U) && (i2c_info_ptr->cb_event != NULL) )
    {
        /* call the user callback */
        i2c_info_ptr->cb_event(event);
    }

}

/**
 * @brief    initialize i2c
 * @note     this function will
 *           -set tx/rx fifo length,
 *           -set addr mode to 7-bit addr
 *           -disable the interrupt,
 *           -initialize all the variable to 0
 * @note     implemented only 7-bit Slave Addr
 * @param    i2c    : Pointer to i2c resources structure
 * @retval   none
 */
void i2c_initialize (i2c_resources_t *i2c)
{
    i2c_info_t     *i2c_info_ptr = i2c->info;
    i2c_reg_set_t  *i2c_reg_ptr  = (i2c_reg_set_t *)i2c->reg_base;

    /* default setting */
    i2c_info_ptr->addr_mode = I2C_7BIT_ADDRESS;
    i2c_info_ptr->tar_addr  = I2C_0_TARADDR;
    i2c_info_ptr->retry_cnt = I2C_MAX_RETRY_COUNT;

    /* now 7bit addr only */
    if (i2c_info_ptr->addr_mode == I2C_7BIT_ADDRESS)
    {
        i2c_info_ptr->tar_addr &= I2C_7BIT_ADDRESS_MASK;
        i2c_info_ptr->slv_addr &= I2C_7BIT_ADDRESS_MASK;
    }

    /* i2c_ip config IC_TX_BUFFER_DEPTH */
    i2c_info_ptr->tx_fifo_length = 64;
    i2c_info_ptr->rx_fifo_length = 64;

    i2c_calc_spike(i2c->clk/1000, &(i2c_info_ptr->i2c_spklen));

    //ic_caploading only for high speed mode
    i2c_calc_sclcnt(i2c->clk/1000, I2C_CAP_LOADING_100PF, \
         &(i2c_info_ptr->i2c_scl_cnt), &(i2c_info_ptr->i2c_spklen));

    /* Disable device before init it */
    i2c_disable_device(i2c);

    i2c_info_ptr->err_state = I2C_ERR_NONE;
    i2c_info_ptr->next_cond = I2C_MODE_STOP;

    i2c_info_ptr->tx_over = 0;
    i2c_info_ptr->rx_over = 0;

    /* clear interrupt status */
    i2c_info_ptr->int_status = 0;

    /* install i2c interrupt into system */
    i2c_disable_interrupt(i2c);
}

/**
 * @brief    uninitialize i2c
 * @note     this function will
 *           -disable interrupt,
 *           -abort TX/RX,
 *           -initialize all the variable to 0
 * @note     needs to initialize first if wants to use it again.
 * @param    i2c    : Pointer to i2c resources structure
 * @retval   none
 */
void i2c_uninitialize (i2c_resources_t *i2c)
{
    i2c_info_t      *i2c_info_ptr = i2c->info;
    i2c_reg_set_t   *i2c_reg_ptr  = (i2c_reg_set_t *)i2c->reg_base;

    i2c_disable_interrupt(i2c);

    i2c_abort_transmit(i2c);
    i2c_abort_receive(i2c);

    i2c_disable_device(i2c);

    /* initialize all variables to 0 */

    /* initialize the tx_buffer */
    i2c_info_ptr->transfer.tx_buf               = NULL;
    i2c_info_ptr->transfer.tx_total_num         = 0U;
    i2c_info_ptr->transfer.tx_curr_cnt          = 0U;

    i2c_info_ptr->transfer.curr_cnt             = 0U;

    /* initialize the rx_buffer */
    i2c_info_ptr->transfer.rx_buf               = NULL;
    i2c_info_ptr->transfer.rx_total_num         = 0U;
    i2c_info_ptr->transfer.rx_curr_cnt          = 0U;
    i2c_info_ptr->transfer.rx_curr_tx_index     = 0U;

    /* Clear driver status \ref ARM_I2C_STATUS */
    i2c_info_ptr->status.busy                   = 0U;
    i2c_info_ptr->status.mode                   = 0U;
    i2c_info_ptr->status.direction              = 0U;
    i2c_info_ptr->status.arbitration_lost       = 0U;
    i2c_info_ptr->status.bus_error              = 0U;

    i2c_info_ptr->addr_mode                     = I2C_7BIT_ADDRESS;
    i2c_info_ptr->tar_addr                      = 0U;
    i2c_info_ptr->retry_cnt                     = 0U;
    i2c_info_ptr->tx_fifo_length                = 0U;
    i2c_info_ptr->rx_fifo_length                = 0U;
    i2c_info_ptr->err_state                     = I2C_ERR_NONE;
    i2c_info_ptr->next_cond                     = I2C_MODE_STOP;
    i2c_info_ptr->tx_over                       = 0U;
    i2c_info_ptr->rx_over                       = 0U;
    i2c_info_ptr->pending                       = 0U;
    i2c_info_ptr->cur_state                     = I2C_FREE;
    i2c_info_ptr->int_status                    = 0U;
}

/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
