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
 * @file     i2c_ll_drv.h
 * @author   Tanay Rami         | Prabhakar Kumar
 * @email    tanay@alifsemi.com | prabhakar.kumar@alifsemi.com
 * @version  V1.0.0
 * @date     20-June-2020       | 21-July-2022
 * @brief    Header file for Low-level Driver for I2C.
 *           This file is the unique include file that the application
 *           programmer is using in the C source code, usually in main.c.
 *           This file contains:
 *            - Global data types.
 *            - typedefs & enums.
 *            - define & macro.
 *            - Structures and Unions
 *            - Function prototypes.
 ******************************************************************************/

#ifndef I2C_LL_DRV_H_	/* include guard */
#define I2C_LL_DRV_H_

#ifdef  __cplusplus
extern "C"
{
#endif

/**** system includes ****/
#include "Driver_I2C.h"
#include "RTE_Device.h"
#include "RTE_Components.h"
#include CMSIS_device_header

#include <stdint.h>

/**
 * Invalid Interrupt Vector Number When interrupt number is set to this value,
 * all interrupt related function in the device driver source code shouldn't be called,
 */
#define I2C_INVALID_INT_NUM                         (0xFFFFFFFF) /* invalid interrupt number */
#define I2C_INVALID_PRIORITY                        (0xFFFFFFFF) /* invalid interrupt priority  */

/* defines for i2c flag */
#define I2C_FLAG_INITIALIZED                        (1U << 0)    /* i2c initialized */
#define I2C_FLAG_POWERED                            (1U << 1)    /* i2c powered     */
#define I2C_FLAG_MASTER_SETUP                       (1U << 2)    /* i2c master setup speed mode */
#define I2C_FLAG_SLAVE_SETUP                        (1U << 3)    /* i2c Slave setup speed mode */

/* interrupt related flags */
#define I2C_FLAG_TX_OR_RX_INT_ENABLE                (1U << 0)    /* to check tx or rx anyone interrupt is enable */
#define I2C_FLAG_TX_INT_ENABLE                      (1U << 1)    /* tx interrupt enable */
#define I2C_FLAG_RX_INT_ENABLE                      (1U << 2)    /* rx interrupt enable */

/*
 * macros for device working mode \ref ARM_I2C_STATUS
 */
#define I2C_SLAVE_MODE                              (0)          /* Indicate that the device working as slave */
#define I2C_MASTER_MODE                             (1)          /* Indicate that the device working as master */

#define I2C_DIR_TRANSMITTER                         (0)          /* direction transmitter */
#define I2C_DIR_RECEIVER                            (1)          /* direction receiver  */

#define I2C_SLAVE_7BIT_ADDR_MODE                    (0)          /* 7 bit address mode for slave mode */

/* 7bit I2C address mask */
#define I2C_7BIT_ADDRESS_MASK                       (0x7F)
/* 10bit I2C address mask */
#define I2C_10BIT_ADDRESS_MASK                      (0x3FF)

#define I2C_0_TARADDR                               (0x50)

/* maximum retry count */
#define I2C_MAX_RETRY_COUNT                         ((100000)*2)

/* Timeout count, approximate to be 25us
 * in 50MHz CPU @ Standard mode */
#define I2C_DISABLE_MAX_TIME_OUT_CNT                (1250)

/* registers macros --------------------------------------------------------------------------------------------------- */

/* Enable I2C */
#define I2C_IC_ENABLE_I2C_ENABLE                    (1)

/* Disable I2C */
#define I2C_IC_ENABLE_I2C_DISABLE                   (0)

/* i2c IC_ENABLE_STATUS Bits */
#define I2C_IC_ENABLE_STATUS_IC_EN                  (1 << 0)

/* Stop Condition issue after this byte */
#define I2C_IC_DATA_CMD_STOP                        (1 << 9)
/* Restart Condition issue after this byte */
#define I2C_IC_DATA_CMD_RESTART                     (1 << 10)
/* No Restart or stop condition after this byte */
#define I2C_IC_DATA_CMD_NONE                        (0)

/* i2c Status Register Fields. */
#define I2C_IC_STATUS_ACTIVITY                      (0x01)      /* (1 << 0) */
#define I2C_IC_STATUS_TRANSMIT_FIFO_NOT_FULL        (0x02)      /* (1 << 1) */
#define I2C_IC_STATUS_TFE                           (0x04)      /* (1 << 2) */
#define I2C_IC_STATUS_RECEIVE_FIFO_NOT_EMPTY        (0x08)      /* (1 << 3) */
#define I2C_IC_STATUS_RFF                           (0x10)      /* (1 << 4) */
#define I2C_IC_STATUS_MASTER_ACT                    (0x20)      /* (1 << 5) */
#define I2C_IC_STATUS_SLAVE_ACT                     (0x40)      /* (1 << 6) */

/* Perform a write request */
#define I2C_IC_DATA_CMD_WRITE_REQ                   (0)
/* Perform a read request */
#define I2C_IC_DATA_CMD_READ_REQ                    (1 << 8)

/* Speed modes of IC_CON */
#define I2C_IC_CON_SPEED_MASK                       (0x6)
#define I2C_IC_CON_SPEED_STANDARD                   (0x2)
#define I2C_IC_CON_SPEED_FAST                       (0x4)
#define I2C_IC_CON_SPEED_HIGH                       (0x6)

/* Working mode of IC_CON */
#define I2C_IC_CON_MST_SLV_MODE_MASK                (0x41)
#define I2C_IC_CON_ENABLE_MASTER_MODE               (0x41)
#define I2C_IC_CON_ENA_SLAVE_MODE                   (0)

/* I2C interrupt control */
#define I2C_IC_INT_DISABLE_ALL                      (0x0)

/* Interrupt Register Fields */
#define I2C_IC_INTR_STAT_GEN_CALL                   (1 << 11)
#define I2C_IC_INTR_STAT_START_DET                  (1 << 10)
#define I2C_IC_INTR_STAT_STOP_DET                   (1 << 9)
#define I2C_IC_INTR_STAT_ACTIVITY                   (1 << 8)
#define I2C_IC_INTR_STAT_RX_DONE                    (1 << 7)

#define I2C_IC_INTR_STAT_TX_ABRT                    (1 << 6)    /* raw interrupt status */
#define I2C_IC_INTR_STAT_RD_REQ                     (1 << 5)
#define I2C_IC_INTR_STAT_TX_EMPTY                   (1 << 4)
#define I2C_IC_INTR_STAT_TX_OVER                    (1 << 3)    /* raw interrupt status */

#define I2C_IC_INTR_STAT_RX_FULL                    (1 << 2)
#define I2C_IC_INTR_STAT_RX_OVER                    (1 << 1)    /* raw interrupt status */
#define I2C_IC_INTR_STAT_RX_UNDER                   (1 << 0)    /* raw interrupt status */

/* Interrupt enable mask as master */
#define I2C_IC_INT_MST_TX_ENABLE                    (I2C_IC_INTR_STAT_TX_EMPTY|I2C_IC_INTR_STAT_TX_OVER|I2C_IC_INTR_STAT_TX_ABRT)
#define I2C_IC_INT_MST_RX_ENABLE                    (I2C_IC_INTR_STAT_TX_EMPTY|I2C_IC_INTR_STAT_RX_FULL|I2C_IC_INTR_STAT_RX_OVER|I2C_IC_INTR_STAT_RX_UNDER|I2C_IC_INTR_STAT_TX_ABRT)
#define I2C_IC_INT_SLV_TX_ENABLE                    (I2C_IC_INTR_STAT_RD_REQ|I2C_IC_INTR_STAT_TX_ABRT)
#define I2C_IC_INT_SLV_RX_ENABLE                    (I2C_IC_INTR_STAT_RX_FULL|I2C_IC_INTR_STAT_RX_OVER|I2C_IC_INTR_STAT_RX_UNDER)

/* IC_TX_ABRT_SOURCE Register Bit Fields */
#define I2C_IC_TX_ABRT_7B_ADDR_NOACK                (1 << 0)
#define I2C_IC_TX_ABRT_10ADDR1_NOACK                (1 << 1)
#define I2C_IC_TX_ABRT_10ADDR2_NOACK                (1 << 2)

#define I2C_IC_TX_ABRT_TXDATA_NOACK                 (1 << 3)

#define I2C_IC_TX_ABRT_GCALL_NOACK                  (1 << 4)
#define I2C_IC_TX_ABRT_GCALL_READ                   (1 << 5)
#define I2C_IC_TX_ABRT_HS_ACKDET                    (1 << 6)
#define I2C_IC_TX_ABRT_SBYTE_ACKDET                 (1 << 7)
#define I2C_IC_TX_ABRT_HS_NORSTRT                   (1 << 8)
#define I2C_IC_TX_ABRT_SBYTE_NORSTRT                (1 << 9)
#define I2C_IC_TX_ABRT_10B_RD_NORSTRT               (1 << 10)
#define I2C_IC_TX_ABRT_MASTER_DIS                   (1 << 11)

#define I2C_IC_TX_ABRT_ARB_LOST                     (1 << 12)
#define I2C_IC_TX_ABRT_SLVFLUSH_TXFIFO              (1 << 13)
#define I2C_IC_TX_ABRT_SLV_ARBLOST                  (1 << 14)
#define I2C_IC_TX_ABRT_SLVRD_INTX                   (1 << 15)

/* I2C TX & RX threshold settings */
#define I2C_IC_TX_TL_TX_FIFO_THRESHOLD_LVL          (0)
#define I2C_IC_RX_TL_RX_FIFO_THRESHOLD_LVL          (0)

/* Combined bits for i2c abort source as master */
#define I2C_MST_ABRT_ADDR_NOACK                     (I2C_IC_TX_ABRT_7B_ADDR_NOACK|I2C_IC_TX_ABRT_10ADDR1_NOACK|I2C_IC_TX_ABRT_10ADDR2_NOACK)
#define I2C_MST_ABRT_LOST_BUS                       (I2C_IC_TX_ABRT_ARB_LOST)
#define I2C_MST_ABRT_DATA_NOACK                     (I2C_IC_TX_ABRT_TXDATA_NOACK)

/* Combined bits for i2c abort source as slave */
#define I2C_SLV_ABRT_LOST_BUS                       (I2C_IC_TX_ABRT_ARB_LOST|I2C_IC_TX_ABRT_SLV_ARBLOST)

/* register configuration ------------------------------------------------------------------------------------------------------------- */
#ifndef I2C_ALLOW_RESTART
#define I2C_ALLOW_RESTART                           (1)    /* allow restart configuration */
#endif

#ifndef I2C_DYNAMIC_TAR_UPDATE_SUPPORT
#define I2C_DYNAMIC_TAR_UPDATE_SUPPORT              (0)    /* Dynamic target address update support */
#endif

/* Fields of IC_CON register */
/*  I2C IP Config Dependencies. */
#if I2C_ALLOW_RESTART
#define I2C_IC_CON_MASTER_RESTART_EN                (1 << 5)
#else
#define I2C_IC_CON_MASTER_RESTART_EN                (0x00)
#endif

#if I2C_SPECIAL_START_BYTE
#define I2C_IC_TAR_SPECIAL                          (1 << 11)
#define I2C_IC_TAR_GC_OR_START                      (1 << 10)
#else
#define I2C_IC_TAR_SPECIAL                          (0x00)
#define I2C_IC_TAR_GC_OR_START                      (0x00)
#endif

/* register configuration ---------------------------------------------------------------------------------------- */
#define I2C_IC_TAR_7BIT_ADDR_MASK                   (0x7F)    /* 7bit  I2C address mask for target address register  */
#define I2C_IC_SAR_7BIT_ADDR_MASK                   (0x7F)    /* 7bit  I2C address mask for slave  address register  */
#define I2C_IC_TAR_10BIT_ADDR_MASK                  (0x3FF)   /* 10bit I2C address mask for target address register  */
#define I2C_IC_SAR_10BIT_ADDR_MASK                  (0x3FF)   /* 10bit I2C address mask for slave  address register  */

/* i2c register set */
typedef struct i2c_reg_set
{
    volatile uint32_t ic_con;                   /* (0x00) : I2C control                              */
    volatile uint32_t ic_tar;                   /* (0x04) : I2C target address                       */
    volatile uint32_t ic_sar;                   /* (0x08) : I2C slave address                        */
    volatile uint32_t ic_hs_maddr;              /* (0x0c) : I2C HS Master Mode Code address          */
    volatile uint32_t ic_data_cmd;              /* (0x10) : I2C Rx/Tx Data Buffer and Command        */

    volatile uint32_t ic_ss_scl_hcnt;           /* (0x14) : Standard Speed I2C clock SCL High Count  */
    volatile uint32_t ic_ss_scl_lcnt;           /* (0x18) : Standard Speed I2C clock SCL Low Count   */
    volatile uint32_t ic_fs_scl_hcnt;           /* (0x1c) : Fast Speed I2C clock SCL Low Count       */
    volatile uint32_t ic_fs_scl_lcnt;           /* (0x20) : Fast Speed I2C clock SCL Low Count       */
    volatile uint32_t ic_hs_scl_hcnt;           /* (0x24) : High Speed I2C clock SCL Low Count       */
    volatile uint32_t ic_hs_scl_lcnt;           /* (0x28) : High Speed I2C clock SCL Low Count       */

    volatile uint32_t ic_intr_stat;             /* (0x2c) : I2C Interrupt Status                     */
    volatile uint32_t ic_intr_mask;             /* (0x30) : I2C Interrupt Mask                       */
    volatile uint32_t ic_raw_intr_stat;         /* (0x34) : I2C Raw Interrupt Status                 */

    volatile uint32_t ic_rx_tl;                 /* (0x38) : I2C Receive FIFO Threshold               */
    volatile uint32_t ic_tx_tl;                 /* (0x3c) : I2C Transmit FIFO Threshold              */

    volatile uint32_t ic_clr_intr;              /* (0x40) : Clear combined and Individual Interrupts */
    volatile uint32_t ic_clr_rx_under;          /* (0x44) : Clear RX_UNDER Interrupt                 */
    volatile uint32_t ic_clr_rx_over;           /* (0x48) : Clear RX_OVER Interrupt                  */
    volatile uint32_t ic_clr_tx_over;           /* (0x4c) : Clear TX_OVER Interrupt                  */
    volatile uint32_t ic_clr_rd_req;            /* (0x50) : Clear RQ_REQ Interrupt                   */
    volatile uint32_t ic_clr_tx_abrt;           /* (0x54) : Clear TX_ABRT Interrupt                  */
    volatile uint32_t ic_clr_rx_done;           /* (0x58) : Clear RX_DONE Interrupt                  */
    volatile uint32_t ic_clr_activity;          /* (0x5c) : Clear ACTIVITY Interrupt                 */
    volatile uint32_t ic_clr_stop_det;          /* (0x60) : Clear STOP_DET Interrupt                 */
    volatile uint32_t ic_clr_start_det;         /* (0x64) : Clear START_DET Interrupt                */
    volatile uint32_t ic_clr_gen_call;          /* (0x68) : Clear GEN_CALL Interrupt                 */

    volatile uint32_t ic_enable;                /* (0x6c) : I2C Enable                               */
    volatile uint32_t ic_status;                /* (0x70) : I2C Status                               */
    volatile uint32_t ic_txflr;                 /* (0x74) : Transmit FIFO Level Register             */
    volatile uint32_t ic_rxflr;                 /* (0x78) : Receive FIFO Level Register              */
    volatile uint32_t ic_sda_hold;              /* (0x7c) : SDA Hold Time Length Reg                 */
    volatile uint32_t ic_tx_abrt_source;        /* (0x80) : I2C Transmit Abort Status Reg            */
    volatile uint32_t ic_slv_data_nack_only;    /* (0x84) : Generate SLV_DATA_NACK Register          */

    volatile uint32_t ic_dma_cr;                /* (0x88) : DMA Control Register                     */
    volatile uint32_t ic_dma_tdlr;              /* (0x8c) : DMA Transmit Data Level                  */
    volatile uint32_t ic_dma_rdlr;              /* (0x90) : DMA Receive Data Level                   */

    volatile uint32_t ic_sda_setup;             /* (0x94) : SDA Setup Register                       */
    volatile uint32_t ic_ack_general_call;      /* (0x98) : ACK General Call Register                */
    volatile uint32_t ic_enable_status;         /* (0x9c) : Enable Status Register                   */

    volatile uint32_t ic_fs_spklen;             /* (0xa0) : ISS and FS spike suppression limit       */
    volatile uint32_t ic_hs_spklen;             /* (0xa4) : HS spike suppression limit               */
    volatile uint32_t reserved[19];             /* (0xa8) : Reserved                                 */

    volatile uint32_t ic_comp_param_1;          /* (0xf4) : Component Parameter Register             */
    volatile uint32_t ic_comp_version;          /* (0xf8) : Component Version ID Reg                 */
    volatile uint32_t ic_comp_type;             /* (0xfc) : Component Type Reg                       */

} i2c_reg_set_t;

/* I2C Bus possible speed modes */
typedef enum i2c_speed_mode
{
    I2C_SPEED_STANDARD = 0,     /* Bidirectional, Standard-mode (Sm), with a bit rate up to 100 kbit/s               */
    I2C_SPEED_FAST     = 1,     /* Bidirectional, Fast-mode (Fm), with a bit rate up to 400 kbit/s                   */
    I2C_SPEED_FASTPLUS = 2,     /* Bidirectional, Fast-mode Plus (Fm+), with a bit rate up to 1 Mbit/s               */
    I2C_SPEED_HIGH     = 3,     /* Bidirectional, High-speed mode (Hs-mode), with a bit rate up to 3.4 Mbit/s        */
    I2C_SPEED_ULTRA    = 4      /* Unidirectional(Write only), Ultra Fast-mode (UFm), with a bit rate up to 5 Mbit/s */
} i2c_speed_mode_t;

/* I2C Error State */
typedef enum i2c_error_state
{
    I2C_ERR_NONE       = 0,     /* Currently in I2C device free state                            */
    I2C_ERR_LOST_BUS   = 1,     /* Master or slave lost bus during operation                     */
    I2C_ERR_ADDR_NOACK = 2,     /* Slave address is sent but not addressed by any slave devices  */
    I2C_ERR_DATA_NOACK = 3,     /* Data in transfer is not acked when it should be acked         */
    I2C_ERR_TIMEOUT    = 4,     /* Transfer timeout, no more data is received or sent            */
    I2C_ERR_MSTSTOP    = 5,     /* Slave received a STOP condition from master device            */
    I2C_ERR_UNDEF      = 6      /* Undefined error cases */
} i2c_error_state_t;

/* I2C next Condition */
typedef enum i2c_next_condtion
{
    I2C_MODE_STOP       = 0,    /* Send a STOP condition after write/read operation     */
    I2C_MODE_RESTART    = 1     /* Send a RESTART condition after write/read operation  */
} i2c_next_condtion_t;

/* I2C Working State */
typedef enum i2c_working_state
{
    I2C_FREE            = 0,    /* Currently in I2C device free state     */
    I2C_IN_TX           = 1,    /* Currently in I2C master transmit state */
    I2C_IN_RX           = 2     /* Currently in I2C master receive state  */
} i2c_working_state_t;

/* I2C Addressing Mode */
typedef enum i2c_address_mode
{
    I2C_7BIT_ADDRESS    = 0,    /* Use 7bit address mode  */
    I2C_10BIT_ADDRESS   = 1     /* Use 10bit address mode */
} i2c_address_mode_t;


#define I2C_FS_SPIKE_LENGTH_NS                (50)
#define I2C_HS_SPIKE_LENGTH_NS                (10)

#define I2C_MIN_SS_SCL_LCNT(spklen)     ((spklen)+7)
#define I2C_MIN_FS_SCL_LCNT(spklen)     ((spklen)+7)

#define I2C_MIN_SS_SCL_HCNT(spklen)     ((spklen)+5)
#define I2C_MIN_FS_SCL_HCNT(spklen)     ((spklen)+5)

#define I2C_MIN_HS_SCL_LCNT(spklen)     ((spklen)+7)
#define I2C_MIN_HS_SCL_HCNT(spklen)     ((spklen)+5)

#define I2C_MIN_SS_HIGH_TIME_NS         (4000)
#define I2C_MIN_SS_LOW_TIME_NS          (4700)

#define I2C_MIN_FS_HIGH_TIME_NS         (600)
#define I2C_MIN_FS_LOW_TIME_NS          (1300)

#define I2C_MIN_HS_100PF_HIGH_TIME_NS   (60)
#define I2C_MIN_HS_100PF_LOW_TIME_NS    (160)

#define I2C_MIN_HS_400PF_HIGH_TIME_NS   (120)
#define I2C_MIN_HS_400PF_LOW_TIME_NS    (320)

enum
{
    I2C_CAP_LOADING_100PF = 0,
    I2C_CAP_LOADING_400PF
};

/* Spike Suppression Limit Configurations */
typedef struct i2c_spike_length
{
    uint32_t fs_spike_length;                     /*  fast mode is 50ns        */
    uint32_t hs_spike_length;                     /*  high-speed mode is 10ns  */
} i2c_spike_length_t;

/* I2C Clock SCL High and Low Count Configurations for Different Speed */
typedef struct i2c_scl_cnt
{
    uint32_t standard_speed_scl_hcnt;       /* value for ic_ss_scl_hcnt */
    uint32_t standard_speed_scl_lcnt;       /* value for ic_ss_scl_lcnt */
    uint32_t fast_speed_scl_hcnt;           /* value for ic_fs_scl_hcnt */
    uint32_t fast_speed_scl_lcnt;           /* value for ic_fs_scl_lcnt */
    uint32_t high_speed_scl_hcnt;           /* value for ic_hs_scl_hcnt */
    uint32_t high_speed_scl_lcnt;           /* value for ic_hic_ss_scl_hcnts_scl_hcnt */
} i2c_scl_cnt_t;


/* i2c Transfer Information (Run-Time) */
typedef struct i2c_transfer_info
{
  uint8_t                *tx_buf;           /* Pointer to out data buffer                              */
  uint32_t                tx_total_num;     /* Total number of data to be send                         */
  uint32_t                tx_curr_cnt;      /* current Number of data sent from total num              */
  uint8_t                *rx_buf;           /* Pointer to in data buffer                               */
  uint32_t                rx_total_num;     /* Total number of data to be received                     */
  uint32_t                rx_curr_cnt;      /* Number of data received                                 */
  uint32_t                rx_curr_tx_index; /* current index Number which needs to send while receive. */
  uint32_t                curr_cnt;         /* common current count for both TX and RX, we can update in ARM_I2C_GetDataCount function */
} i2c_transfer_info_t;

/* i2c Information (Run-Time) */
/* Structure to save i2c settings and statuses */
typedef struct i2c_info
{
  ARM_I2C_SignalEvent_t   cb_event;         /* Event callback                                                */
  ARM_I2C_STATUS          status;           /* I2C status                                                    */
  i2c_transfer_info_t     transfer;         /* Transfer information                                          */
  bool                    pending;          /* Transfer pending (no STOP) pending for interrupt only         */
  uint8_t                 stalled;          /* temporary on hold Stall mode status flags                     */
  uint32_t                mode;             /* current working mode, which can be \ref DEV_MASTER_MODE "master mode" or \ref DEV_SLAVE_MODE "slave mode"    */
  uint32_t                speed_mode;       /* current working \ref I2C_SPEED_MODE "i2c speed mode"                                                         */
  uint32_t                cur_state;        /* \ref I2C_WORKING_STATE "current working state for i2c device", this should be \ref I2C_FREE for first open   */
  uint32_t                err_state;        /* \ref I2C_ERROR_STATE "current error state for i2c device", this should be \ref I2C_ERR_NONE for first open   */
  uint32_t                addr_mode;        /* \ref I2C_ADDRESS_MODE "current addressing mode", this should be \ref I2C_7BIT_ADDRESS for first open         */
  uint32_t                slv_addr;         /* slave address when working as slave i2c device, this should be 0 for first open                              */
  uint32_t                tar_addr;         /* target slave device address when addressing that slave device, this should be 0 for first open               */
  uint32_t                next_cond;        /* \ref I2C_NEXT_CONDTION "next condition for master transmit or receive", \
                                               possible values are STOP or RESTART, it should be STOP for first open                                        */
  uint32_t                retry_cnt;        /* retry count for TX or RX                                                                                     */
  uint32_t                tx_over;          /* i2c tx overflow count                                          */
  uint32_t                rx_over;          /* i2c rx overflow count                                          */
  uint8_t                 flags;            /* i2c driver flags                                               */
  uint32_t                int_status;       /* interrupt status for i2c                                       */
  uint32_t                tx_fifo_length;   /* transmit fifo length, set by user in object implementation     */
  uint32_t                rx_fifo_length;   /* receive  fifo length, set by user in object implementation     */
  i2c_spike_length_t      i2c_spklen;       /* i2c spike suppression length settings                          */
  i2c_scl_cnt_t           i2c_scl_cnt;      /* i2c scl count settings  */
} i2c_info_t;

/* i2c Resources definitions */
/* @brief Structure to save contexts for a i2c channel */
typedef struct i2c_resources
{
    uint32_t     reg_base;                  /* i2c register base address    */
    uint32_t     clk;                       /* system clock                 */
    i2c_info_t   *info;                     /* i2c device information       */
    IRQn_Type    irq_num;                   /* i2c interrupt vector number  */
    uint32_t     irq_priority;              /* i2c interrupt priority       */
} i2c_resources_t;

/* function declarations */
void     i2c_initialize (i2c_resources_t *i2c);
void     i2c_uninitialize (i2c_resources_t *i2c);
void     i2c_reset_device(i2c_resources_t *i2c);
int32_t  i2c_master_init (i2c_resources_t *i2c, uint32_t i2c_bus_speed);
int32_t  i2c_slave_init (i2c_resources_t *i2c, uint32_t i2c_slave_addr);

void     i2c_irq_handler (i2c_resources_t *i2c);
int32_t  i2c_enable_master_transmit_interrupt (i2c_resources_t *i2c, uint32_t addr, const uint8_t *data, uint32_t num, bool xfer_pending);
int32_t  i2c_enable_master_receive_interrupt  (i2c_resources_t *i2c, uint32_t addr, uint8_t *data, uint32_t num, bool xfer_pending);
int32_t  i2c_enable_slave_transmit_interrupt (i2c_resources_t *i2c, const uint8_t *data, uint32_t num);
int32_t  i2c_enable_slave_receive_interrupt  (i2c_resources_t *i2c, uint8_t *data, uint32_t num);
void     i2c_abort_transmit(i2c_resources_t *i2c);
void     i2c_abort_receive(i2c_resources_t *i2c);

#ifdef  __cplusplus
}
#endif

#endif /* I2C_LL_DRV_H_ */
