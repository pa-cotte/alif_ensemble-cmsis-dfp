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
 * @file     PDM_dev.h
 * @author   Nisarga A M
 * @email    nisarga.am@alifsemi.com
 * @version  V1.0.0
 * @date     12-Jan-2023
 * @brief    CMSIS-Driver for PDM.
 * @bug      None.
 * @Note     None
 ******************************************************************************/

#ifndef PDM_DEV_H_
#define PDM_DEV_H_

#ifdef  __cplusplus
extern "C"
{
#endif

/* System includes */
#include "RTE_Device.h"
#include "RTE_Components.h"
#include CMSIS_device_header

/* include for PDM Driver */
#include "Driver_PDM.h"

#define PDM_IRQ_ENABLE                (0xFF03)                  /* To enable the interrupt status register */

#define PDM_BYPASS_IIR                (1 << 2)                  /* Bypass DC blocking IIR filter */
#define PDM_BYPASS_FIR                (1 << 3)                  /* Bypass FIR filter */
#define PDM_PEAK_DETECT_NODE          (1 << 4)                  /* Peak detection node */
#define PDM_DMA_HANDSHAKE             (1 << 24)                 /* DMA handshaking signals for flow control */
#define PDM_SAMPLE_ADV                (1 << 17)                 /* Sample advance */

#define PDM_INTERRUPT_STATUS_VALUE    (0x1)                     /* To check the interrupt status */

#define PDM_FIFO_ALMOST_FULL_IRQ      (0x1 << 0)                /* FIFO almost full Interrupt */
#define PDM_FIFO_OVERFLOW_IRQ         (0x1 << 1)                /* FIFO overflow interrupt */
#define PDM_AUDIO_DETECT_IRQ          (0xFF << 8)               /* Audio detect interrupt */
#define PDM_CHANNEL_ENABLE            (0xFF)                    /* To check the which channel is enabled */
#define PDM_MODES                     (0xFF << 16)              /* To check for the PDM modes */
#define PDM_CLOCK_CONFIG              (0x100)                   /* PDM clock configuration */

#define PDM_FIFO_CLEAR                (1 << 31)                 /* To clear FIFO clear bit */

#define PDM_AUDIO_CHANNEL             (0x3)
#define PDM_CHANNEL_0_1               (PDM_AUDIO_CHANNEL << 0)  /* check for channel 0 and 1 */
#define PDM_CHANNEL_2_3               (PDM_AUDIO_CHANNEL << 2)  /* check for channel 2 and 3 */
#define PDM_CHANNEL_4_5               (PDM_AUDIO_CHANNEL << 4)  /* check for channel 4 and 5 */
#define PDM_CHANNEL_6_7               (PDM_AUDIO_CHANNEL << 6)  /* check for channel 6 and 7 */

#define PDM_AUDIO_CH_0_1              0                         /* PDM audio channel 0 and 1 */
#define PDM_AUDIO_CH_2_3              1                         /* PDM audio channel 2 and 3 */
#define PDM_AUDIO_CH_4_5              2                         /* PDM audio channel 4 and 5 */
#define PDM_AUDIO_CH_6_7              3                         /* PDM audio channel 6 and 7 */
#define PDM_CLK_MODE                  16                        /* PDM clock frequency mode */

/**
 @brief   : PDM flags to check the PDM initialization and PDM power done.
 */
typedef enum {
    PDM_FLAG_DRV_INIT_DONE    = (1U << 0),  /*PDM Driver is Initialized */
    PDM_FLAG_DRV_POWER_DONE   = (1U << 1),  /*PDM Driver is Powered */
} PDM_FLAG_Type;

/**
 @brief : Channel Configuration and Status Registers
 */
typedef struct {
    __IOM uint32_t CH_FIR_COEF[PDM_MAX_FIR_COEFFICIENT]; /* channel Fir coefficient */
    __IOM uint32_t RESERVED2[14];                    /* reserved from (0x88) to (0xBC) */
    __IOM uint32_t IIR_COEF_SEL;                     /* IIR Filter Coefficient Selection register */
    __IOM uint32_t PHASE_CONTROL;                    /* Phase Control Register */
    __IOM uint32_t GAIN;                             /* Gain Register */
    __IOM uint32_t PKDET_TH;                         /* Peak Detector Threshold Register */
    __IOM uint32_t PKDET_ITV;                        /* Peak Detector Interval Register */
    __IM  uint32_t PKDET_STAT;                       /* Peak Detector Status Register */
    __IM  uint32_t RESERVERD3[26];                   /* reserved register */
} PDM_CONFG_INFO;

/**
 @brief struct PDM_TypeDef:- Register map for PDM
 */
typedef struct{
    __IOM uint32_t AUDIO_CNTRL_0;            /* PDM Audio control Register 0 */
    __IOM uint32_t AUDIO_CNTRL_1;            /* PDM Audio control Register 1 */
    __IOM uint32_t FIFO_WATERMARK;         /* FIFO Almost-Full Watermark Threshold Register */
    __IM  uint32_t FIFO_STAT;                /* FIFO Status Register */
    __IM  uint32_t ERROR_STAT;                /* Error Interrupt Status Register */
    __IM  uint32_t WARNING_STAT;              /* Warnings Interrupt Status Register */
    __IM  uint32_t AUDIO_detect_STAT;         /* Audio Detection Interrupt Status Register */
    __IOM uint32_t IRQ_ENABLE;               /* Interrupt Enable Register */
    __IOM uint32_t AUDIO_OUT[4];             /* Audio Out Registers */
    __IM  uint32_t RESERVED1[4];             /* Reserved from (0x30) to (0x3C) */
    PDM_CONFG_INFO CH_CNFG[PDM_MAX_CHANNEL]; /* Channel Configuration and Status Registers */
}LPPDM_Type;

/**
 @brief struct PDM_info_t:- To store PDM Capture Configuration
 */
typedef struct{
    uint32_t en_channel;      /* Multiple channel */
    uint32_t curr_cnt;        /* Current count value */
    uint32_t total_cnt;       /* Total count value */
    uint32_t *ch0_1_addr;     /* Channel 0 and 1 audio output values are stored in this address */
    uint32_t *ch2_3_addr;     /* Channel 2 and 3 audio output values are stored in this address */
    uint32_t *ch4_5_addr;     /* Channel 4 and 5 audio output values are stored in this address */
    uint32_t *ch6_7_addr;     /* Channel 6 and 7 audio output values are stored in this address */
}PDM_info_t;

/**
 * Access structure for the saving the PDM Setting and status
 */
typedef struct PDM_resources
{
    ARM_PDM_SignalEvent_t             cb_event;             /* PDM application event callback */
    LPPDM_Type                       *reg_base;            /* PDM register address */
    PDM_info_t                        info;                 /* To store PDM Capture Configuration*/
    uint8_t                           flags;                /* PDM Driver Flags */
    IRQn_Type                         error_irq;            /* PDM error IRQ number */
    IRQn_Type                         warning_irq;          /* PDM warning IRQ number */
    IRQn_Type                         audio_detect_irq;     /* PDM audio detect IRQ number */
    uint32_t                          error_irq_priority;   /*PDM error IRQ priority */
    uint32_t                          warning_irq_priority; /*PDM warning IRQ priority */
    uint32_t                          audio_irq_priority;   /*PDM audio IRQ priority */
}PDM_resources_t;

#endif /* PDM_DEV_H_ */
