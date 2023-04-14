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
 * @file     Driver_PDM.h
 * @author   Nisarga A M
 * @email    nisarga.am@alifsemi.com
 * @version  V1.0.0
 * @date     15-Jan-2023
 * @brief    CMSIS-Driver for PDM.
 * @bug      None.
 * @Note     None
 ******************************************************************************/

#ifndef DRIVER_PDM_H_
#define DRIVER_PDM_H_

#include "Driver_Common.h"

#ifdef _cplusplus
extern "c"
{
#endif

#define ARM_PDM_API_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(1,0)  /* API version */

#define ARM_PDM_MODE                                        0x00UL

/* Control code for PDM */
#define ARM_PDM_MODE_MICROPHONE_SLEEP                       0x00UL
#define ARM_PDM_MODE_STANDARD_VOICE_512_CLK_FRQ             0x01UL
#define ARM_PDM_MODE_HIGH_QUALITY_512_CLK_FRQ               0x02UL
#define ARM_PDM_MODE_HIGH_QUALITY_768_CLK_FRQ               0x03UL
#define ARM_PDM_MODE_HIGH_QUALITY_1024_CLK_FRQ              0x04UL
#define ARM_PDM_MODE_WIDE_BANDWIDTH_AUDIO_1536_CLK_FRQ      0x05UL
#define ARM_PDM_MODE_FULL_BANDWIDTH_AUDIO_2400_CLK_FRQ      0x06UL
#define ARM_PDM_MODE_FULL_BANDWIDTH_AUDIO_3071_CLK_FRQ      0x07UL
#define ARM_PDM_MODE_ULTRASOUND_4800_CLOCK_FRQ              0x08UL
#define ARM_PDM_MODE_ULTRASOUND_96_SAMPLING_RATE            0x09UL

#define ARM_PDM_BYPASS_IIR_FILTER                           0x0AUL
#define ARM_PDM_BYPASS_FIR_FILTER                           0x0BUL

#define ARM_PDM_PEAK_DETECTION_NODE                         0x0CUL
#define ARM_PDM_SAMPLE_ADVANCE                              0x0DUL
#define ARM_PDM_DMA_HANDSHAKE                               0x0EUL

/* PDM event */
#define ARM_PDM_EVENT_ERROR                                (1UL << 0)
#define ARM_PDM_EVENT_CAPTURE_COMPLETE                     (1UL << 1)
#define ARM_PDM_EVENT_AUDIO_DETECTION                      (1UL << 2)

/* PDM maximum channel */
#define PDM_MAX_CHANNEL                                     8

/* PDM channel FIR length */
#define PDM_MAX_FIR_COEFFICIENT                             18

/* PDM channels */
#define ARM_PDM_AUDIO_CHANNEL_0                            (1 << 0)
#define ARM_PDM_AUDIO_CHANNEL_1                            (1 << 1)
#define ARM_PDM_AUDIO_CHANNEL_2                            (1 << 2)
#define ARM_PDM_AUDIO_CHANNEL_3                            (1 << 3)
#define ARM_PDM_AUDIO_CHANNEL_4                            (1 << 4)
#define ARM_PDM_AUDIO_CHANNEL_5                            (1 << 5)
#define ARM_PDM_AUDIO_CHANNEL_6                            (1 << 6)
#define ARM_PDM_AUDIO_CHANNEL_7                            (1 << 7)

typedef void (*ARM_PDM_SignalEvent_t) (uint32_t event);  /*Pointer to \ref PDM_SignalEvent : Signal PDM Event*/

/**
 @brief: These channel configurations are specific to each channels
 */
typedef struct _PDM_CH_CONFIG {
    uint8_t ch_num;                 /* Channel number */
    uint32_t ch_fir_coef[PDM_MAX_FIR_COEFFICIENT]; /* Channel FIR filter Coefficient */
    uint32_t ch_iir_coef;           /* Channel IIR Filter Coefficient */
    uint32_t ch_phase;              /* Channel Phase Control */
    uint32_t ch_gain;               /* Channel gain control */
    uint32_t ch_peak_detect_th;     /* Channel Peak Detector Threshold */
    uint32_t ch_peak_detect_itv;    /* Channel Peak Detector Interval */
    uint32_t ch_peak_detect_stat;   /* Channel Peak Detector Status */
}PDM_CH_CONFIG;

/**
 * brief: PDM Capture Configuration
 */
typedef struct _PDM_Capture_CONFIG {
    uint8_t en_multiple_ch;         /* Select the multiple channel */
    uint32_t *ch0_1_addr;           /* Channel 0 and 1 audio output values are stored in this address provided by user */
    uint32_t *ch2_3_addr;           /* Channel 2 and 3 audio output values are stored in this address provided by user */
    uint32_t *ch4_5_addr;           /* Channel 4 and 5 audio output values are stored in this address provided by user */
    uint32_t *ch6_7_addr;           /* Channel 6 and 7 audio output values are stored in this address provided by user */
    uint32_t total_no_samples;      /* Store total number of samples */
    uint32_t fifo_watermark;        /* Store fifo watermark threshold value */
}PDM_Capture_CONFIG;

/**
 @brief : PDM Driver Capabilities
 */
typedef struct _ARM_PDM_CAPABILITIES{
    uint32_t MICROPHONE_SLEEP_MODE        :1;   /* Supports Microphone sleep mode*/
    uint32_t STANDARD_VOICE_MODE          :1;   /* Supports Standard voice mode */
    uint32_t HIGH_QUALITY_VOICE_MODE      :1;   /* Supports High quality voice mode */
    uint32_t WIDE_BANDWIDTH_AUDIO_MODE    :1;   /* Supports Wide bandwidth audio mode */
    uint32_t FULL_BANDWIDTH_AUDIO_MODE    :1;   /* Supports Full bandwidth audio mode */
    uint32_t ULTRASOUND_MODE              :1;   /* Supports Ultrasound mode */
    uint32_t reserved                     :27;  /* Reserved (must be Zero) */
}ARM_PDM_CAPABILITIES;

/**
 @brief  Access Structure of PDM Driver
*/
typedef struct ARM_DRIVER_PDM{
    ARM_DRIVER_VERSION            (*GetVersion)        (void);                                 /* pointer is pointing to PDM_GetVersion : used to get the driver version */
    ARM_PDM_CAPABILITIES          (*GetCapabilities)   (void);                                 /* pointer is pointing to PDM_Capabilities : used to get the driver capabilities */
    int32_t                       (*Initialize)        (ARM_PDM_SignalEvent_t cb_event);       /* Pointer pointing to \ref PDM_intialize */
    int32_t                       (*Uninitialize)      (void);                                 /* Pointer to PDM_Uninitialize : Un-initialize comparator Interface */
    int32_t                       (*PowerControl)      (ARM_POWER_STATE state);                /* Pointer to PDM_PowerControl : Control Comparator Interface Power */
    int32_t                       (*Control)           (uint32_t control, uint32_t arg);       /* Pointer to PDM_Control : Control Comparator Interface */
    int32_t                       (*Config)            (PDM_CH_CONFIG *cnfg);                  /* Pointer to PDM Config: Channel configurations specific to each channel */
    int32_t                       (*Capture)           (PDM_Capture_CONFIG *cap_cnfg) ;        /* Pointer to PDM_Capture : PDM Capture Configuration */
}const ARM_DRIVER_PDM;

#endif /* DRIVER_PDM_H_ */
