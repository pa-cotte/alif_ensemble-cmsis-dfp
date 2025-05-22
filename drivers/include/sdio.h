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
 * @file     sdio.h
 * @author   Deepak Kumar
 * @email    deepak@alifsemi.com
 * @version  V0.0.1
 * @date     28-Nov-2022
 * @brief    exposed SDIO Driver variables and APIs.
 * @bug      None.
 * @Note     None
 ******************************************************************************/

#ifndef _SDIO_H_
#define _SDIO_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* SDIO commands                         type  argument     response */
#define SDIO_SEND_OP_COND          5U /* bcr  [23:0] OCR         R4  */
#define SDIO_RW_DIRECT            52U /* ac   [31:0] See below   R5  */
#define SDIO_RW_EXTENDED          53U /* adtc [31:0] See below   R5  */

/*
 * SD_IO_RW_DIRECT argument format:
 *
 *      [31] R/W flag
 *      [30:28] Function number
 *      [27] RAW flag
 *      [25:9] Register address
 *      [7:0] Data
 */
#define SDIO_RW_FLAG_Pos   (31U)
#define SDIO_RW_FLAG_Msk   (1U << SDIO_RW_FLAG_Pos)
#define SDIO_FN_Pos        (28U)
#define SDIO_RAW_FLAG_Pos  (27U)
#define SDIO_RAW_FLAG_Msk  (1U << SDIO_RAW_FLAG_Pos)
#define SDIO_REG_ADDR_Pos  (9U)

/*
 * SD_IO_RW_EXTENDED argument format:
 *
 *      [31] R/W flag
 *      [30:28] Function number
 *      [27] Block mode
 *      [26] Increment address
 *      [25:9] Register address
 *      [8:0] Byte/block count
 */
#define SDIO_RW_EXT_BLK_MODE_Pos   (27U)
#define SDIO_RW_EXT_BLK_MODE_Msk   (1U << SDIO_RW_EXT_BLK_MODE_Pos)
#define SDIO_RW_EXT_INCR_ADDR_Pos  (26U)
#define SDIO_RW_EXT_INCR_ADDR_Msk  (1U << SDIO_RW_EXT_INCR_ADDR_Pos)

#define CMD5_RESP_IO_READY_Msk     (1U << 31U)
#define CMD5_RESP_18V_PRES_Msk     (1U << 24U)
#define CMD5_RESP_MEMORY_PRES_Msk  (1U << 27U)
#define CMD5_RESP_NIOF_PRES_Pos    28U
#define CMD5_RESP_NIOF_PRES_Msk    (7U << CMD5_RESP_NIOF_PRES_Pos)
#define CMD5_RESP_OCR_Msk          0x00FFFFFFU
#define CMD5_RESP_OCR_3V3_Msk      (1 << 21U)

/*
   SDIO status in R5
   Type
    e : error bit
    s : status bit
    r : detected and set for the actual command response
    x : detected and set during command execution. the host must poll
    the card by sending status command in order to read these bits.
    Clear condition
    a : according to the card state
    b : always related to the previous command. Reception of
    a valid command will clear it (with a delay of one command)
    c : clear by read
*/

#define SDIO_R5_COM_CRC_ERROR        (1U << 15U)    /* er, b   */
#define SDIO_R5_ILLEGAL_COMMAND      (1U << 14U)    /* er, b   */
#define SDIO_R5_ERROR                (1U << 11U)    /* erx, c  */
#define SDIO_R5_FUNCTION_NUMBER      (1U << 9U)     /* er, c   */
#define SDIO_R5_OUT_OF_RANGE         (1U << 8U)     /* er, c   */
#define SDIO_R5_STATUS(x)            (x & 0xCB00U)
#define SDIO_R5_IO_CURRENT_STATE(x)  ((x & 0x3000U) >> 12U) /* s, b */
#define SDIO_MAX_CIA_ADDR       (0x1FFFFU)
#define SDIO_MAX_FUNCTION       (7U)

/*
 * Card Common Control Registers (CCCR)
 */

#ifndef SDIO_CCCR_CCCR
#define SDIO_CCCR_CCCR          0x00U
#endif
#ifndef SDIO_CCCR_REV_1_00
#define SDIO_CCCR_REV_1_00      0U    /* CCCR/FBR Version 1.00 */
#endif
#ifndef SDIO_CCCR_REV_1_10
#define SDIO_CCCR_REV_1_10      1U    /* CCCR/FBR Version 1.10 */
#endif
#ifndef SDIO_CCCR_REV_1_20
#define SDIO_CCCR_REV_1_20      2U    /* CCCR/FBR Version 1.20 */
#endif
#ifndef SDIO_CCCR_REV_3_00
#define SDIO_CCCR_REV_3_00      3U    /* CCCR/FBR Version 3.00 */
#endif
#ifndef SDIO_SDIO_REV_1_00
#define SDIO_SDIO_REV_1_00      0U    /* SDIO Spec Version 1.00 */
#endif
#ifndef SDIO_SDIO_REV_1_10
#define SDIO_SDIO_REV_1_10      1U    /* SDIO Spec Version 1.10 */
#endif
#ifndef SDIO_SDIO_REV_1_20
#define SDIO_SDIO_REV_1_20      2U    /* SDIO Spec Version 1.20 */
#endif
#ifndef SDIO_SDIO_REV_2_00
#define SDIO_SDIO_REV_2_00      3U    /* SDIO Spec Version 2.00 */
#endif
#ifndef SDIO_SDIO_REV_3_00
#define SDIO_SDIO_REV_3_00      4U    /* SDIO Spec Version 3.00 */
#endif
#ifndef SDIO_CCCR_SD
#define SDIO_CCCR_SD            0x01U
#endif
#ifndef SDIO_SD_REV_1_01
#define SDIO_SD_REV_1_01        0U    /* SD Physical Spec Version 1.01 */
#endif
#ifndef SDIO_SD_REV_1_10
#define SDIO_SD_REV_1_10        1U    /* SD Physical Spec Version 1.10 */
#endif
#ifndef SDIO_SD_REV_2_00
#define SDIO_SD_REV_2_00        2U    /* SD Physical Spec Version 2.00 */
#endif
#ifndef SDIO_SD_REV_3_00
#define SDIO_SD_REV_3_00        3U    /* SD Physical Spec Version 3.00 */
#endif
#ifndef SDIO_CCCR_IOEx
#define SDIO_CCCR_IOEx          0x02U
#endif
#ifndef SDIO_CCCR_IORx
#define SDIO_CCCR_IORx          0x03U
#endif
#ifndef SDIO_CCCR_IENx
#define SDIO_CCCR_IENx          0x04U    /* Function/Master Interrupt Enable */
#endif
#ifndef SDIO_CCCR_INTx
#define SDIO_CCCR_INTx          0x05U    /* Function Interrupt Pending */
#endif
#ifndef SDIO_CCCR_ABORT
#define SDIO_CCCR_ABORT         0x06U    /* function abort/card reset */
#endif
#ifndef SDIO_CCCR_IF
#define SDIO_CCCR_IF            0x07U    /* bus interface controls */
#endif
#ifndef SDIO_BUS_WIDTH_Msk
#define SDIO_BUS_WIDTH_Msk      0x03U    /* data bus width setting */
#endif
#ifndef SDIO_BUS_WIDTH_1BIT
#define SDIO_BUS_WIDTH_1BIT     0x00U
#endif
#ifndef SDIO_BUS_WIDTH_RESERVED
#define SDIO_BUS_WIDTH_RESERVED 0x01U
#endif
#ifndef SDIO_BUS_WIDTH_4BIT
#define SDIO_BUS_WIDTH_4BIT     0x02U
#endif
#ifndef SDIO_BUS_ECSI
#define SDIO_BUS_ECSI           0x20U    /* Enable continuous SPI interrupt */
#endif
#ifndef SDIO_BUS_SCSI
#define SDIO_BUS_SCSI           0x40U    /* Support continuous SPI interrupt */
#endif
#ifndef SDIO_BUS_ASYNC_INT
#define SDIO_BUS_ASYNC_INT      0x20U
#endif
#ifndef SDIO_BUS_CD_DISABLE
#define SDIO_BUS_CD_DISABLE     0x80U    /* disable pull-up on DAT3 (pin 1) */
#endif
#ifndef SDIO_CCCR_CAPS
#define SDIO_CCCR_CAPS          0x08U
#endif
#ifndef SDIO_CCCR_CAP_SDC
#define SDIO_CCCR_CAP_SDC       0x01U    /* can do CMD52 while data transfer */
#endif
#ifndef SDIO_CCCR_CAP_SMB
#define SDIO_CCCR_CAP_SMB       0x02U    /* can do multi-block xfers (CMD53) */
#endif
#ifndef SDIO_CCCR_CAP_SRW
#define SDIO_CCCR_CAP_SRW       0x04U    /* supports read-wait protocol */
#endif
#ifndef SDIO_CCCR_CAP_SBS
#define SDIO_CCCR_CAP_SBS       0x08U    /* supports suspend/resume */
#endif
#ifndef SDIO_CCCR_CAP_S4MI
#define SDIO_CCCR_CAP_S4MI      0x10U    /* interrupt during 4-bit CMD53 */
#endif
#ifndef SDIO_CCCR_CAP_E4MI
#define SDIO_CCCR_CAP_E4MI      0x20U    /* enable ints during 4-bit CMD53 */
#endif
#ifndef SDIO_CCCR_CAP_LSC
#define SDIO_CCCR_CAP_LSC       0x40U    /* low speed card */
#endif
#ifndef SDIO_CCCR_CAP_4BLS
#define SDIO_CCCR_CAP_4BLS      0x80U    /* 4 bit low speed card */
#endif
#ifndef SDIO_CCCR_CIS
#define SDIO_CCCR_CIS           0x09U    /* common CIS pointer (3 bytes) */
#endif
#ifndef SDIO_CCCR_SUSPEND
#define SDIO_CCCR_SUSPEND           0x0CU
#endif
#ifndef SDIO_CCCR_SELx
#define SDIO_CCCR_SELx              0x0DU
#endif
#ifndef SDIO_CCCR_EXECx
#define SDIO_CCCR_EXECx             0x0EU
#endif
#ifndef SDIO_CCCR_READYx
#define SDIO_CCCR_READYx            0x0FU
#endif
#ifndef SDIO_CCCR_BLKSIZE
#define SDIO_CCCR_BLKSIZE           0x10U
#endif
#ifndef SDIO_CCCR_POWER
#define SDIO_CCCR_POWER             0x12U
#endif
#ifndef SDIO_POWER_SMPC
#define  SDIO_POWER_SMPC            0x01U    /* Supports Master Power Control */
#endif
#ifndef SDIO_POWER_EMPC
#define  SDIO_POWER_EMPC            0x02U    /* Enable Master Power Control */
#endif
#ifndef SDIO_CCCR_SPEED
#define SDIO_CCCR_SPEED             0x13U
#endif
#ifndef SDIO_SPEED_SHS
#define  SDIO_SPEED_SHS             0x01U    /* Supports High-Speed mode */
#endif
#ifndef SDIO_SPEED_BSS_SHIFT
#define  SDIO_SPEED_BSS_SHIFT       1U
#endif
#ifndef SDIO_SPEED_BSS_Msk
#define  SDIO_SPEED_BSS_Msk         (7U << SDIO_SPEED_BSS_SHIFT)
#endif
#ifndef SDIO_SPEED_SDR12
#define  SDIO_SPEED_SDR12           (0U << SDIO_SPEED_BSS_SHIFT)
#endif
#ifndef SDIO_SPEED_SDR25
#define  SDIO_SPEED_SDR25           (1U << SDIO_SPEED_BSS_SHIFT)
#endif
#ifndef SDIO_SPEED_SDR50
#define  SDIO_SPEED_SDR50           (2U << SDIO_SPEED_BSS_SHIFT)
#endif
#ifndef SDIO_SPEED_SDR104
#define  SDIO_SPEED_SDR104          (3U << SDIO_SPEED_BSS_SHIFT)
#endif
#ifndef SDIO_SPEED_DDR50
#define  SDIO_SPEED_DDR50           (4U << SDIO_SPEED_BSS_SHIFT)
#endif
#ifndef SDIO_SPEED_EHS
#define  SDIO_SPEED_EHS             SDIO_SPEED_SDR25    /* Enable High-Speed */
#endif
#ifndef SDIO_CCCR_UHS
#define SDIO_CCCR_UHS               0x14U
#endif
#ifndef SDIO_UHS_SDR50
#define  SDIO_UHS_SDR50             0x01U
#endif
#ifndef SDIO_UHS_SDR104
#define  SDIO_UHS_SDR104            0x02U
#endif
#ifndef SDIO_UHS_DDR50
#define  SDIO_UHS_DDR50             0x04U
#endif
#ifndef SDIO_CCCR_DRIVE_STRENGTH
#define SDIO_CCCR_DRIVE_STRENGTH    0x15U
#endif
#ifndef SDIO_SDTx_Msk
#define SDIO_SDTx_Msk               0x07U
#endif
#ifndef SDIO_DRIVE_SDTA
#define SDIO_DRIVE_SDTA             (1U << 0U)
#endif
#ifndef SDIO_DRIVE_SDTC
#define SDIO_DRIVE_SDTC             (1U << 1U)
#endif
#ifndef SDIO_DRIVE_SDTD
#define SDIO_DRIVE_SDTD             (1U << 2U)
#endif
#ifndef SDIO_DRIVE_DTSx_Msk
#define SDIO_DRIVE_DTSx_Msk         0x03U
#endif
#ifndef SDIO_DRIVE_DTSx_SHIFT
#define SDIO_DRIVE_DTSx_SHIFT       4U
#endif
#ifndef SDIO_DTSx_SET_TYPE_B
#define SDIO_DTSx_SET_TYPE_B        (0U << SDIO_DRIVE_DTSx_SHIFT)
#endif
#ifndef SDIO_DTSx_SET_TYPE_A
#define SDIO_DTSx_SET_TYPE_A        (1U << SDIO_DRIVE_DTSx_SHIFT)
#endif
#ifndef SDIO_DTSx_SET_TYPE_C
#define SDIO_DTSx_SET_TYPE_C        (2U << SDIO_DRIVE_DTSx_SHIFT)
#endif
#ifndef SDIO_DTSx_SET_TYPE_D
#define SDIO_DTSx_SET_TYPE_D        (3U << SDIO_DRIVE_DTSx_SHIFT)
#endif
#ifndef SDIO_CCCR_INTERRUPT_EXT
#define SDIO_CCCR_INTERRUPT_EXT     0x16U
#endif
#ifndef SDIO_INTERRUPT_EXT_SAI
#define SDIO_INTERRUPT_EXT_SAI      (1U << 0U)
#endif
#ifndef SDIO_INTERRUPT_EXT_EAI
#define SDIO_INTERRUPT_EXT_EAI      (1U << 1U)
#endif
/*
 * Function Basic Registers (FBR)
 */

#ifndef SDIO_FBR_BASE
#define SDIO_FBR_BASE(f)            ((f) * 0x100U) /* base of function f's FBRs */
#endif
#ifndef SDIO_FBR_STD_IF
#define SDIO_FBR_STD_IF             0x00U
#endif
#ifndef SDIO_FBR_SUPPORTS_CSA
#define SDIO_FBR_SUPPORTS_CSA       0x40U    /* supports Code Storage Area */
#endif
#ifndef SDIO_FBR_ENABLE_CSA
#define SDIO_FBR_ENABLE_CSA         0x80U    /* enable Code Storage Area */
#endif
#ifndef SDIO_FBR_STD_IF_EXT
#define SDIO_FBR_STD_IF_EXT         0x01U
#endif
#ifndef SDIO_FBR_POWER
#define SDIO_FBR_POWER              0x02U
#endif
#ifndef SDIO_FBR_POWER_SPS
#define SDIO_FBR_POWER_SPS          0x01U    /* Supports Power Selection */
#endif
#ifndef SDIO_FBR_POWER_EPS
#define SDIO_FBR_POWER_EPS          0x02U    /* Enable (low) Power Selection */
#endif
#ifndef SDIO_FBR_CIS
#define SDIO_FBR_CIS                0x09U    /* CIS pointer (3 bytes) */
#endif
#ifndef SDIO_FBR_CSA
#define SDIO_FBR_CSA                0x0CU    /* CSA pointer (3 bytes) */
#endif
#ifndef SDIO_FBR_CSA_DATA
#define SDIO_FBR_CSA_DATA           0x0FU
#endif
#ifndef SDIO_FBR_BLKSIZE
#define SDIO_FBR_BLKSIZE            0x10U    /* block size (2 bytes) */
#endif

typedef struct _sdio_opcond_t{
    uint8_t isInitialized;  /*!< indicates Card is initialized              */
    uint8_t func_number;    /*!< Number of function available in the card   */
    uint8_t memory_present; /*!< Memory Present Flag                        */
    uint8_t s18a;           /*!< low voltage support flag                   */
    uint8_t ocr;            /*!< operating voltage                          */
}sdio_opcond_t;

typedef struct _sdio_t{
    sdio_opcond_t sdio_opcd; /*!< sdio card information */
}sdio_t;

#ifdef __cplusplus
}
#endif

#endif
