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
 * @file     peripheral_types.h
 * @author   Rupesh Kumar
 * @email    rupesh@alifsemi.com
 * @brief    System peripherals types declarations
 * @version  V1.0.0
 * @date     14 Apr 2023
 * @bug      None
 * @Note     None
 ******************************************************************************/
#ifndef PERIPHERAL_TYPES_H
#define PERIPHERAL_TYPES_H

#include "global_map.h"

#if defined (M55_HP)
 #include "M55_HP.h"
#elif defined (M55_HE)
 #include "M55_HE.h"
#else
 #error device not specified!
#endif

#ifdef __cplusplus
extern "C" {
#endif


/* =========================================================================================================================== */
/* ================                                      CLKCTL_PER_MST                                       ================ */
/* =========================================================================================================================== */


/**
  * @brief CLKCTL_PER_MST (CLKCTL_PER_MST)
  */

typedef struct {                                /*!< (@ 0x4903F000) CLKCTL_PER_MST Structure                                   */
  __IOM uint32_t  CAMERA_PIXCLK_CTRL;           /*!< (@ 0x00000000) CPI Pixel Clock Control Register                           */
  __IOM uint32_t  CDC200_PIXCLK_CTRL;           /*!< (@ 0x00000004) CDC Pixel Clock Control Register                           */
  __IOM uint32_t  CSI_PIXCLK_CTRL;              /*!< (@ 0x00000008) CSI Pixel Clock Control Register                           */
  __IOM uint32_t  PERIPH_CLK_ENA;               /*!< (@ 0x0000000C) Peripheral Clock Enable Register                           */
  __IOM uint32_t  DPHY_PLL_CTRL0;               /*!< (@ 0x00000010) MIPI-DPHY PLL Control Register 0                           */
  __IOM uint32_t  DPHY_PLL_CTRL1;               /*!< (@ 0x00000014) MIPI-DPHY PLL Control Register 1                           */
  __IOM uint32_t  DPHY_PLL_CTRL2;               /*!< (@ 0x00000018) MIPI-DPHY PLL Control Register 2                           */
  __IM  uint32_t  RESERVED;
  __IM  uint32_t  DPHY_PLL_STAT0;               /*!< (@ 0x00000020) MIPI-DPHY PLL Status Register 0                            */
  __IM  uint32_t  DPHY_PLL_STAT1;               /*!< (@ 0x00000024) MIPI-DPHY PLL Status Register 1                            */
  __IM  uint32_t  RESERVED1[2];
  __IOM uint32_t  TX_DPHY_CTRL0;                /*!< (@ 0x00000030) MIPI-DPHY TX Control Register 0                            */
  __IOM uint32_t  TX_DPHY_CTRL1;                /*!< (@ 0x00000034) MIPI-DPHY TX Control Register 1                            */
  __IOM uint32_t  RX_DPHY_CTRL0;                /*!< (@ 0x00000038) MIPI-DPHY RX Control Register 0                            */
  __IOM uint32_t  RX_DPHY_CTRL1;                /*!< (@ 0x0000003C) MIPI-DPHY RX Control Register 1                            */
  __IOM uint32_t  MIPI_CKEN;                    /*!< (@ 0x00000040) MIPI-DPHY Clock Enable Register                            */
  __IOM uint32_t  DSI_CTRL;                     /*!< (@ 0x00000044) DSI Control Register                                       */
  __IM  uint32_t  RESERVED2[10];
  __IOM uint32_t  DMA_CTRL;                     /*!< (@ 0x00000070) DMA0 Boot Control Register                                 */
  __IOM uint32_t  DMA_IRQ;                      /*!< (@ 0x00000074) DMA0 Boot IRQ Non-Secure Register                          */
  __IOM uint32_t  DMA_PERIPH;                   /*!< (@ 0x00000078) DMA0 Boot Peripheral Non-Secure Register                   */
  __IOM uint32_t  DMA_GLITCH_FLT;               /*!< (@ 0x0000007C) DMA0 Glitch Filter Register                                */
  __IOM uint32_t  ETH_CTRL0;                    /*!< (@ 0x00000080) ETH Control Register                                       */
  __IM  uint32_t  ETH_STAT0;                    /*!< (@ 0x00000084) ETH Status Register                                        */
  __IM  uint32_t  ETH_PTP_TMST0;                /*!< (@ 0x00000088) ETH Timestamp Register 0                                   */
  __IM  uint32_t  ETH_PTP_TMST1;                /*!< (@ 0x0000008C) ETH Timestamp Register 1                                   */
  __IOM uint32_t  SDC_CTRL0;                    /*!< (@ 0x00000090) SDMMC Control Register                                     */
  __IM  uint32_t  SDC_STAT0;                    /*!< (@ 0x00000094) SDMMC Status Register 0                                    */
  __IM  uint32_t  SDC_STAT1;                    /*!< (@ 0x00000098) SDMMC Status Register 1                                    */
  __IM  uint32_t  RESERVED3;
  __IOM uint32_t  USB_GPIO0;                    /*!< (@ 0x000000A0) USB GPIO Register                                          */
  __IM  uint32_t  USB_STAT0;                    /*!< (@ 0x000000A4) USB Status Register                                        */
  __IOM uint32_t  USB_CTRL1;                    /*!< (@ 0x000000A8) USB Control Register 1                                     */
  __IOM uint32_t  USB_CTRL2;                    /*!< (@ 0x000000AC) USB Control Register 2                                     */
} CLKCTL_PER_MST_Type;                          /*!< Size = 176 (0xb0)                                                         */


/* =========================================================================================================================== */
/* ================                                      CLKCTL_PER_SLV                                       ================ */
/* =========================================================================================================================== */


/**
  * @brief CLKCTL_PER_SLV (CLKCTL_PER_SLV)
  */

typedef struct {                                /*!< (@ 0x4902F000) CLKCTL_PER_SLV Structure                                   */
  __IOM uint32_t  EXPMST0_CTRL;                 /*!< (@ 0x00000000) Clock Control Register                                     */
  __IM  uint32_t  RESERVED;
  __IOM uint32_t  UART_CTRL;                    /*!< (@ 0x00000008) UART Control Register                                      */
  __IOM uint32_t  CANFD_CTRL;                   /*!< (@ 0x0000000C) CANFD Control Register                                     */
  __IOM uint32_t  I2S0_CTRL;                    /*!< (@ 0x00000010) I2Sn Control Register                                      */
  __IOM uint32_t  I2S1_CTRL;                    /*!< (@ 0x00000014) I2Sn Control Register                                      */
  __IOM uint32_t  I2S2_CTRL;                    /*!< (@ 0x00000018) I2Sn Control Register                                      */
  __IOM uint32_t  I2S3_CTRL;                    /*!< (@ 0x0000001C) I2Sn Control Register                                      */
  __IM  uint32_t  RESERVED1;
  __IOM uint32_t  I3C_CTRL;                     /*!< (@ 0x00000024) I3C Control Register                                       */
  __IOM uint32_t  SSI_CTRL;                     /*!< (@ 0x00000028) SPI Control Register                                       */
  __IM  uint32_t  RESERVED2;
  __IOM uint32_t  ADC_CTRL;                     /*!< (@ 0x00000030) ADC Control Register                                       */
  __IOM uint32_t  DAC_CTRL;                     /*!< (@ 0x00000034) DAC Control Register                                       */
  __IOM uint32_t  CMP_CTRL;                     /*!< (@ 0x00000038) CMP Control Register                                       */
  __IM  uint32_t  RESERVED3;
  __IOM uint32_t  FREQ_MON_CTRL0;               /*!< (@ 0x00000040) Frequency Monitor 0 Control Register                       */
  __IM  uint32_t  FREQ_MON_STAT0;               /*!< (@ 0x00000044) Frequency Monitor 0 Status Register                        */
  __IOM uint32_t  FREQ_MON_CTRL1;               /*!< (@ 0x00000048) Frequency Monitor 1 Control Register                       */
  __IM  uint32_t  FREQ_MON_STAT1;               /*!< (@ 0x0000004C) Frequency Monitor 1 Status Register                        */
  __IM  uint32_t  RESERVED4[12];
  __IOM uint32_t  GPIO_CTRL0;                   /*!< (@ 0x00000080) GPIOn Control Register                                     */
  __IOM uint32_t  GPIO_CTRL1;                   /*!< (@ 0x00000084) GPIOn Control Register                                     */
  __IOM uint32_t  GPIO_CTRL2;                   /*!< (@ 0x00000088) GPIOn Control Register                                     */
  __IOM uint32_t  GPIO_CTRL3;                   /*!< (@ 0x0000008C) GPIOn Control Register                                     */
  __IOM uint32_t  GPIO_CTRL4;                   /*!< (@ 0x00000090) GPIOn Control Register                                     */
  __IOM uint32_t  GPIO_CTRL5;                   /*!< (@ 0x00000094) GPIOn Control Register                                     */
  __IOM uint32_t  GPIO_CTRL6;                   /*!< (@ 0x00000098) GPIOn Control Register                                     */
  __IOM uint32_t  GPIO_CTRL7;                   /*!< (@ 0x0000009C) GPIOn Control Register                                     */
  __IOM uint32_t  GPIO_CTRL8;                   /*!< (@ 0x000000A0) GPIOn Control Register                                     */
  __IOM uint32_t  GPIO_CTRL9;                   /*!< (@ 0x000000A4) GPIOn Control Register                                     */
  __IOM uint32_t  GPIO_CTRL10;                  /*!< (@ 0x000000A8) GPIOn Control Register                                     */
  __IOM uint32_t  GPIO_CTRL11;                  /*!< (@ 0x000000AC) GPIOn Control Register                                     */
  __IOM uint32_t  GPIO_CTRL12;                  /*!< (@ 0x000000B0) GPIOn Control Register                                     */
  __IOM uint32_t  GPIO_CTRL13;                  /*!< (@ 0x000000B4) GPIOn Control Register                                     */
  __IOM uint32_t  GPIO_CTRL14;                  /*!< (@ 0x000000B8) GPIOn Control Register                                     */
} CLKCTL_PER_SLV_Type;                          /*!< Size = 188 (0xbc)                                                         */


/* =========================================================================================================================== */
/* ================                                  Peripheral declaration                                   ================ */
/* =========================================================================================================================== */


/** @addtogroup Device_Peripheral_declaration
  * @{
  */


#define CLKCTL_PER_SLV              ((CLKCTL_PER_SLV_Type*)    CLKCTL_PER_SLV_BASE)
#define CLKCTL_PER_MST              ((CLKCTL_PER_MST_Type*)    CLKCTL_PER_MST_BASE)


/** @} */ /* End of group Device_Peripheral_declaration */


#ifdef __cplusplus
}
#endif

#endif /* PERIPHERAL_TYPES_H */
