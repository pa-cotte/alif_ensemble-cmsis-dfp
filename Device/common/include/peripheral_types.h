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

#include <stdint.h>
#include "global_map.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef __IM
#define __IM   volatile const
#endif

#ifndef __OM
#define __OM   volatile
#endif

#ifndef __IOM
#define __IOM  volatile
#endif

/* =========================================================================================================================== */
/* ================                                            CGU                                            ================ */
/* =========================================================================================================================== */

/**
  * @brief CGU (CGU)
  */

/* CGU CLK_ENA field definitions */
#define CLK_ENA_CLK160M    (1U << 20) /* Enable 160M_CLK */
#define CLK_ENA_CLK100M    (1U << 21) /* Enable 100M_CLK */
#define CLK_ENA_CLK20M     (1U << 22) /* Enable USB and 10M_CLK */
#define CLK_ENA_CLK38P4M   (1U << 23) /* Enable HFOSC_CLK */


typedef struct {                                /*!< (@ 0x1A602000) CGU Structure                                              */
  __IOM uint32_t  OSC_CTRL;                     /*!< (@ 0x00000000) Oscillator Control Register                                */
  __IOM uint32_t  PLL_LOCK_CTRL;                /*!< (@ 0x00000004) PLL Lock Control Register                                  */
  __IOM uint32_t  PLL_CLK_SEL;                  /*!< (@ 0x00000008) PLL Clock Select Register                                  */
  __IM  uint32_t  RESERVED;
  __IOM uint32_t  ESCLK_SEL;                    /*!< (@ 0x00000010) Clock Select Register for M55-HP and M55-HE                */
  __IOM uint32_t  CLK_ENA;                      /*!< (@ 0x00000014) Clock Enable Register                                      */
  __IM  uint32_t  RESERVED1[2];
  __IM  uint32_t  CGU_IRQ;                      /*!< (@ 0x00000020) CGU Interrupt Status Register                              */
} CGU_Type;                                     /*!< Size = 36 (0x24)                                                          */


/* =========================================================================================================================== */
/* ================                                      CLKCTL_PER_MST                                       ================ */
/* =========================================================================================================================== */

/**
  * @brief CLKCTL_PER_MST (CLKCTL_PER_MST)
  */

/* CLKCTL_PER_MST CDC200_PIXCLK_CTRL field definitions */
#define CDC200_PIXCLK_CTRL_CKEN             (1U << 0) /* CDC200 Pixel clock enables */
#define CDC200_PIXCLK_CTRL_CLK_SEL          (1U << 4) /* CDC200 Pixel clock select  */
#define CDC200_PIXCLK_CTRL_DIVISOR_Pos      16U       /* CDC200 Pixel clock divisor */
#define CDC200_PIXCLK_CTRL_DIVISOR_Msk      (0x1FFU << CDC200_PIXCLK_CTRL_DIVISOR_Pos)

/* CLKCTL_PER_MST PERIPH_CLK_ENA field definitions */
#define PERIPH_CLK_ENA_CPI_CKEN             (1U << 0)  /* Enable clock supply for CPI */
#define PERIPH_CLK_ENA_DPI_CKEN             (1U << 1)  /* Enable clock supply for DPI controller (CDC) */
#define PERIPH_CLK_ENA_DMA_CKEN             (1U << 4)  /* Enable clock supply for DMA0 */
#define PERIPH_CLK_ENA_GPU_CKEN             (1U << 8)  /* Enable clock supply for GPU2D */
#define PERIPH_CLK_ENA_ETH_CKEN             (1U << 12) /* Enable clock supply for ETH */
#define PERIPH_CLK_ENA_SDC_CKEN             (1U << 16) /* Enable clock supply for SDMMC */
#define PERIPH_CLK_ENA_USB_CKEN             (1U << 20) /* Enable clock supply for USB */
#define PERIPH_CLK_ENA_CSI_CKEN             (1U << 24) /* Enable clock supply for CSI */
#define PERIPH_CLK_ENA_DSI_CKEN             (1U << 28) /* Enable clock supply for DSI */

/* CLKCTL_PER_MST DPHY_PLL_CTRL0 field definitions */
#define DPHY_PLL_CTRL0_FORCE_LOCK           (1U << 0)  /* Force lock to device */
#define DPHY_PLL_CTRL0_SHADOW_CONTROL       (1U << 4)  /* Selection of PLL configuration mechanism */
#define DPHY_PLL_CTRL0_UPDATEPLL            (1U << 8)  /* Control for PLL operation frequency updated */
#define DPHY_PLL_CTRL0_SHADOW_CLEAR         (1U << 12) /* Shadow registers clear */
#define DPHY_PLL_CTRL0_CLKSEL_Pos           20U        /* CLKEXT divider selection */
#define DPHY_PLL_CTRL0_CLKSEL_Msk           (0x3U << DPHY_PLL_CTRL0_CLKSEL_Pos)

/* CLKCTL_PER_MST DPHY_CTRL0 field definitions */
#define DPHY_CTRL0_BIST_ON                  (1U << 0) /* BIST OK */
#define DPHY_CTRL0_BIST_DONE                (1U << 1) /* BIST DONE */
#define DPHY_CTRL0_BIST_OK                  (1U << 2) /* BIST OK */
#define DPHY_CTRL0_TESTPORT_SEL             (1U << 4) /* Test port select */
#define DPHY_CTRL0_TXRXZ                    (1U << 8) /* Selects master or slave configuration for the PHY */
#define DPHY_CTRL0_BASEDIR_Pos              12U       /* Configures the base direction for PHY data lanes */
#define DPHY_CTRL0_BASEDIR_Msk              (0x3U << DPHY_CTRL0_BASEDIR_Pos)
#define DPHY_CTRL0_HSFREQRANGE_Pos          16U       /* Module operating frequency */
#define DPHY_CTRL0_HSFREQRANGE_Msk          (0x7FU << DPHY_CTRL0_HSFREQRANGE_Pos)
#define DPHY_CTRL0_CFGCLKFREQRANGE_Pos      24U       /* Input reference clock frequency */
#define DPHY_CTRL0_CFGCLKFREQRANGE_Msk      (0xFFU << DPHY_CTRL0_CFGCLKFREQRANGE_Pos)

/* CLKCTL_PER_MST DPHY_CTRL1 field definitions */
#define DPHY_CTRL1_FORCERXMODE_Pos          0U /* Controls FORCERXMODE pin of DPHY */
#define DPHY_CTRL1_FORCERXMODE_Msk          (0x3U << DPHY_CTRL1_FORCERXMODE_Pos)

/* CLKCTL_PER_MST MIPI_CKEN field definitions */
#define MIPI_CLKEN_TXDPHY_CKEN              (1U << 0)  /* Enable configure clock for TX D-PHY */
#define MIPI_CLKEN_RXDPHY_CKEN              (1U << 4)  /* Enable configure clock for RX D-PHY */
#define MIPI_CLKEN_PLLREF_CKEN              (1U << 8)  /* Enable reference clock for MIPI D-PHY PLL */
#define MIPI_CLKEN_BYPASS_CKEN              (1U << 12) /* Enable bypass clock for MIPI D-PHY PLL */

/* CLKCTL_PER_MST, M55_Common_CFG DMA_CTRL field definitions */
#define DMA_CTRL_BOOT_MANAGER               (1U << 0)  /* 0: Secure, 1: Non-Secure */
#define DMA_CTRL_SW_RST                     (1U << 16) /* Software reset of DMA0 */


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
/* CLKCTL_PER_SLV EXPMST0_CTRL EXPMST0 Control field definitions */
#define EXPMST0_CTRL_IPCLK_FORCE            (1U << 31)                      /* Force peripherals functional clocks ON  */
#define EXPMST0_CTRL_PCLK_FORCE             (1U << 30)                      /* Force APB interface (PCLK) clocks ON    */
#define EXPMST0_CTRL_PDM_SEL_76M8_CLK       (0U << 9)                       /* PDM Clock Source Select 76.8Mhz         */
#define EXPMST0_CTRL_PDM_SEL_EXT_CLK        (1U << 9)                       /* PDM Clock Source Select External clock  */
#define EXPMST0_CTRL_PDM_CKEN               (1U << 8)                       /* PDM Clock Enable                        */
#define EXPMST0_CTRL_BKRAM_CKEN             (1U << 4)                       /* Backup SRAM Clock Enable                */


/* CLKCTL_PER_SLV I2Sn_CTRL I2S Control field definitions */
#define I2S_CTRL_SCLK_AON                   (1U << 20)                      /* SCLK Always On                          */
#define I2S_CTRL_DIV_BYPASS                 (1U << 17)                      /* Bypass Clock Divider                    */
#define I2S_CTRL_CLK_SEL_Msk                (1U << 16)                      /* Clock Source Selection Mask             */
#define I2S_CTRL_CLK_SEL_76M8_CLK           (0U << 16)                      /* Enable 76.8MHz Crystal Oscillator Clock */
#define I2S_CTRL_CLK_SEL_EXT_CLK            (1U << 16)                      /* Enable External Audio clock input       */
#define I2S_CTRL_CKEN                       (1U << 12)                      /* Enable I2S controller clock             */
#define I2S_CTRL_CKDIV_Pos                  (0)                             /* Clock divider start position            */
#define I2S_CTRL_CKDIV_Msk                  (0x3FF << I2S_CTRL_CKDIV_Pos)   /* Clock divider start mask                */

/* CLKCTL_PER_SLV SSI_CTRL field definitions */
#define SSI_CTRL_SS_IN_SEL_0                (1U  <<  0U) /* SPI0 Slave select mode */
#define SSI_CTRL_SS_IN_SEL_1                (1U  <<  1U) /* SPI1 Slave select mode */
#define SSI_CTRL_SS_IN_SEL_2                (1U  <<  2U) /* SPI2 Slave select mode */
#define SSI_CTRL_SS_IN_SEL_3                (1U  <<  3U) /* SPI3 Slave select mode */
#define SSI_CTRL_SS_IN_VAL_0                (1U  <<  8U) /* SPI0 Slave select value */
#define SSI_CTRL_SS_IN_VAL_1                (1U  <<  9U) /* SPI1 Slave select value */
#define SSI_CTRL_SS_IN_VAL_2                (1U  << 10U) /* SPI2 Slave select value */
#define SSI_CTRL_SS_IN_VAL_3                (1U  << 11U) /* SPI3 Slave select value */

/* CLKCTL_PER_SLV ADC_CTRL field definitions */
#define ADC_CTRL_ADC0_CKEN                  (1U  << 0U)  /* ADC0 clock enable */
#define ADC_CTRL_ADC1_CKEN                  (1U  << 4U)  /* ADC1 clock enable */
#define ADC_CTRL_ADC2_CKEN                  (1U  << 8U)  /* ADC2 clock enable */

/* CLKCTL_PER_SLV GPIO_CTRLn field definitions */
#define GPIO_CTRL_DB_CKEN                   (1U  << 12U) /* GPIO Debounce clock enable */

/* CLKCTL_PER_SLV DAC field definitions */
#define DAC_CTRL_DAC0_CKEN                  (1U  <<  0U) /* DAC0 clock enable */
#define DAC_CTRL_DAC1_CKEN                  (1U  <<  4U) /* DAC1 clock enable */

/* CLKCTL_PER_SLV CMP field definitions */
#define CMP_CTRL_CMP0_CLKEN                (1U << 0U)  /* Enable CMP0 clock */
#define CMP_CTRL_CMP1_CLKEN                (1U << 4U)  /* Enable CMP1 clock */
#define CMP_CTRL_CMP2_CLKEN                (1U << 8U)  /* Enable CMP2 clock */
#define CMP_CTRL_CMP3_CLKEN                (1U << 12U) /* Enable CMP3 clock */

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
/* ================                                           VBAT                                            ================ */
/* =========================================================================================================================== */

/**
  * @brief VBAT (VBAT)
  */

/* VBAT TIMER_CLKSEL field definitions */
#define TIMER_CLKSEL_Pos                   (0U)   /* TIMER_CLKSEL bit position */
#define TIMER_CLKSEL_Msk                   (3U << TIMER_CLKSEL_Pos)

/*VBAT PWR_CTRL field definitions*/
#define PWR_CTRL_TX_DPHY_PWR_MASK          (1U << 0)    /* Mask off the power supply for MIPI TX DPHY */
#define PWR_CTRL_TX_DPHY_ISO               (1U << 1)    /* Enable isolation for MIPI TX DPHY */
#define PWR_CTRL_RX_DPHY_PWR_MASK          (1U << 4)    /* Mask off the power supply for MIPI RX DPHY */
#define PWR_CTRL_RX_DPHY_ISO               (1U << 5)    /* Enable isolation for MIPI RX DPHY */
#define PWR_CTRL_DPHY_PLL_PWR_MASK         (1U << 8)    /* Mask off the power supply for MIPI PLL */
#define PWR_CTRL_DPHY_PLL_ISO              (1U << 9)    /* Enable isolation for MIPI PLL */
#define PWR_CTRL_DPHY_VPH_1P8_PWR_BYP_EN   (1U << 12)   /* dphy vph 1p8 power bypass enable */
#define PWR_CTRL_DPHY_VPH_1P8_PWR_BYP_VAL  (1U << 13)   /* dphy vph 1p8 power bypass val */
#define PWR_CTRL_UPHY_PWR_MASK             (1U << 16)   /* Mask off the power supply for USB PHY */
#define PWR_CTRL_UPHY_ISO                  (1U << 17)   /* Enable isolation for USB PHY */

/* VBAT RTC_CLK_EN field definitions */
#define RTC_CLK_ENABLE                     (1U << 0U)  /* Enable RTC clock */

typedef struct {                                /*!< (@ 0x1A609000) VBAT Structure                                             */
  __IOM uint32_t  GPIO_CTRL;                    /*!< (@ 0x00000000) GPIO 1.8 V / 3.3 V Power Control Register                  */
  __IOM uint32_t  TIMER_CLKSEL;                 /*!< (@ 0x00000004) LPTIMER Clock Select Register                              */
  __IOM uint32_t  PWR_CTRL;                     /*!< (@ 0x00000008) Power Control Register                                     */
  __IOM uint32_t  RET_CTRL;                     /*!< (@ 0x0000000C) Memory Retention Control Register                          */
  __IOM uint32_t  RTC_CLK_EN;                   /*!< (@ 0x00000010) LPRTC Clock Enable Register                                */
} VBAT_Type;                                    /*!< Size = 20 (0x14)                                                          */


/* =========================================================================================================================== */
/* ================                                         M55HE_CFG                                         ================ */
/* =========================================================================================================================== */


/**
  * @brief M55HE_CFG (M55HE_CFG)
  */

typedef struct {                                /*!< (@ 0x43007000) M55HE_CFG Structure                                        */
  __IOM uint32_t  HE_DMA_CTRL;                  /*!< (@ 0x00000000) DMA2 Boot Control Register                                 */
  __IOM uint32_t  HE_DMA_IRQ;                   /*!< (@ 0x00000004) DMA2 Boot IRQ Non-Secure Register                          */
  __IOM uint32_t  HE_DMA_PERIPH;                /*!< (@ 0x00000008) DMA2 Boot Peripheral Non-Secure Register                   */
  __IOM uint32_t  HE_DMA_SEL;                   /*!< (@ 0x0000000C) DMA2 Select Register                                       */
  __IOM uint32_t  HE_CLK_ENA;                   /*!< (@ 0x00000010) Peripheral Clock Enable Register                           */
  __IOM uint32_t  HE_I2S_CTRL;                  /*!< (@ 0x00000014) LPI2S Control Register                                     */
  __IM  uint32_t  RESERVED[2];
  __IOM uint32_t  HE_CAMERA_PIXCLK;             /*!< (@ 0x00000020) LPCPI Pixel Clock Control Register                         */
} M55HE_CFG_Type;

/**
  * @brief M55HP_CFG (M55HP_CFG)
  */

typedef struct {                                /*!< (@ 0x400F0000) M55HP_CFG Structure                                        */
  __IOM uint32_t  HP_DMA_CTRL;                  /*!< (@ 0x00000000) DMA1 Boot Control Register                                 */
  __IOM uint32_t  HP_DMA_IRQ;                   /*!< (@ 0x00000004) DMA1 Boot IRQ Non-Secure Register                          */
  __IOM uint32_t  HP_DMA_PERIPH;                /*!< (@ 0x00000008) DMA1 Boot Peripheral Non-Secure Register                   */
  __IOM uint32_t  HP_DMA_SEL;                   /*!< (@ 0x0000000C) DMA1 Select Register                                       */
  __IOM uint32_t  HP_CLK_ENA;                   /*!< (@ 0x00000010) Peripheral Clock Enable Register                           */
} M55HP_CFG_Type;                               /*!< Size = 20 (0x14)                                                          */

/**
  * @brief M55_CFG_Common_Type (M55_CFG_Common_Type)
  * M55_HE_CFG and M55_HP_CFG common registers structure.
  */

/* M55_CFG_Common_Type CLK_ENA field definitions */
#define CLK_ENA_NPU_CKEN                    (1U << 0)  /* Enable clock supply for NPU */
#define CLK_ENA_DMA_CKEN                    (1U << 4)  /* Enable clock supply for DMA */

typedef struct {                                /*!< M55_HE_CFG and M55_HP_CFG common registers structure                      */
  __IOM uint32_t  DMA_CTRL;                     /*!< (@ 0x00000000) DMA Boot Control Register                                  */
  __IOM uint32_t  DMA_IRQ;                      /*!< (@ 0x00000004) DMA Boot IRQ Non-Secure Register                           */
  __IOM uint32_t  DMA_PERIPH;                   /*!< (@ 0x00000008) DMA Boot Peripheral Non-Secure Register                    */
  __IOM uint32_t  DMA_SEL;                      /*!< (@ 0x0000000C) DMA Select Register                                        */
  __IOM uint32_t  CLK_ENA;                      /*!< (@ 0x00000010) Local Peripheral Clock Enable Register                     */
} M55_CFG_Common_Type;

/**
  * @brief ANA (ANA)
  */

typedef struct {                                /*!< (@ 0x1A60A000) ANA Structure                                              */
  __IOM uint32_t  MISC_CTRL;                    /*!< (@ 0x00000000) VBAT Misc Control Register                                 */
  __IOM uint32_t  PWR_CTRL;                     /*!< (@ 0x00000004) VBAT Power ControlNOTE: Internal register!                 */
  __IOM uint32_t  WKUP_CTRL;                    /*!< (@ 0x00000008) VBAT Wake-up Source Control Register                       */
  __IOM uint32_t  TIMER_PWR;                    /*!< (@ 0x0000000C) PWR ON/OFF Timer ValueNOTE: Internal register!             */
  __IOM uint32_t  BISR_CTRL;                    /*!< (@ 0x00000010) VBAT BISR Control RegisterNOTE: Internal register!         */
  __IOM uint32_t  TIMER_DCDC;                   /*!< (@ 0x00000014) DCDC ON/OFF Timer ValueNOTE: Internal register!            */
  __IOM uint32_t  RET_CTRL;                     /*!< (@ 0x00000018) Memory Retention ControlNOTE: Internal register!           */
  __IM  uint32_t  FSM_STAT;                     /*!< (@ 0x0000001C) VBAT FSM Status RegisterNOTE: Internal register!           */
  __IOM uint32_t  CM55_HE_NS_VTOR;              /*!< (@ 0x00000020) 25-bit VTOR RegisterNOTE: Internal register!               */
  __IOM uint32_t  CM55_HE_SE_VTOR;              /*!< (@ 0x00000024) 25-bit VTOR RegisterNOTE: Internal register!               */
  __IM  uint32_t  RESERVED[2];
  __IOM uint32_t  DCDC_REG1;                    /*!< (@ 0x00000030) DC/DC Control Register 1                                   */
  __IOM uint32_t  DCDC_REG2;                    /*!< (@ 0x00000034) DC/DC Control Register 2                                   */
  __IOM uint32_t  VBAT_ANA_REG1;                /*!< (@ 0x00000038) VBAT Analog Control Register 1                             */
  __IOM uint32_t  VBAT_ANA_REG2;                /*!< (@ 0x0000003C) VBAT Analog Control Register 2                             */
  __IOM uint32_t  VBAT_ANA_REG3;                /*!< (@ 0x00000040) VBAT Analog Control Register 3                             */
} ANA_Type;

/* =========================================================================================================================== */
/* ================                                           CMP0                                            ================ */
/* =========================================================================================================================== */


/**
  * @brief CMP0 (CMP0)
  */

typedef struct {                                /*!< (@ 0x49023000) CMP0 Structure                                             */
  __IOM uint32_t  CMP_COMP_REG1;                /*!< (@ 0x00000000) Comparator Register 1                                      */
  __IOM uint32_t  CMP_COMP_REG2;                /*!< (@ 0x00000004) Comparator Register 2                                      */
  __IOM uint32_t  CMP_POLARITY_CTRL;            /*!< (@ 0x00000008) CMP Polarity Control Register                              */
  __IOM uint32_t  CMP_WINDOW_CTRL;              /*!< (@ 0x0000000C) CMP Window Control Register                                */
  __IOM uint32_t  CMP_FILTER_CTRL;              /*!< (@ 0x00000010) CMP Filter Control Register                                */
  __IOM uint32_t  CMP_PRESCALER_CTRL;           /*!< (@ 0x00000014) CMP Prescaler Control Register                             */
  __IM  uint32_t  RESERVED[2];
  __IOM uint32_t  CMP_INTERRUPT_STATUS;         /*!< (@ 0x00000020) CMP Interrupt Status and Clear Register                    */
  __IOM uint32_t  CMP_INTERRUPT_MASK;           /*!< (@ 0x00000024) CMP Interrupt Mask Register                                */
} CMP01_Type;                                    /*!< Size = 40 (0x28)                                                          */

/* =========================================================================================================================== */
/* ================                                      AON Register                                         ================ */
/* =========================================================================================================================== */
/**
  * @brief AON (AON)
  */
typedef struct {                                /*!< (@ 0x1A604000) AON Structure                               */
  __IOM uint32_t  RTSS_HP_CTRL;                 /*!< (@ 0x00000000) M55-HP Control Register                     */
  __IOM uint32_t  RTSS_HP_RESET;                /*!< (@ 0x00000004) M55-HP Reset Status Register                */
  __IM  uint32_t  RESERVED[2];
  __IOM uint32_t  RTSS_HE_CTRL;                 /*!< (@ 0x00000010) M55-HE Control Register                     */
  __IOM uint32_t  RTSS_HE_RESET;                /*!< (@ 0x00000014) M55-HE Reset Status Register                */
  __IM  uint32_t  RESERVED1;
  __IOM uint32_t  RTSS_HE_LPUART_CKEN;          /*!< (@ 0x0000001C) LPUART Clock Enable Register                */
  __IOM uint32_t  SYSTOP_CLK_DIV;               /*!< (@ 0x00000020) System Bus Clock Divider Control Register   */
  __IM  uint32_t  RESERVED2[7];
  __IOM uint32_t  PMU_PERIPH;                   /*!< (@ 0x00000040) ADC Control Register                        */
} AON_Type;                                     /*!< Size = 68 (0x44)                                           */

/* =========================================================================================================================== */
/* ================                                  Peripheral declaration                                   ================ */
/* =========================================================================================================================== */


/** @addtogroup Device_Peripheral_declaration
  * @{
  */

#define CGU                         ((CGU_Type*)               CGU_BASE)
#define CLKCTL_PER_SLV              ((CLKCTL_PER_SLV_Type*)    CLKCTL_PER_SLV_BASE)
#define CLKCTL_PER_MST              ((CLKCTL_PER_MST_Type*)    CLKCTL_PER_MST_BASE)
#define VBAT                        ((VBAT_Type*)              VBAT_BASE)
#define AON                         ((AON_Type*)               AON_BASE)
#define ANA_REG                     ((ANA_Type *)              ANA_BASE)
#define CMP_REG                     ((CMP01_Type *)            CMP0_BASE)
#define M55HE_CFG                   ((M55HE_CFG_Type*)         M55HE_CFG_BASE)
#if (M55_HP)
#define M55HP_CFG                   ((M55HP_CFG_Type*)         M55HP_CFG_BASE)
#define M55LOCAL_CFG                ((M55_CFG_Common_Type*)    M55HP_CFG_BASE)
#endif
#if (M55_HE)
#define M55LOCAL_CFG                ((M55_CFG_Common_Type*)    M55HE_CFG_BASE)
#endif


/** @} */ /* End of group Device_Peripheral_declaration */


#ifdef __cplusplus
}
#endif

#endif /* PERIPHERAL_TYPES_H */
