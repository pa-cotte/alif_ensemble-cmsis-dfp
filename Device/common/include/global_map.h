/* Copyright (C) 2022 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

#ifndef GLOBAL_MAP_H
#define GLOBAL_MAP_H


/* =========================================================================================================================== */
/* ================                          Device Specific Peripheral Address Map                           ================ */
/* =========================================================================================================================== */


/** @addtogroup Device_Peripheral_peripheralAddr
  * @{
  */

#define CLKCTL_SYS_BASE             0x1A010000UL
#define WDT_AP_CTRL_BASE            0x1A300000UL
#define WDT_AP_REFRESH_BASE         0x1A310000UL
#define WDT_AP_S_CTRL_BASE          0x1A320000UL
#define WDT_AP_S_REFRESH_BASE       0x1A330000UL
#define IRQRTR_BASE                 0x1A500000UL
#define CGU_BASE                    0x1A602000UL
#define PINMUX_BASE                 0x1A603000UL
#define AON_BASE                    0x1A604000UL
#define AONEXP_SECENC__internal_BASE 0x1A605000UL
#define AONEXP_MODEM__internal_BASE 0x1A606000UL
#define SECENC_EWIC__internal_BASE  0x1A607000UL
#define VBAT_BASE                   0x1A609000UL
#define ANA_BASE                    0x1A60A000UL
#define VBATMDM__internal_BASE      0x1A60B000UL
#define STOP_MODE_BASE              0x1A60F000UL
#define FCTLR_BASE                  0x1A800000UL
#define FC1_BASE                    0x1A810000UL
#define FC2_BASE                    0x1A820000UL
#define FC3_BASE                    0x1A830000UL
#define FC4_BASE                    0x1A840000UL
#define FC5_BASE                    0x1A850000UL
#define FC6_BASE                    0x1A860000UL
#define FC7_BASE                    0x1A870000UL
#define FC8_BASE                    0x1A880000UL
#define FC9_BASE                    0x1A890000UL
#define FC10_BASE                   0x1A8A0000UL
#define FC11_BASE                   0x1A8B0000UL
#define FC12_BASE                   0x1A8C0000UL
#define FC13_BASE                   0x1A8D0000UL
#define FC14_BASE                   0x1A8E0000UL
#define MHU_A32_M55HP_0_RX_BASE     0x40000000UL
#define MHU_M55HP_A32_0_TX_BASE     0x40010000UL
#define MHU_A32_M55HP_1_RX_BASE     0x40020000UL
#define MHU_M55HP_A32_1_TX_BASE     0x40030000UL
#define MHU_SECPU_M55HP_0_RX_BASE   0x40040000UL
#define MHU_M55HP_SECPU_0_TX_BASE   0x40050000UL
#define MHU_SECPU_M55HP_1_RX_BASE   0x40060000UL
#define MHU_M55HP_SECPU_1_TX_BASE   0x40070000UL
#define MHU_M55HE_M55HP_0_RX_BASE   0x40080000UL
#define MHU_M55HP_M55HE_0_TX_BASE   0x40090000UL
#define MHU_M55HE_M55HP_1_RX_BASE   0x400A0000UL
#define MHU_M55HP_M55HE_1_TX_BASE   0x400B0000UL
#define DMA1_SEC_BASE               0x400C0000UL
#define DMA1_NS_BASE                0x400E0000UL
#define NPU_HP_BASE                 0x400E1000UL
#define EVTRTR1_BASE                0x400E2000UL
#define M55HP_CFG_BASE              0x400F0000UL
#define WDT_HP_CTRL_BASE            0x40100000UL
#define WDT_HP_REFRESH_BASE         0x40101000UL
#define LPRTC_BASE                  0x42000000UL
#define LPTIMER_BASE                0x42001000UL
#define LPGPIO_BASE                 0x42002000UL
#define LPGPIO_CTRL_BASE            0x42007000UL
#define LPSPI_BASE                  0x43000000UL
#define LPI2S_BASE                  0x43001000UL
#define LPPDM_BASE                  0x43002000UL
#define LPCPI_BASE                  0x43003000UL
#define M55HE_CFG_BASE              0x43007000UL
#define LPUART_BASE                 0x43008000UL
#define LPI2C_BASE                  0x43009000UL
#define UTIMER_BASE                 0x48000000UL
#define ETH_BASE                    0x48100000UL
#define SDMMC_BASE                  0x48102000UL
#define SPI0_BASE                   0x48103000UL
#define SPI1_BASE                   0x48104000UL
#define SPI2_BASE                   0x48105000UL
#define SPI3_BASE                   0x48106000UL
#define CRC0_BASE                   0x48107000UL
#define CRC1_BASE                   0x48108000UL
#define USB_BASE                    0x48200000UL
#define GPIO0_BASE                  0x49000000UL
#define GPIO1_BASE                  0x49001000UL
#define GPIO2_BASE                  0x49002000UL
#define GPIO3_BASE                  0x49003000UL
#define GPIO4_BASE                  0x49004000UL
#define GPIO5_BASE                  0x49005000UL
#define GPIO6_BASE                  0x49006000UL
#define GPIO7_BASE                  0x49007000UL
#define GPIO8_BASE                  0x49008000UL
#define GPIO9_BASE                  0x49009000UL
#define GPIO10_BASE                 0x4900A000UL
#define GPIO11_BASE                 0x4900B000UL
#define GPIO12_BASE                 0x4900C000UL
#define GPIO13_BASE                 0x4900D000UL
#define GPIO14_BASE                 0x4900E000UL
#define I2C0_BASE                   0x49010000UL
#define I2C1_BASE                   0x49011000UL
#define I2C2_BASE                   0x49012000UL
#define I2C3_BASE                   0x49013000UL
#define I2S0_BASE                   0x49014000UL
#define I2S1_BASE                   0x49015000UL
#define I2S2_BASE                   0x49016000UL
#define I2S3_BASE                   0x49017000UL
#define UART0_BASE                  0x49018000UL
#define UART1_BASE                  0x49019000UL
#define UART2_BASE                  0x4901A000UL
#define UART3_BASE                  0x4901B000UL
#define UART4_BASE                  0x4901C000UL
#define UART5_BASE                  0x4901D000UL
#define UART6_BASE                  0x4901E000UL
#define UART7_BASE                  0x4901F000UL
#define ADC120_BASE                 0x49020000UL
#define ADC121_BASE                 0x49021000UL
#define ADC122_BASE                 0x49022000UL
#define CMP0_BASE                   0x49023000UL
#define CMP1_BASE                   0x49024000UL
#define CMP2_BASE                   0x49025000UL
#define CMP3_BASE                   0x49026000UL
#define ADC24_BASE                  0x49027000UL
#define DAC120_BASE                 0x49028000UL
#define DAC121_BASE                 0x49029000UL
#define PDM_BASE                    0x4902D000UL
#define HWSEM_BASE                  0x4902E000UL
#define CLKCTL_PER_SLV_BASE         0x4902F000UL
#define CPI_BASE                    0x49030000UL
#define CDC_BASE                    0x49031000UL
#define DSI_BASE                    0x49032000UL
#define CSI_BASE                    0x49033000UL
#define I3C_BASE                    0x49034000UL
#define EVTRTR0_BASE                0x49035000UL
#define CANFD_BASE                  0x49036000UL
#define INTGEN_0__internal_BASE     0x49038000UL
#define INTGEN_1__internal_BASE     0x49039000UL
#define INTGEN_2__internal_BASE     0x4903A000UL
#define INTGEN_3__internal_BASE     0x4903B000UL
#define INTGEN_4__internal_BASE     0x4903C000UL
#define INTGEN_5__internal_BASE     0x4903D000UL
#define CLKCTL_PER_MST_BASE         0x4903F000UL
#define GPU2D_BASE                  0x49040000UL
#define DMA0_SEC_BASE               0x49080000UL
#define DMA0_NS_BASE                0x490A0000UL
#define OSPI0_BASE                  0x83000000UL
#define AES0_BASE                   0x83001000UL
#define OSPI1_BASE                  0x83002000UL
#define AES1_BASE                   0x83003000UL
#define M55HP_NVIC_S_BASE           0xE000E100UL
#define M55HP_NVIC_NS_BASE          0xE002E100UL

/** @} */ /* End of group Device_Peripheral_peripheralAddr */


/* =========================================================================================================================== */
/* ================                                  Peripheral declaration                                   ================ */
/* =========================================================================================================================== */


/** @addtogroup Device_Peripheral_declaration
  * @{
  */

#define CLKCTL_SYS                  ((CLKCTL_SYS_Type*)        CLKCTL_SYS_BASE)
#define WDT_AP_CTRL                 ((WDT_AP_CTRL_Type*)       WDT_AP_CTRL_BASE)
#define WDT_AP_REFRESH              ((WDT_AP_CTRL_Type*)       WDT_AP_REFRESH_BASE)
#define WDT_AP_S_CTRL               ((WDT_AP_CTRL_Type*)       WDT_AP_S_CTRL_BASE)
#define WDT_AP_S_REFRESH            ((WDT_AP_CTRL_Type*)       WDT_AP_S_REFRESH_BASE)
#define IRQRTR                      ((IRQRTR_Type*)            IRQRTR_BASE)
#define CGU                         ((CGU_Type*)               CGU_BASE)
#define PINMUX                      ((PINMUX_Type*)            PINMUX_BASE)
#define AON                         ((AON_Type*)               AON_BASE)
#define AONEXP_SECENC__internal     ((AONEXP_SECENC__internal_Type*)  AONEXP_SECENC__internal_BASE)
#define AONEXP_MODEM__internal      ((AONEXP_MODEM__internal_Type*)  AONEXP_MODEM__internal_BASE)
#define SECENC_EWIC__internal       ((SECENC_EWIC__internal_Type*)  SECENC_EWIC__internal_BASE)
#define VBAT                        ((VBAT_Type*)              VBAT_BASE)
#define ANA                         ((ANA_Type*)               ANA_BASE)
#define VBATMDM__internal           ((VBATMDM__internal_Type*)  VBATMDM__internal_BASE)
#define STOP_MODE                   ((STOP_MODE_Type*)         STOP_MODE_BASE)
#define FCTLR                       ((FCTLR_Type*)             FCTLR_BASE)
#define FC1                         ((FCTLR_Type*)             FC1_BASE)
#define FC2                         ((FCTLR_Type*)             FC2_BASE)
#define FC3                         ((FCTLR_Type*)             FC3_BASE)
#define FC4                         ((FCTLR_Type*)             FC4_BASE)
#define FC5                         ((FCTLR_Type*)             FC5_BASE)
#define FC6                         ((FCTLR_Type*)             FC6_BASE)
#define FC7                         ((FCTLR_Type*)             FC7_BASE)
#define FC8                         ((FCTLR_Type*)             FC8_BASE)
#define FC9                         ((FCTLR_Type*)             FC9_BASE)
#define FC10                        ((FCTLR_Type*)             FC10_BASE)
#define FC11                        ((FCTLR_Type*)             FC11_BASE)
#define FC12                        ((FCTLR_Type*)             FC12_BASE)
#define FC13                        ((FCTLR_Type*)             FC13_BASE)
#define FC14                        ((FCTLR_Type*)             FC14_BASE)
#define MHU_A32_M55HP_0_RX          ((MHU_A32_M55HP_0_RX_Type*)  MHU_A32_M55HP_0_RX_BASE)
#define MHU_M55HP_A32_0_TX          ((MHU_A32_M55HP_0_RX_Type*)  MHU_M55HP_A32_0_TX_BASE)
#define MHU_A32_M55HP_1_RX          ((MHU_A32_M55HP_0_RX_Type*)  MHU_A32_M55HP_1_RX_BASE)
#define MHU_M55HP_A32_1_TX          ((MHU_A32_M55HP_0_RX_Type*)  MHU_M55HP_A32_1_TX_BASE)
#define MHU_SECPU_M55HP_0_RX        ((MHU_A32_M55HP_0_RX_Type*)  MHU_SECPU_M55HP_0_RX_BASE)
#define MHU_M55HP_SECPU_0_TX        ((MHU_A32_M55HP_0_RX_Type*)  MHU_M55HP_SECPU_0_TX_BASE)
#define MHU_SECPU_M55HP_1_RX        ((MHU_A32_M55HP_0_RX_Type*)  MHU_SECPU_M55HP_1_RX_BASE)
#define MHU_M55HP_SECPU_1_TX        ((MHU_A32_M55HP_0_RX_Type*)  MHU_M55HP_SECPU_1_TX_BASE)
#define MHU_M55HE_M55HP_0_RX        ((MHU_A32_M55HP_0_RX_Type*)  MHU_M55HE_M55HP_0_RX_BASE)
#define MHU_M55HP_M55HE_0_TX        ((MHU_A32_M55HP_0_RX_Type*)  MHU_M55HP_M55HE_0_TX_BASE)
#define MHU_M55HE_M55HP_1_RX        ((MHU_A32_M55HP_0_RX_Type*)  MHU_M55HE_M55HP_1_RX_BASE)
#define MHU_M55HP_M55HE_1_TX        ((MHU_A32_M55HP_0_RX_Type*)  MHU_M55HP_M55HE_1_TX_BASE)
#define DMA1_SEC                    ((DMA1_SEC_Type*)          DMA1_SEC_BASE)
#define DMA1_NS                     ((DMA1_SEC_Type*)          DMA1_NS_BASE)
#define NPU_HP                      ((NPU_HP_Type*)            NPU_HP_BASE)
#define EVTRTR1                     ((EVTRTR1_Type*)           EVTRTR1_BASE)
#define M55HP_CFG                   ((M55HP_CFG_Type*)         M55HP_CFG_BASE)
#define WDT_HP_CTRL                 ((WDT_HP_CTRL_Type*)       WDT_HP_CTRL_BASE)
#define WDT_HP_REFRESH              ((WDT_HP_CTRL_Type*)       WDT_HP_REFRESH_BASE)
#define LPRTC                       ((LPRTC_Type*)             LPRTC_BASE)
#define LPTIMER                     ((LPTIMER_Type*)           LPTIMER_BASE)
#define LPGPIO                      ((LPGPIO_Type*)            LPGPIO_BASE)
#define LPGPIO_CTRL                 ((LPGPIO_CTRL_Type*)       LPGPIO_CTRL_BASE)
#define LPSPI                       ((LPSPI_Type*)             LPSPI_BASE)
#define LPI2S                       ((LPI2S_Type*)             LPI2S_BASE)
#define LPPDM                       ((LPPDM_Type*)             LPPDM_BASE)
#define LPCPI                       ((LPCPI_Type*)             LPCPI_BASE)
#define M55HE_CFG                   ((M55HE_CFG_Type*)         M55HE_CFG_BASE)
#define LPUART                      ((LPUART_Type*)            LPUART_BASE)
#define LPI2C                       ((LPI2C_Type*)             LPI2C_BASE)
#define UTIMER                      ((UTIMER_Type*)            UTIMER_BASE)
#define ETH                         ((ETH_Type*)               ETH_BASE)
#define SDMMC                       ((SDMMC_Type*)             SDMMC_BASE)
#define SPI0                        ((LPSPI_Type*)             SPI0_BASE)
#define SPI1                        ((LPSPI_Type*)             SPI1_BASE)
#define SPI2                        ((LPSPI_Type*)             SPI2_BASE)
#define SPI3                        ((LPSPI_Type*)             SPI3_BASE)
#define CRC0                        ((CRC0_Type*)              CRC0_BASE)
#define CRC1                        ((CRC0_Type*)              CRC1_BASE)
#define USB                         ((USB_Type*)               USB_BASE)
#define GPIO0                       ((LPGPIO_Type*)            GPIO0_BASE)
#define GPIO1                       ((LPGPIO_Type*)            GPIO1_BASE)
#define GPIO2                       ((LPGPIO_Type*)            GPIO2_BASE)
#define GPIO3                       ((LPGPIO_Type*)            GPIO3_BASE)
#define GPIO4                       ((LPGPIO_Type*)            GPIO4_BASE)
#define GPIO5                       ((LPGPIO_Type*)            GPIO5_BASE)
#define GPIO6                       ((LPGPIO_Type*)            GPIO6_BASE)
#define GPIO7                       ((LPGPIO_Type*)            GPIO7_BASE)
#define GPIO8                       ((LPGPIO_Type*)            GPIO8_BASE)
#define GPIO9                       ((LPGPIO_Type*)            GPIO9_BASE)
#define GPIO10                      ((LPGPIO_Type*)            GPIO10_BASE)
#define GPIO11                      ((LPGPIO_Type*)            GPIO11_BASE)
#define GPIO12                      ((LPGPIO_Type*)            GPIO12_BASE)
#define GPIO13                      ((LPGPIO_Type*)            GPIO13_BASE)
#define GPIO14                      ((LPGPIO_Type*)            GPIO14_BASE)
#define I2C0                        ((LPI2C_Type*)             I2C0_BASE)
#define I2C1                        ((LPI2C_Type*)             I2C1_BASE)
#define I2C2                        ((LPI2C_Type*)             I2C2_BASE)
#define I2C3                        ((LPI2C_Type*)             I2C3_BASE)
#define I2S0                        ((LPI2S_Type*)             I2S0_BASE)
#define I2S1                        ((LPI2S_Type*)             I2S1_BASE)
#define I2S2                        ((LPI2S_Type*)             I2S2_BASE)
#define I2S3                        ((LPI2S_Type*)             I2S3_BASE)
#define UART0                       ((LPUART_Type*)            UART0_BASE)
#define UART1                       ((LPUART_Type*)            UART1_BASE)
#define UART2                       ((LPUART_Type*)            UART2_BASE)
#define UART3                       ((LPUART_Type*)            UART3_BASE)
#define UART4                       ((LPUART_Type*)            UART4_BASE)
#define UART5                       ((LPUART_Type*)            UART5_BASE)
#define UART6                       ((LPUART_Type*)            UART6_BASE)
#define UART7                       ((LPUART_Type*)            UART7_BASE)
#define ADC120                      ((ADC120_Type*)            ADC120_BASE)
#define ADC121                      ((ADC120_Type*)            ADC121_BASE)
#define ADC122                      ((ADC120_Type*)            ADC122_BASE)
#define CMP0                        ((CMP0_Type*)              CMP0_BASE)
#define CMP1                        ((CMP0_Type*)              CMP1_BASE)
#define CMP2                        ((CMP0_Type*)              CMP2_BASE)
#define CMP3                        ((CMP0_Type*)              CMP3_BASE)
#define ADC24                       ((ADC120_Type*)            ADC24_BASE)
#define DAC120                      ((DAC120_Type*)            DAC120_BASE)
#define DAC121                      ((DAC120_Type*)            DAC121_BASE)
#define PDM                         ((LPPDM_Type*)             PDM_BASE)
#define HWSEM                       ((HWSEM_Type*)             HWSEM_BASE)
#define CLKCTL_PER_SLV              ((CLKCTL_PER_SLV_Type*)    CLKCTL_PER_SLV_BASE)
#define CPI                         ((LPCPI_Type*)             CPI_BASE)
#define CDC                         ((CDC_Type*)               CDC_BASE)
#define DSI                         ((DSI_Type*)               DSI_BASE)
#define CSI                         ((CSI_Type*)               CSI_BASE)
#define I3C                         ((I3C_Type*)               I3C_BASE)
#define EVTRTR0                     ((EVTRTR1_Type*)           EVTRTR0_BASE)
#define CANFD                       ((CANFD_Type*)             CANFD_BASE)
#define INTGEN_0__internal          ((INTGEN_0__internal_Type*)  INTGEN_0__internal_BASE)
#define INTGEN_1__internal          ((INTGEN_1__internal_Type*)  INTGEN_1__internal_BASE)
#define INTGEN_2__internal          ((INTGEN_2__internal_Type*)  INTGEN_2__internal_BASE)
#define INTGEN_3__internal          ((INTGEN_3__internal_Type*)  INTGEN_3__internal_BASE)
#define INTGEN_4__internal          ((INTGEN_4__internal_Type*)  INTGEN_4__internal_BASE)
#define INTGEN_5__internal          ((INTGEN_5__internal_Type*)  INTGEN_5__internal_BASE)
#define CLKCTL_PER_MST              ((CLKCTL_PER_MST_Type*)    CLKCTL_PER_MST_BASE)
#define GPU2D                       ((GPU2D_Type*)             GPU2D_BASE)
#define DMA0_SEC                    ((DMA1_SEC_Type*)          DMA0_SEC_BASE)
#define DMA0_NS                     ((DMA1_SEC_Type*)          DMA0_NS_BASE)
#define OSPI0                       ((OSPI0_Type*)             OSPI0_BASE)
#define AES0                        ((AES0_Type*)              AES0_BASE)
#define OSPI1                       ((OSPI0_Type*)             OSPI1_BASE)
#define AES1                        ((AES0_Type*)              AES1_BASE)
#define M55HP_NVIC_S                ((M55HP_NVIC_S_Type*)      M55HP_NVIC_S_BASE)
#define M55HP_NVIC_NS               ((M55HP_NVIC_S_Type*)      M55HP_NVIC_NS_BASE)

/** @} */ /* End of group Device_Peripheral_declaration */




/******************************************************************************/
/*                         Global memory map                              */
/******************************************************************************/

/*On chip RAM Regions */
#define SRAM0_BASE                0x02000000
#define SRAM0_SIZE                0x00400000		/* 4M */
#define SRAM1_BASE                0x08000000
#define SRAM1_SIZE                0x00280000		/* 2.5M */
#define SRAM2_BASE                0x50000000
#define SRAM2_SIZE                0x00040000		/* 256K */
#define SRAM3_BASE                0x50800000
#define SRAM3_SIZE                0x00100000		/* 1M */
#define SRAM4_BASE                0x60000000
#define SRAM4_SIZE                0x00040000		/* 256K */
#define SRAM5_BASE                0x60800000
#define SRAM5_SIZE                0x00040000		/* 256K */
#define SRAM6_BASE                0x62000000
#define SRAM6_SIZE                0x00200000		/* 2M */
#define SRAM7_BASE                0x63000000
#define SRAM7_SIZE                0x00080000		/* 512K */
#define SRAM8_BASE                0x63100000
#define SRAM8_SIZE                0x00200000		/* 2M */
#define SRAM9_BASE                0x64000000
#define SRAM9_SIZE                0x000C0000		/* 768K */
/* On Chip NVM */
#define MRAM_BASE                 0x80000000
#define MRAM_SIZE                 0x00580000		/* 5.5M */

#if 0 // This is Ax code; will be removed eventually

/* Peripheral regions Base Address */
#define EXPMST0_BASE              (0x48000000UL)
#define LP_PERIPHERAL_BASE        (0x70000000UL)

/* EXPMST0 Global Peripheral Address Map */

/* AHB - 2 */
#define EXPMST0_AHB_A_BASE        (EXPMST0_BASE)
#define EXPMST0_AHB_B_BASE        (EXPMST0_BASE + 0x00100000)

/* AHB-A Peripherals */
#define UTIMER_BASE               (EXPMST0_AHB_A_BASE)

/* AHB-B Peripherals */
#define ETH_BASE                  (EXPMST0_AHB_B_BASE)
#define SDMMC_BASE                (EXPMST0_AHB_B_BASE + 0x00002000)
#define SPI0_BASE                 (EXPMST0_AHB_B_BASE + 0x00003000)
#define SPI1_BASE                 (EXPMST0_AHB_B_BASE + 0x00004000)
#define SPI2_BASE                 (EXPMST0_AHB_B_BASE + 0x00005000)
#define SPI3_BASE                 (EXPMST0_AHB_B_BASE + 0x00006000)
#define CRC0_BASE                 (EXPMST0_AHB_B_BASE + 0x00007000)
#define CRC1_BASE                 (EXPMST0_AHB_B_BASE + 0x00008000)
#define USB0_BASE                 (EXPMST0_AHB_B_BASE + 0x00100000)

/* APB - 5 */
#define EXPMST0_APB_A_BASE        (EXPMST0_BASE + 0x01000000)
#define EXPMST0_APB_B_BASE        (EXPMST0_BASE + 0x01010000)
#define EXPMST0_APB_C_BASE        (EXPMST0_BASE + 0x01020000)
#define EXPMST0_APB_D_BASE        (EXPMST0_BASE + 0x01030000)
#define EXPMST0_APB_E_BASE        (EXPMST0_BASE + 0x01040000)


/* APB-A Peripherals */
#define GPIO1_BASE                  (EXPMST0_APB_A_BASE)
#define GPIO2_BASE                  (EXPMST0_APB_A_BASE + 0x00001000)
#define GPIO3_BASE                  (EXPMST0_APB_A_BASE + 0x00002000)
#define GPIO4_BASE                  (LP_PERIPHERAL_BASE)
#define UART0_BASE                  (EXPMST0_APB_A_BASE + 0x00008000)
#define UART1_BASE                  (EXPMST0_APB_A_BASE + 0x00009000)
#define UART2_BASE                  (EXPMST0_APB_A_BASE + 0x0000A000)
#define UART3_BASE                  (EXPMST0_APB_A_BASE + 0x0000B000)
#define UART4_BASE                  (EXPMST0_APB_A_BASE + 0x0000C000)
#define UART5_BASE                  (EXPMST0_APB_A_BASE + 0x0000D000)
#define UART6_BASE                  (EXPMST0_APB_A_BASE + 0x0000E000)
#define UART7_BASE                  (EXPMST0_APB_A_BASE + 0x0000F000)

/* APB-B Peripherals */
#define I2C0_BASE                 (EXPMST0_APB_B_BASE)
#define I2C1_BASE                 (EXPMST0_APB_B_BASE + 0x00001000)
#define I2C2_BASE                 (EXPMST0_APB_B_BASE + 0x00002000)
#define I2C3_BASE                 (EXPMST0_APB_B_BASE + 0x00003000)
#define I2S0_BASE                 (EXPMST0_APB_B_BASE + 0x00004000)
#define I2S1_BASE                 (EXPMST0_APB_B_BASE + 0x00005000)
#define I2S2_BASE                 (EXPMST0_APB_B_BASE + 0x00006000)
#define I2S3_BASE                 (EXPMST0_APB_B_BASE + 0x00007000)
#define I3C0_BASE                 (EXPMST0_APB_B_BASE + 0x00008000)
#define CANFD_BASE                (EXPMST0_APB_B_BASE + 0x00009000)
#define CAN_CNT_BASE              (EXPMST0_APB_B_BASE + 0x0000A000)
#define HWSEM0_BASE               (EXPMST0_APB_B_BASE + 0x0000B000)
#define HWSEM1_BASE               (EXPMST0_APB_B_BASE + 0x0000B010)
#define HWSEM2_BASE               (EXPMST0_APB_B_BASE + 0x0000B020)
#define HWSEM3_BASE               (EXPMST0_APB_B_BASE + 0x0000B030)
#define HWSEM4_BASE               (EXPMST0_APB_B_BASE + 0x0000B040)
#define HWSEM5_BASE               (EXPMST0_APB_B_BASE + 0x0000B050)
#define HWSEM6_BASE               (EXPMST0_APB_B_BASE + 0x0000B060)
#define HWSEM7_BASE               (EXPMST0_APB_B_BASE + 0x0000B070)
#define HWSEM8_BASE               (EXPMST0_APB_B_BASE + 0x0000B080)
#define HWSEM9_BASE               (EXPMST0_APB_B_BASE + 0x0000B090)
#define HWSEM10_BASE              (EXPMST0_APB_B_BASE + 0x0000B0A0)
#define HWSEM11_BASE              (EXPMST0_APB_B_BASE + 0x0000B0B0)
#define HWSEM12_BASE              (EXPMST0_APB_B_BASE + 0x0000B0C0)
#define HWSEM13_BASE              (EXPMST0_APB_B_BASE + 0x0000B0D0)
#define HWSEM14_BASE              (EXPMST0_APB_B_BASE + 0x0000B0E0)
#define HWSEM15_BASE              (EXPMST0_APB_B_BASE + 0x0000B0F0)
#define PDM_BASE                  (EXPMST0_APB_B_BASE + 0x0000C000)

/* APB-C Peripherals */
#define ADC0_BASE                 (EXPMST0_APB_C_BASE)
#define ADC1_BASE                 (EXPMST0_APB_C_BASE + 0x00001000)
#define ADC2_BASE                 (EXPMST0_APB_C_BASE + 0x00002000)
#define COMP0_BASE                (EXPMST0_APB_C_BASE + 0x00003000)
#define COMP1_BASE                (EXPMST0_APB_C_BASE + 0x00004000)
#define COMP2_BASE                (EXPMST0_APB_C_BASE + 0x00005000)
#define COMP3_BASE                (EXPMST0_APB_C_BASE + 0x00006000)
#define DAC0_BASE                 (EXPMST0_APB_C_BASE + 0x00008000)
#define DAC1_BASE                 (EXPMST0_APB_C_BASE + 0x00009000)
#define CFGMST0_BASE              (EXPMST0_APB_C_BASE + 0x0000F000)
#define CFGMST0_GPIO_CTRL         (CFGMST0_BASE + 0x00000004)
#define CFGMST0_SSI               (CFGMST0_BASE + 0x00000028)

/* APB-D Peripherals */
#define CAMERA0_BASE              (EXPMST0_APB_D_BASE)
#define CDC200_BASE               (EXPMST0_APB_D_BASE + 0x00001000)
#define MIPI_DSI_BASE             (EXPMST0_APB_D_BASE + 0x00002000)
#define MIPI_CSI2_BASE            (EXPMST0_APB_D_BASE + 0x00003000)
#define CFGSLV1_BASE              (EXPMST0_APB_D_BASE + 0x0000F000)
#define CDC200_PIXCLK_CTRL        (CFGSLV1_BASE + 0x00000004)
#define CSI_PIXCLK_CTRL           (CFGSLV1_BASE + 0x00000008)

/* APB-E Peripherals */
#define AES0_BASE                 (EXPMST0_APB_E_BASE)
#define AES1_BASE                 (EXPMST0_APB_E_BASE + 0x01000)
#define DAVE2D_BASE               (EXPMST0_APB_E_BASE + 0x02000)
#define DMA0_SE_BASE              (EXPMST0_APB_E_BASE + 0x40000)
#define DMA0_NS_BASE              (EXPMST0_APB_E_BASE + 0x60000)

/* VBAT Global Address Map */
#define VBAT_BASE                 (LP_PERIPHERAL_BASE)
#define RTC0_BASE                 (VBAT_BASE + 0x00010000)
#define LPTIMER_BASE              (VBAT_BASE + 0x00030000)
#if RTE_SILICON_REV_A
#define LPTIMER_CLK_SEL_REG       (LPTIMER_BASE + 0x00010028)
#endif
#if RTE_SILICON_REV_B0
#define LPTIMER_CLK_SEL_REG       (0x1A609004)
#endif
#define VBAT_REGS_BASE            (VBAT_BASE + 0x00040000)
#define VBAT_ANA_REG2_BASE        (VBAT_REGS_BASE + 0x0000001C)

/* LPAON Global Address Map */
#define PADCTRL_BASE                        (LPAON_VBAT_REGS_BASE)
#define VBAT_GPIOV_REGS                     (LP_PERIPHERAL_BASE + 0x70000)
#define LP_PADCTRL_BASE                     (VBAT_GPIOV_REGS + 0x160)
#define LPAON_BASE                          (LP_PERIPHERAL_BASE + 0x01000000)
#define PINMUX_BASE                         (LPAON_BASE + 0x00006000)
#define LPAON_VBAT_REGS_BASE                (LPAON_BASE + 0x00007000)
#define LPAON_CLK_RST_4_BASE                (LPAON_VBAT_REGS_BASE + 0x00000410)
#define LPAON_REGS_BASE                     (LPAON_BASE + 0x0000A000)
#define LPAON_ANA_XTAL_OSCILLATOR_REG_BASE  (LPAON_REGS_BASE + 0x00000100)

/* Ethernet 50MHz clock mux register */
#define ETH_50M_CLK_MUX_REG       (LPAON_VBAT_REGS_BASE + 0x00000408)

/* OSPI Address Map */
#define OSPI0_BASE                0xD0000000
#define OSPI1_BASE                0xD8000000
#define OSPI0_SIZE                0x08000000UL
#define OSPI1_SIZE                0x08000000UL

#endif

#endif /* GLOBAL_MAP_H */
