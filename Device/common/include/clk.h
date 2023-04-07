/* Copyright (C) 2022 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

#ifndef CLK_H_
#define CLK_H_

#include <peripheral_types.h>

#ifndef AXI_CLOCK
#define AXI_CLOCK 76800000
#endif

#ifndef AHB_CLOCK
#define AHB_CLOCK 38400000
#endif

#ifndef APB_CLOCK
#define APB_CLOCK 19200000
#endif

#define I2C_PERIPHERAL_CLOCK    APB_CLOCK

#define CDC200_PIXCLK           AXI_CLOCK

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

static inline void enable_cpi_periph_clk(void)
{
    CLKCTL_PER_MST->PERIPH_CLK_ENA |= PERIPH_CLK_ENA_CPI_CKEN;
}

static inline void disable_cpi_periph_clk(void)
{
    CLKCTL_PER_MST->PERIPH_CLK_ENA &= ~PERIPH_CLK_ENA_CPI_CKEN;
}

static inline void enable_dpi_periph_clk(void)
{
    CLKCTL_PER_MST->PERIPH_CLK_ENA |= PERIPH_CLK_ENA_DPI_CKEN;
}

static inline void disable_dpi_periph_clk(void)
{
    CLKCTL_PER_MST->PERIPH_CLK_ENA &= ~PERIPH_CLK_ENA_DPI_CKEN;
}

static inline void enable_dma_periph_clk(void)
{
    CLKCTL_PER_MST->PERIPH_CLK_ENA |= PERIPH_CLK_ENA_DMA_CKEN;
}

static inline void disable_dma_periph_clk(void)
{
    CLKCTL_PER_MST->PERIPH_CLK_ENA &= ~PERIPH_CLK_ENA_DMA_CKEN;
}

static inline void enable_gpu_periph_clk(void)
{
    CLKCTL_PER_MST->PERIPH_CLK_ENA |= PERIPH_CLK_ENA_GPU_CKEN;
}

static inline void disable_gpu_periph_clk(void)
{
    CLKCTL_PER_MST->PERIPH_CLK_ENA &= ~PERIPH_CLK_ENA_GPU_CKEN;
}

static inline void enable_eth_periph_clk(void)
{
    CLKCTL_PER_MST->PERIPH_CLK_ENA |= PERIPH_CLK_ENA_ETH_CKEN;
}

static inline void disable_eth_periph_clk(void)
{
    CLKCTL_PER_MST->PERIPH_CLK_ENA &= ~PERIPH_CLK_ENA_ETH_CKEN;
}

static inline void enable_sdc_periph_clk(void)
{
    CLKCTL_PER_MST->PERIPH_CLK_ENA |= PERIPH_CLK_ENA_SDC_CKEN;
}

static inline void disable_sdc_periph_clk(void)
{
    CLKCTL_PER_MST->PERIPH_CLK_ENA &= ~PERIPH_CLK_ENA_SDC_CKEN;
}

static inline void enable_usb_periph_clk(void)
{
    CLKCTL_PER_MST->PERIPH_CLK_ENA |= PERIPH_CLK_ENA_USB_CKEN;
}

static inline void disable_usb_periph_clk(void)
{
    CLKCTL_PER_MST->PERIPH_CLK_ENA &= ~PERIPH_CLK_ENA_USB_CKEN;
}

static inline void enable_csi_periph_clk(void)
{
    CLKCTL_PER_MST->PERIPH_CLK_ENA |= PERIPH_CLK_ENA_CSI_CKEN;
}

static inline void disable_csi_periph_clk(void)
{
    CLKCTL_PER_MST->PERIPH_CLK_ENA &= ~PERIPH_CLK_ENA_CSI_CKEN;
}

static inline void enable_dsi_periph_clk(void)
{
    CLKCTL_PER_MST->PERIPH_CLK_ENA |= PERIPH_CLK_ENA_DSI_CKEN;
}

static inline void disable_dsi_periph_clk(void)
{
    CLKCTL_PER_MST->PERIPH_CLK_ENA &= ~PERIPH_CLK_ENA_DSI_CKEN;
}

#endif /* CLK_H_ */
