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
 * @file     cpi.c
 * @author   Chandra Bhushan Singh
 * @email    chandrabhushan.singh@alifsemi.com
 * @version  V1.0.0
 * @date     27-March-2023
 * @brief    Low level driver Specific source file.
 ******************************************************************************/

#include "cpi.h"

/**
  \fn          void cpi_start_snapshot_mode(LPCPI_Type *cpi)
  \brief       Capture frame in snapshot mode.
                   -Set CAM_CTRL = 0—prepare for soft reset
                   -Set CAM_CTRL = 0x100—activate soft reset
                   -Set CAM_CTRL = 0—stop soft reset
                   -Set CAM_CTRL = 0x1001 or 0x1011—start the CPI controller,
                    with bit [SNAPSHOT] defining the operation mode:
                        -[SNAPSHOT] = 0—Capture video frames continuously
                        -[SNAPSHOT] = 1—Capture one frame then stop
  \param[in]   cpi      Pointer to the CPI register map.
  \return      none.
*/
void cpi_start_snapshot_mode(LPCPI_Type *cpi)
{
    cpi->CAM_CTRL = 0;
    cpi->CAM_CTRL |= CAM_CTRL_SW_RESET;
    cpi->CAM_CTRL = 0;
    cpi->CAM_CTRL = (CAM_CTRL_SNAPSHOT | CAM_CTRL_START | CAM_CTRL_FIFO_CLK_SEL);
}

/**
  \fn          void cpi_start_video_mode(LPCPI_Type *cpi)
  \brief       Capture frames in video mode.
                   -Set CAM_CTRL = 0—prepare for soft reset
                   -Set CAM_CTRL = 0x100—activate soft reset
                   -Set CAM_CTRL = 0—stop soft reset
                   -Set CAM_CTRL = 0x1001 or 0x1011—start the CPI controller,
                    with bit [SNAPSHOT] defining the operation mode:
                        -[SNAPSHOT] = 0—Capture video frames continuously
                        -[SNAPSHOT] = 1—Capture one frame then stop
  \param[in]   cpi      Pointer to the CPI register map.
  \return      none.
*/
void cpi_start_video_mode(LPCPI_Type *cpi)
{
    cpi->CAM_CTRL = 0;
    cpi->CAM_CTRL |= CAM_CTRL_SW_RESET;
    cpi->CAM_CTRL = 0;
    cpi->CAM_CTRL = (CAM_CTRL_START | CAM_CTRL_FIFO_CLK_SEL);
}

/**
  \fn           void cpi_set_csi_halt(LPCPI_Type *cpi, CPI_CSI_IPI_HALT_FN select)
  \brief        Enable/Disable MIPI CSI controller halt function.
  \param[in]    cpi    Pointer to CPI register map
  \param[in]    select 0: Disable CSI IPI halt function
                       1: Enable CSI IPI halt function
  \return       none
*/
void cpi_set_csi_halt(LPCPI_Type *cpi, CPI_CSI_IPI_HALT_FN select)
{
    if(select == CPI_CSI_IPI_HALT_FN_ENABLE)
    {
        cpi->CAM_CFG |= CAM_CFG_CSI_HALT_EN;
    }
    else
    {
        cpi->CAM_CFG &= ~CAM_CFG_CSI_HALT_EN;
    }
}

/**
  \fn           void cpi_set_vsync_wait(LPCPI_Type *cpi, CPI_WAIT_VSYNC select)
  \brief        Enable/Disable capture video frame on the rising edge of VSYNC.
  \param[in]    cpi   Pointer to CPI register map
  \param[in]    select 0: Start video capture without waiting for VSYNC
                       1: Start video capture on rising edge of VSYNC
  \return       none
*/
void cpi_set_vsync_wait(LPCPI_Type *cpi, CPI_WAIT_VSYNC select)
{
    if(select == CPI_WAIT_VSYNC_ENABLE)
    {
        cpi->CAM_CFG |= CAM_CFG_WAIT_VSYNC;
    }
    else
    {
        cpi->CAM_CFG &= ~CAM_CFG_WAIT_VSYNC;
    }
}

/**
  \fn           void cpi_set_data_synchronization(LPCPI_Type *cpi, CPI_CAPTURE_DATA_ENABLE select)
  \brief        Capture data when VSYNC is high.
  \param[in]    cpi  Pointer to CPI register map
  \param[in]    select 0: Capture data regardless of VSYNC status
                       1: Capture data when VSYNC is high
  \return       none
*/
void cpi_set_data_synchronization(LPCPI_Type *cpi, CPI_CAPTURE_DATA_ENABLE select)
{
    if(select == CPI_CAPTURE_DATA_ENABLE_IF_VSYNC_AND_HSYNC_HIGH)
    {
        cpi->CAM_CFG |= CAM_CFG_VSYNC_EN;
    }
    else
    {
        cpi->CAM_CFG &= ~CAM_CFG_VSYNC_EN;
    }
}

/**
  \fn           void cpi_set_row_roundup(LPCPI_Type *cpi, CPI_ROW_ROUNDUP select)
  \brief        Enable/Disable round up pixel data to 64-bit at the end of each row..
  \param[in]    cpi    Pointer to CPI register map
  \param[in]    select 0: Not round up
                       1: Round up pixel data to 64-bit at the end of each row
  \return       none
*/
void cpi_set_row_roundup(LPCPI_Type *cpi, CPI_ROW_ROUNDUP select)
{
    if(select == CPI_ROW_ROUNDUP_ENABLE)
    {
        cpi->CAM_CFG |= CAM_CFG_ROW_ROUNDUP;
    }
    else
    {
        cpi->CAM_CFG &= ~CAM_CFG_ROW_ROUNDUP;
    }
}

/**
  \fn           void cpi_set_pixelclk_polarity(LPCPI_Type *cpi, CPI_SIG_POLARITY polarity)
  \brief        Invert/Not invert CPI pixel clock polarity.
  \param[in]    cpi      Pointer to CPI register map
  \param[in]    polarity 0: Not invert external camera PIXEL_CLK
                         1: Invert external camera PIXEL_CLK
  \return       none
*/
void cpi_set_pixelclk_polarity(LPCPI_Type *cpi, CPI_SIG_POLARITY polarity)
{
    if(polarity == CPI_SIG_POLARITY_INVERT_ENABLE)
    {
        cpi->CAM_CFG &= ~CAM_CFG_PCLK_POL;
    }
    else
    {
        cpi->CAM_CFG |= CAM_CFG_PCLK_POL;
    }
}

/**
  \fn           void cpi_set_hsync_polarity(LPCPI_Type *cpi, CPI_SIG_POLARITY polarity)
  \brief        Invert/Not invert CPI hsync polarity.
  \param[in]    cpi      Pointer to CPI register map
  \param[in]    polarity 0: Not invert HSYNC input
                         1: Invert HSYNC input
  \return       none
*/
void cpi_set_hsync_polarity(LPCPI_Type *cpi, CPI_SIG_POLARITY polarity)
{
    if(polarity == CPI_SIG_POLARITY_INVERT_ENABLE)
    {
        cpi->CAM_CFG &= ~CAM_CFG_HSYNC_POL;
    }
    else
    {
        cpi->CAM_CFG |= CAM_CFG_HSYNC_POL;
    }
}

/**
  \fn           void cpi_set_vsync_polarity(LPCPI_Type *cpi, CPI_SIG_POLARITY polarity)
  \brief        Invert/Not invert CPI vsync polarity.
  \param[in]    cpi      Pointer to CPI register map
  \param[in]    polarity 0: Not invert VSYNC input
                         1: Invert VSYNC input
  \return       none
*/
void cpi_set_vsync_polarity(LPCPI_Type *cpi, CPI_SIG_POLARITY polarity)
{
    if(polarity == CPI_SIG_POLARITY_INVERT_ENABLE)
    {
        cpi->CAM_CFG &= ~CAM_CFG_VSYNC_POL;
    }
    else
    {
        cpi->CAM_CFG |= CAM_CFG_VSYNC_POL;
    }
}

/**
  \fn           void cpi_set_code10on8bit_coding(LPCPI_Type *cpi, CPI_CODE10ON8_CODING select)
  \brief        Special coding: transfer 10-bit coding over 8-bit data bus.
                Valid only when [DATA_MODE] field is set to 8-bit.
  \param[in]    cpi    Pointer to CPI register map
  \param[in]    select 0: Disable special 10-bit coding
                       1: Enable special 10-bit coding
  \return       none
*/
void cpi_set_code10on8bit_coding(LPCPI_Type *cpi, CPI_CODE10ON8_CODING select)
{
    if(select == CPI_CODE10ON8_CODING_ENABLE)
    {
        cpi->CAM_CFG |= CAM_CFG_CODE10ON8;
    }
    else
    {
        cpi->CAM_CFG &= ~CAM_CFG_CODE10ON8;
    }
}

/**
  \fn           void cpi_set_data_field(LPCPI_Type *cpi, CPI_DATA_FIELD select)
  \brief        Select MSB/LSB first to be stored in a memory.
                Valid only when [DATA_MODE] field is set to 1-/2-/4-/8-bit.
  \param[in]    cpi    Pointer to CPI register map
  \param[in]    select 0: LSB
                       1: MSB
  \return       none
*/
void cpi_set_data_field(LPCPI_Type *cpi, CPI_DATA_FIELD select)
{
    if(select == CPI_DATA_FIELD_MSB_FIRST)
    {
        cpi->CAM_CFG |= CAM_CFG_MSB;
    }
    else
    {
        cpi->CAM_CFG &= ~CAM_CFG_MSB;
    }
}

/**
  \fn          void cpi_set_fifo_control(LPCPI_Type *cpi, uint8_t r_wmark, uint8_t w_wmark)
  \brief       Set CPI FIFO Configurations:
                 - FIFO Read Water Mark
                 - FIFO Write Water Mark
  \param[in]   cpi     Pointer to the CPI register map.
  \param[in]   r_wmark FIFO read watermark.
  \param[in]   w_wmark FIFO write watermark.
  \return      none.
*/
void cpi_set_fifo_control(LPCPI_Type *cpi, uint8_t r_wmark, uint8_t w_wmark)
{
    cpi->CAM_FIFO_CTRL &= ~CAM_FIFO_CTRL_RD_WMARK_Msk;
    cpi->CAM_FIFO_CTRL |= r_wmark;
    cpi->CAM_FIFO_CTRL &= ~CAM_FIFO_CTRL_WR_WMARK_Msk;
    cpi->CAM_FIFO_CTRL |= (w_wmark << CAM_FIFO_CTRL_WR_WMARK_Pos);
}

/**
  \fn          void cpi_set_frame_config(LPCPI_Type *cpi, uint16_t frame_width, uint16_t frame_height)
  \brief       Set CPI Frame Configurations:
                 - Frame Width(Column)
                 - Frame Height(Row)
  \param[in]   cpi          Pointer to the CPI register map.
  \param[in]   frame_width  valid data in a row.
  \param[in]   frame_height valid data rows in a frame.
  \return      none.
*/
void cpi_set_frame_config(LPCPI_Type *cpi, uint16_t frame_width, uint16_t frame_height)
{
    cpi->CAM_VIDEO_FCFG &= ~CAM_VIDEO_FCFG_DATA_Msk;
    cpi->CAM_VIDEO_FCFG = frame_width;
    cpi->CAM_VIDEO_FCFG &= ~CAM_VIDEO_FCFG_ROW_Msk;
    cpi->CAM_VIDEO_FCFG |= (frame_height << CAM_VIDEO_FCFG_ROW_Pos);
}
