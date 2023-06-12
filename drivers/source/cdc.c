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
 * @file     cdc.c
 * @author   Prasanna Ravi
 * @email    prasanna.ravi@alifsemi.com
 * @version  V1.0.0
 * @date     10-April-2023
 * @brief    Low level driver Specific Source file.
 ******************************************************************************/

#include "cdc.h"

/**
  \fn          void cdc_layer_on(CDC_Type *cdc, CDC_LAYER layer)
  \brief       cdc layer on.
  \param[in]   cdc      Pointer to the cdc register map.
  \param[in]   layer    cdc layer to on.
  \return      none.
*/
void cdc_layer_on(CDC_Type *cdc, CDC_LAYER layer)
{
    switch (layer)
    {
        case CDC_LAYER_1:
            cdc->CDC_L1_CTRL |= CDC_LAYER_ON_MASK;
            break;
        case CDC_LAYER_2:
            cdc->CDC_L2_CTRL |= CDC_LAYER_ON_MASK;
            break;
        case CDC_LAYER_3:
            cdc->CDC_L3_CTRL |= CDC_LAYER_ON_MASK;
            break;
        default:
            break;
    }
}

/**
  \fn          void cdc_layer_off(CDC_Type *cdc, CDC_LAYER layer)
  \brief       cdc layer off.
  \param[in]   cdc     Pointer to the cdc register map.
  \param[in]   layer   cdc layer to off.
  \return      none.
*/
void cdc_layer_off(CDC_Type *cdc, CDC_LAYER layer)
{
    switch (layer)
    {
        case CDC_LAYER_1:
            cdc->CDC_L1_CTRL &= ~CDC_LAYER_ON_MASK;
            break;
        case CDC_LAYER_2:
            cdc->CDC_L2_CTRL &= ~CDC_LAYER_ON_MASK;
            break;
        case CDC_LAYER_3:
            cdc->CDC_L3_CTRL &= ~CDC_LAYER_ON_MASK;
            break;
        default:
            break;
    }
}

/**
  \fn          void cdc_set_win_hpos(CDC_Type *cdc, CDC_LAYER layer, uint32_t hpos)
  \brief       Set cdc window Horizontal Position.
  \param[in]   cdc     Pointer to the cdc register map.
  \param[in]   layer   cdc layer to set window Horizontal Position.
  \param[in]   hpos    window Horizontal Position to set.
               hpos[31..16] Horizontal Stop Position.
               hpos[15..0]  Horizontal Start Position.
  \return      none.
*/
void cdc_set_win_hpos(CDC_Type *cdc, CDC_LAYER layer, uint32_t hpos)
{
    switch (layer)
    {
        case CDC_LAYER_1:
            cdc->CDC_L1_WIN_HPOS = hpos;
            break;
        case CDC_LAYER_2:
            cdc->CDC_L2_WIN_HPOS = hpos;
            break;
        case CDC_LAYER_3:
            cdc->CDC_L3_WIN_HPOS = hpos;
            break;
        default:
            break;
    }
}

/**
  \fn          uint32_t cdc_get_win_hpos(CDC_Type *cdc, CDC_LAYER layer)
  \brief       Get cdc window Horizontal Position.
  \param[in]   cdc     Pointer to the cdc register map.
  \param[in]   layer   cdc layer to get window Horizontal Position.
  \return      current window Horizontal Position value.
               bit[31..16] Horizontal Stop Position.
               bit[15..0]  Horizontal Start Position.
*/
uint32_t cdc_get_win_hpos(CDC_Type *cdc, CDC_LAYER layer)
{
    uint32_t val = 0;

    switch (layer)
    {
        case CDC_LAYER_1:
            val = cdc->CDC_L1_WIN_HPOS;
            break;
        case CDC_LAYER_2:
            val = cdc->CDC_L2_WIN_HPOS;
            break;
        case CDC_LAYER_3:
            val = cdc->CDC_L3_WIN_HPOS;
            break;
        default:
            break;
    }

    return val;
}

/**
  \fn          void cdc_set_win_vpos(CDC_Type *cdc, CDC_LAYER layer, uint32_t vpos)
  \brief       Set cdc window Vertical Position.
  \param[in]   cdc     Pointer to the cdc register map.
  \param[in]   layer   cdc layer to set window Vertical Position.
  \param[in]   vpos    window Vertical Position to set.
               vpos[31..16] Vertical Stop Position.
               vpos[15..0]  Vertical Start Position.
  \return      none.
*/
void cdc_set_win_vpos(CDC_Type *cdc, CDC_LAYER layer, uint32_t vpos)
{
    switch (layer)
    {
        case CDC_LAYER_1:
            cdc->CDC_L1_WIN_VPOS = vpos;
            break;
        case CDC_LAYER_2:
            cdc->CDC_L2_WIN_VPOS = vpos;
            break;
        case CDC_LAYER_3:
            cdc->CDC_L3_WIN_VPOS = vpos;
            break;
        default:
            break;
    }
}

/**
  \fn          uint32_t cdc_get_win_vpos(CDC_Type *cdc, CDC_LAYER layer)
  \brief       Get cdc window Vertical Position.
  \param[in]   cdc     Pointer to the cdc register map.
  \param[in]   layer   cdc layer to get window Vertical Position.
  \return      current window Vertical Position value.
               bit[31..16] Vertical Stop Position.
               bit[15..0]  Vertical Start Position.
*/
uint32_t cdc_get_win_vpos(CDC_Type *cdc, CDC_LAYER layer)
{
    uint32_t val = 0;

    switch (layer)
    {
        case CDC_LAYER_1:
            val = cdc->CDC_L1_WIN_VPOS;
            break;
        case CDC_LAYER_2:
            val = cdc->CDC_L2_WIN_VPOS;
            break;
        case CDC_LAYER_3:
            val = cdc->CDC_L3_WIN_VPOS;
            break;
        default:
            break;
    }

    return val;
}

/**
  \fn          void cdc_set_pixel_format(CDC_Type *cdc, CDC_LAYER layer, CDC_PIXEL_FORMAT pixel_format)
  \brief       Set cdc pixel format.
  \param[in]   cdc            Pointer to the cdc register map.
  \param[in]   layer          cdc layer to set pixel format.
  \param[in]   pixel_format   pixel format to set.
               bit[2..0] Pixel format.
  \return      none.
*/
void cdc_set_pixel_format(CDC_Type *cdc, CDC_LAYER layer, CDC_PIXEL_FORMAT pixel_format)
{
    switch (layer)
    {
        case CDC_LAYER_1:
            cdc->CDC_L1_PIX_FORMAT = pixel_format;
            break;
        case CDC_LAYER_2:
            cdc->CDC_L2_PIX_FORMAT = pixel_format;
            break;
        case CDC_LAYER_3:
            cdc->CDC_L3_PIX_FORMAT = pixel_format;
            break;
        default:
            break;
    }
}

/**
  \fn          CDC_PIXEL_FORMAT cdc_get_pixel_format(CDC_Type *cdc, CDC_LAYER layer)
  \brief       Get cdc pixel format.
  \param[in]   cdc     Pointer to the cdc register map.
  \param[in]   layer   cdc layer to get pixel format.
  \return      current pixel format value.
               bit[2..0] Pixel format.
*/
CDC_PIXEL_FORMAT cdc_get_pixel_format(CDC_Type *cdc, CDC_LAYER layer)
{
    uint32_t val = 0;

    switch (layer)
    {
        case CDC_LAYER_1:
            val = cdc->CDC_L1_PIX_FORMAT;
            break;
        case CDC_LAYER_2:
            val = cdc->CDC_L2_PIX_FORMAT;
            break;
        case CDC_LAYER_3:
            val = cdc->CDC_L3_PIX_FORMAT;
            break;
        default:
            break;
    }

    return val;
}

/**
  \fn          void cdc_set_constant_alpha(CDC_Type *cdc, CDC_LAYER layer, uint32_t alpha_constant)
  \brief       Set cdc alpha constant.
  \param[in]   cdc              Pointer to the cdc register map.
  \param[in]   layer            cdc layer to set alpha constant.
  \param[in]   alpha_constant   alpha constant to set.
               bit[7..0] Constant alpha.
  \return      none.
*/
void cdc_set_constant_alpha(CDC_Type *cdc, CDC_LAYER layer, uint32_t alpha_constant)
{
    switch (layer)
    {
        case CDC_LAYER_1:
            cdc->CDC_L1_CONST_ALPHA = alpha_constant;
            break;
        case CDC_LAYER_2:
            cdc->CDC_L2_CONST_ALPHA = alpha_constant;
            break;
        case CDC_LAYER_3:
            cdc->CDC_L3_CONST_ALPHA = alpha_constant;
            break;
        default:
            break;
    }
}

/**
  \fn          uint32_t cdc_get_constant_alpha(CDC_Type *cdc, CDC_LAYER layer)
  \brief       Get cdc alpha constant.
  \param[in]   cdc     Pointer to the cdc register map.
  \param[in]   layer   cdc layer to get alpha constant.
  \return      current alpha constant value.
               bit[7..0] Constant alpha.
*/
uint32_t cdc_get_constant_alpha(CDC_Type *cdc, CDC_LAYER layer)
{
    uint32_t val = 0;

    switch (layer)
    {
        case CDC_LAYER_1:
            val = cdc->CDC_L1_CONST_ALPHA;
            break;
        case CDC_LAYER_2:
            val = cdc->CDC_L2_CONST_ALPHA;
            break;
        case CDC_LAYER_3:
            val = cdc->CDC_L3_CONST_ALPHA;
            break;
        default:
            break;
    }

    return val;
}


/**
  \fn          void cdc_set_fb_address(CDC_Type *cdc, CDC_LAYER layer, uint32_t fb)
  \brief       Set cdc frame buffer address.
  \param[in]   cdc     Pointer to the cdc register map.
  \param[in]   layer   cdc layer to set frame buffer address.
  \param[in]   addr    frame buffer address to set.
  \return      none.
*/
void cdc_set_fb_address(CDC_Type *cdc, CDC_LAYER layer, uint32_t addr)
{
    switch (layer)
    {
        case CDC_LAYER_1:
            cdc->CDC_L1_CFB_ADDR = addr;
            break;
        case CDC_LAYER_2:
            cdc->CDC_L2_CFB_ADDR = addr;
            break;
        case CDC_LAYER_3:
            cdc->CDC_L3_CFB_ADDR = addr;
            break;
        default:
            break;
    }
}

/**
  \fn          uint32_t  cdc_get_fb_address(CDC_Type *cdc, CDC_LAYER layer)
  \brief       Get cdc frame buffer address.
  \param[in]   cdc     Pointer to the cdc register map.
  \param[in]   layer   cdc layer to get frame buffer address.
  \return      current frame buffer address value.
*/
uint32_t  cdc_get_fb_address(CDC_Type *cdc, CDC_LAYER layer)
{
    uint32_t val = 0;

    switch (layer)
    {
        case CDC_LAYER_1:
            val = cdc->CDC_L1_CFB_ADDR;
            break;
        case CDC_LAYER_2:
            val = cdc->CDC_L2_CFB_ADDR;
            break;
        case CDC_LAYER_3:
            val = cdc->CDC_L3_CFB_ADDR;
            break;
        default:
            break;
    }

    return val;
}

/**
  \fn          void cdc_set_fb_length(CDC_Type *cdc, CDC_LAYER layer, uint32_t fb)
  \brief       Set cdc frame buffer length.
  \param[in]   cdc         Pointer to the cdc register map.
  \param[in]   layer       cdc layer to set frame buffer length.
  \param[in]   fb_length   frame buffer length to set.
               fb_length[31..16] Pitch of color FB in bytes (signed).
               fb_length[15..0]  Line length of color FB in bytes – 1 + (width of bus in bytes).
  \return      none.
*/
void cdc_set_fb_length(CDC_Type *cdc, CDC_LAYER layer, uint32_t fb_length)
{
    switch (layer)
    {
        case CDC_LAYER_1:
            cdc->CDC_L1_CFB_LENGTH = fb_length;
            break;
        case CDC_LAYER_2:
            cdc->CDC_L2_CFB_LENGTH = fb_length;
            break;
        case CDC_LAYER_3:
            cdc->CDC_L3_CFB_LENGTH = fb_length;
            break;
        default:
            break;
    }
}

/**
  \fn          uint32_t  cdc_get_fb_length(CDC_Type *cdc, CDC_LAYER layer)
  \brief       Get cdc frame buffer length.
  \param[in]   cdc     Pointer to the cdc register map.
  \param[in]   layer   cdc layer to get frame buffer length.
  \return      current frame buffer length value.
               bit[31..16] Pitch of color FB in bytes (signed).
               bit[15..0]  Line length of color FB in bytes – 1 + (width of bus in bytes).
*/
uint32_t  cdc_get_fb_length(CDC_Type *cdc, CDC_LAYER layer)
{
    uint32_t val = 0;

    switch (layer)
    {
        case CDC_LAYER_1:
            val = cdc->CDC_L1_CFB_LENGTH;
            break;
        case CDC_LAYER_2:
            val = cdc->CDC_L2_CFB_LENGTH;
            break;
        case CDC_LAYER_3:
            val = cdc->CDC_L3_CFB_LENGTH;
            break;
        default:
            break;
    }

    return val;
}

/**
  \fn          void cdc_set_fb_lines(CDC_Type *cdc, CDC_LAYER layer, uint32_t fb)
  \brief       Set cdc frame buffer lines.
  \param[in]   cdc      Pointer to the cdc register map.
  \param[in]   layer    cdc layer to set frame buffer lines.
  \param[in]   fb_lines frame buffer lines to set.
               bit[15..0] Number of lines in the color FB.
  \return      none.
*/
void cdc_set_fb_lines(CDC_Type *cdc, CDC_LAYER layer, uint32_t fb_lines)
{
    switch (layer)
    {
        case CDC_LAYER_1:
            cdc->CDC_L1_CFB_LINES = fb_lines;
            break;
        case CDC_LAYER_2:
            cdc->CDC_L2_CFB_LINES = fb_lines;
            break;
        case CDC_LAYER_3:
            cdc->CDC_L3_CFB_LINES = fb_lines;
            break;
        default:
            break;
    }
}

/**
  \fn          uint32_t  cdc_get_fb_lines(CDC_Type *cdc, CDC_LAYER layer)
  \brief       Get cdc frame buffer lines.
  \param[in]   cdc      Pointer to the cdc register map.
  \param[in]   layer    cdc layer to get frame buffer lines.
  \return      current frame buffer lines value.
               bit[15..0] Number of lines in the color FB.
*/
uint32_t  cdc_get_fb_lines(CDC_Type *cdc, CDC_LAYER layer)
{
    uint32_t val = 0;

    switch (layer)
    {
        case CDC_LAYER_1:
            val = cdc->CDC_L1_CFB_LINES;
            break;
        case CDC_LAYER_2:
            val = cdc->CDC_L2_CFB_LINES;
            break;
        case CDC_LAYER_3:
            val = cdc->CDC_L3_CFB_LINES;
            break;
        default:
            break;
    }

    return val;
}
