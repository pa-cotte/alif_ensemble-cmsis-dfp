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
 * @file     cdc.h
 * @author   Prasanna Ravi
 * @email    prasanna.ravi@alifsemi.com
 * @version  V1.0.0
 * @date     10-April-2023
 * @brief    Low level driver Specific Header file.
 ******************************************************************************/

#ifndef CDC_H_
#define CDC_H_

#ifdef  __cplusplus
extern "C"
{
#endif

#include <stdint.h>

typedef struct {                                          /*!< (@ 0x49031000) CDC Structure                                              */
    volatile const uint32_t CDC_HW_VER;                   /*!< (@ 0x00000000) HW Version Register                                        */
    volatile const uint32_t CDC_LCNT;                     /*!< (@ 0x00000004) Layer Count Register                                       */
    volatile       uint32_t CDC_SYNC_SIZE_CFG;            /*!< (@ 0x00000008) Sync Size Register                                         */
    volatile       uint32_t CDC_BP_CFG;                   /*!< (@ 0x0000000C) Back Porch Register                                        */
    volatile       uint32_t CDC_ACTW_CFG;                 /*!< (@ 0x00000010) Active Width Register                                      */
    volatile       uint32_t CDC_TOTALW_CFG;               /*!< (@ 0x00000014) Total Width Register                                       */
    volatile       uint32_t CDC_GLB_CTRL;                 /*!< (@ 0x00000018) Global Control Register                                    */
    volatile const uint32_t CDC_CFG1;                     /*!< (@ 0x0000001C) Global Configuration 1 Register                            */
    volatile const uint32_t CDC_CFG2;                     /*!< (@ 0x00000020) Global Configuration 2 Register                            */
    volatile       uint32_t CDC_SRCTRL;                   /*!< (@ 0x00000024) Shadow Reload Control Register                             */
    volatile       uint32_t CDC_GAMMA_CORR;               /*!< (@ 0x00000028) Gamma Correction Register                                  */
    volatile       uint32_t CDC_BACKGND_COLOR;            /*!< (@ 0x0000002C) Background Color Register                                  */
    volatile const uint32_t RESERVED;
    volatile       uint32_t CDC_IRQ_MASK0;                /*!< (@ 0x00000034) IRQ Enable 1 Register                                      */
    volatile const uint32_t CDC_IRQ_STATUS0;              /*!< (@ 0x00000038) IRQ Status 1 Register                                      */
    volatile       uint32_t CDC_IRQ_CLEAR0;               /*!< (@ 0x0000003C) IRQ Clear Register                                         */
    volatile       uint32_t CDC_LINE_IRQ_POS;             /*!< (@ 0x00000040) Line Number IRQ Control Register                           */
    volatile const uint32_t CDC_POS_STAT;                 /*!< (@ 0x00000044) Position Status Register                                   */
    volatile const uint32_t CDC_SYNC_BLANK_STAT;          /*!< (@ 0x00000048) Sync/Blank Status Register                                 */
    volatile const uint32_t RESERVED1[6];
    volatile       uint32_t CDC_IRQ_MASK1;                /*!< (@ 0x00000064) Secondary IRQ Enable Register                              */
    volatile const uint32_t CDC_IRQ_STATUS1;              /*!< (@ 0x00000068) Secondary IRQ Status Register                              */
    volatile       uint32_t CDC_IRQ_CLEAR1;               /*!< (@ 0x0000006C) Secondary IRQ Clear Register                               */
    volatile       uint32_t CDC_SLINE_IRQ_POS;            /*!< (@ 0x00000070) Secure Line IRQ Position Control Register                  */
    volatile const uint32_t RESERVED2;
    volatile       uint32_t CDC_CRC_REF;                  /*!< (@ 0x00000078) CRC Reference Register                                     */
    volatile const uint32_t CDC_CRC_RES;                  /*!< (@ 0x0000007C) CRC Result Register                                        */
    volatile       uint32_t CDC_ROT_BUFF_ADDR0;           /*!< (@ 0x00000080) Rotation Buffer 0 Address Register                         */
    volatile       uint32_t CDC_ROT_BUFF_ADDR1;           /*!< (@ 0x00000084) Rotation Buffer 1 Address Register                         */
    volatile       uint32_t CDC_ROT_BUFF_PITCH;           /*!< (@ 0x00000088) Rotation Buffer Pitch Register                             */
    volatile       uint32_t CDC_ROT_BLEND_COLOR;          /*!< (@ 0x0000008C) Rotation Blend Color Register                              */
    volatile const uint32_t RESERVED3[28];
    volatile const uint32_t CDC_L1_CFG1;                  /*!< (@ 0x00000100) Layer (n) Configuration 1 Register                         */
    volatile const uint32_t CDC_L1_CFG2;                  /*!< (@ 0x00000104) Layer (n) Configuration 2 Register                         */
    volatile       uint32_t CDC_L1_REL_CTRL;              /*!< (@ 0x00000108) Layer (n) Shadow Reload Control Register                   */
    volatile       uint32_t CDC_L1_CTRL;                  /*!< (@ 0x0000010C) Layer (n) Control Register                                 */
    volatile       uint32_t CDC_L1_WIN_HPOS;              /*!< (@ 0x00000110) Layer (n) Window Horizontal Position Register              */
    volatile       uint32_t CDC_L1_WIN_VPOS;              /*!< (@ 0x00000114) Layer (n) Window Vertical Position Register                */
    volatile       uint32_t CDC_L1_CKEY;                  /*!< (@ 0x00000118) Layer (n) Color Key Register                               */
    volatile       uint32_t CDC_L1_PIX_FORMAT;            /*!< (@ 0x0000011C) Layer (n) Pixel Format Register                            */
    volatile       uint32_t CDC_L1_CONST_ALPHA;           /*!< (@ 0x00000120) Layer (n) Constant Alpha Register                          */
    volatile       uint32_t CDC_L1_DFLT_COLOR;            /*!< (@ 0x00000124) Layer (n) Default Color Register                           */
    volatile       uint32_t CDC_L1_BLEND_CFG;             /*!< (@ 0x00000128) Layer (n) Blending Factors Register                        */
    volatile       uint32_t CDC_L1_FB_BCTRL;              /*!< (@ 0x0000012C) Layer (n) FB Bus Control Register                          */
    volatile const uint32_t RESERVED4;
    volatile       uint32_t CDC_L1_CFB_ADDR;              /*!< (@ 0x00000134) Layer (n) Color FB Address Register                        */
    volatile       uint32_t CDC_L1_CFB_LENGTH;            /*!< (@ 0x00000138) Layer (n) Color FB Length Register                         */
    volatile       uint32_t CDC_L1_CFB_LINES;             /*!< (@ 0x0000013C) Layer (n) Color FB Lines Register                          */
    volatile const uint32_t RESERVED5[4];
    volatile       uint32_t CDC_L1_CLUT_WRACC;            /*!< (@ 0x00000150) Layer (n) CLUT Write Access Register                       */
    volatile       uint32_t CDC_L1_SCALER_INSIZE;         /*!< (@ 0x00000154) Layer (n) Scaler Input Size Register                       */
    volatile       uint32_t CDC_L1_SCALER_OUTSIZE;        /*!< (@ 0x00000158) Layer (n) Scaler Output Size Register                      */
    volatile       uint32_t CDC_L1_VSCALE_FACTOR;         /*!< (@ 0x0000015C) Layer (n) Vertical Scaling Factor Register                 */
    volatile       uint32_t CDC_L1_VSCALE_PHASE;          /*!< (@ 0x00000160) Layer (n) Vertical Scaling Phase Register                  */
    volatile       uint32_t CDC_L1_HSCALE_FACTOR;         /*!< (@ 0x00000164) Layer (n) Horizontal Scaling Factor Register               */
    volatile       uint32_t CDC_L1_HSCALE_PHASE;          /*!< (@ 0x00000168) Layer (n) Horizontal Scaling Phase Register                */
    volatile const uint32_t RESERVED6[37];
    volatile const uint32_t CDC_L2_CFG1;                  /*!< (@ 0x00000200) Layer (n) Configuration 1 Register                         */
    volatile const uint32_t CDC_L2_CFG2;                  /*!< (@ 0x00000204) Layer (n) Configuration 2 Register                         */
    volatile       uint32_t CDC_L2_REL_CTRL;              /*!< (@ 0x00000208) Layer (n) Shadow Reload Control Register                   */
    volatile       uint32_t CDC_L2_CTRL;                  /*!< (@ 0x0000020C) Layer (n) Control Register                                 */
    volatile       uint32_t CDC_L2_WIN_HPOS;              /*!< (@ 0x00000210) Layer (n) Window Horizontal Position Register              */
    volatile       uint32_t CDC_L2_WIN_VPOS;              /*!< (@ 0x00000214) Layer (n) Window Vertical Position Register                */
    volatile       uint32_t CDC_L2_CKEY;                  /*!< (@ 0x00000218) Layer (n) Color Key Register                               */
    volatile       uint32_t CDC_L2_PIX_FORMAT;            /*!< (@ 0x0000021C) Layer (n) Pixel Format Register                            */
    volatile       uint32_t CDC_L2_CONST_ALPHA;           /*!< (@ 0x00000220) Layer (n) Constant Alpha Register                          */
    volatile       uint32_t CDC_L2_DFLT_COLOR;            /*!< (@ 0x00000224) Layer (n) Default Color Register                           */
    volatile       uint32_t CDC_L2_BLEND_CFG;             /*!< (@ 0x00000228) Layer (n) Blending Factors Register                        */
    volatile       uint32_t CDC_L2_FB_BCTRL;              /*!< (@ 0x0000022C) Layer (n) FB Bus Control Register                          */
    volatile const uint32_t RESERVED7;
    volatile       uint32_t CDC_L2_CFB_ADDR;              /*!< (@ 0x00000234) Layer (n) Color FB Address Register                        */
    volatile       uint32_t CDC_L2_CFB_LENGTH;            /*!< (@ 0x00000238) Layer (n) Color FB Length Register                         */
    volatile       uint32_t CDC_L2_CFB_LINES;             /*!< (@ 0x0000023C) Layer (n) Color FB Lines Register                          */
    volatile const uint32_t RESERVED8[4];
    volatile       uint32_t CDC_L2_CLUT_WRACC;            /*!< (@ 0x00000250) Layer (n) CLUT Write Access Register                       */
    volatile       uint32_t CDC_L2_SCALER_INSIZE;         /*!< (@ 0x00000254) Layer (n) Scaler Input Size Register                       */
    volatile       uint32_t CDC_L2_SCALER_OUTSIZE;        /*!< (@ 0x00000258) Layer (n) Scaler Output Size Register                      */
    volatile       uint32_t CDC_L2_VSCALE_FACTOR;         /*!< (@ 0x0000025C) Layer (n) Vertical Scaling Factor Register                 */
    volatile       uint32_t CDC_L2_VSCALE_PHASE;          /*!< (@ 0x00000260) Layer (n) Vertical Scaling Phase Register                  */
    volatile       uint32_t CDC_L2_HSCALE_FACTOR;         /*!< (@ 0x00000264) Layer (n) Horizontal Scaling Factor Register               */
    volatile       uint32_t CDC_L2_HSCALE_PHASE;          /*!< (@ 0x00000268) Layer (n) Horizontal Scaling Phase Register                */
    volatile const uint32_t RESERVED9[37];
    volatile const uint32_t CDC_L3_CFG1;                  /*!< (@ 0x00000300) Layer (n) Configuration 1 Register                         */
    volatile const uint32_t CDC_L3_CFG2;                  /*!< (@ 0x00000304) Layer (n) Configuration 2 Register                         */
    volatile       uint32_t CDC_L3_REL_CTRL;              /*!< (@ 0x00000308) Layer (n) Shadow Reload Control Register                   */
    volatile       uint32_t CDC_L3_CTRL;                  /*!< (@ 0x0000030C) Layer (n) Control Register                                 */
    volatile       uint32_t CDC_L3_WIN_HPOS;              /*!< (@ 0x00000310) Layer (n) Window Horizontal Position Register              */
    volatile       uint32_t CDC_L3_WIN_VPOS;              /*!< (@ 0x00000314) Layer (n) Window Vertical Position Register                */
    volatile       uint32_t CDC_L3_CKEY;                  /*!< (@ 0x00000318) Layer (n) Color Key Register                               */
    volatile       uint32_t CDC_L3_PIX_FORMAT;            /*!< (@ 0x0000031C) Layer (n) Pixel Format Register                            */
    volatile       uint32_t CDC_L3_CONST_ALPHA;           /*!< (@ 0x00000320) Layer (n) Constant Alpha Register                          */
    volatile       uint32_t CDC_L3_DFLT_COLOR;            /*!< (@ 0x00000324) Layer (n) Default Color Register                           */
    volatile       uint32_t CDC_L3_BLEND_CFG;             /*!< (@ 0x00000328) Layer (n) Blending Factors Register                        */
    volatile       uint32_t CDC_L3_FB_BCTRL;              /*!< (@ 0x0000032C) Layer (n) FB Bus Control Register                          */
    volatile const uint32_t RESERVED10;
    volatile       uint32_t CDC_L3_CFB_ADDR;              /*!< (@ 0x00000334) Layer (n) Color FB Address Register                        */
    volatile       uint32_t CDC_L3_CFB_LENGTH;            /*!< (@ 0x00000338) Layer (n) Color FB Length Register                         */
    volatile       uint32_t CDC_L3_CFB_LINES;             /*!< (@ 0x0000033C) Layer (n) Color FB Lines Register                          */
    volatile const uint32_t RESERVED11[4];
    volatile       uint32_t CDC_L3_CLUT_WRACC;            /*!< (@ 0x00000350) Layer (n) CLUT Write Access Register                       */
    volatile       uint32_t CDC_L3_SCALER_INSIZE;         /*!< (@ 0x00000354) Layer (n) Scaler Input Size Register                       */
    volatile       uint32_t CDC_L3_SCALER_OUTSIZE;        /*!< (@ 0x00000358) Layer (n) Scaler Output Size Register                      */
    volatile       uint32_t CDC_L3_VSCALE_FACTOR;         /*!< (@ 0x0000035C) Layer (n) Vertical Scaling Factor Register                 */
    volatile       uint32_t CDC_L3_VSCALE_PHASE;          /*!< (@ 0x00000360) Layer (n) Vertical Scaling Phase Register                  */
    volatile       uint32_t CDC_L3_HSCALE_FACTOR;         /*!< (@ 0x00000364) Layer (n) Horizontal Scaling Factor Register               */
    volatile       uint32_t CDC_L3_HSCALE_PHASE;          /*!< (@ 0x00000368) Layer (n) Horizontal Scaling Phase Register                */
} CDC_Type;

/*CDC configurations*/

/* The Bus width value 7 relates to the bus width (8 bytes in this configuration)
 * and serves to correctly calculate the address of the last word of data for
 * that line internally
 */
#define BUS_WIDTH                        (7)

/* Alpha constant is a fixed value that is used to represent the opacity of one of
 * the layers being blended. The alpha constant is typically used when the opacity
 * of one layer is known or fixed and does not vary across the image.
 */
#define ALPHA_CONSTANT                   (0x000000FF)

/*CDC Register Descriptions*/

/*Global Enable bit parameters*/
#define CDC_GLOBAL_EN                    0U  /**< CDC global enable*/
#define CDC_GLOBAL_EN_MASK               (1U << CDC_GLOBAL_EN)

/*Shadow Reload Control bit parameters*/
#define CDC_IMMEDIATE_RELOAD             0U  /**< CDC immediate reload */
#define CDC_IMMEDIATE_RELOAD_MASK        (1U << CDC_IMMEDIATE_RELOAD)

/*Sync/Blank Status bit parameters*/
#define CDC_VBLANK_STATUS                0U  /**< CDC vblank status */
#define CDC_VBLANK_STATUS_MASK           (1U << CDC_VBLANK_STATUS)
#define CDC_HBLANK_STATUS                1U  /**< CDC hblank status */
#define CDC_HBLANK_STATUS_MASK           (1U << CDC_HBLANK_STATUS)
#define CDC_VSYNC_STATUS                 2U  /**< CDC vsync status */
#define CDC_VSYNC_STATUS_MASK            (1U << CDC_VSYNC_STATUS)
#define CDC_HSYNC_STATUS                 3U  /**< CDC hsync status */
#define CDC_HSYNC_STATUS_MASK            (1U << CDC_HSYNC_STATUS)

/*Layer control bit parameters*/
#define CDC_LAYER_ON                     0U  /**< CDC layer On */
#define CDC_LAYER_ON_MASK                (1U << CDC_LAYER_ON)

/*CDC_IRQ control bit parameters*/
#define CDC_IRQ_LINE                     (1U << 0)   /**< Line IRQ */
#define CDC_IRQ_FIFO_UNDERRUN_WARNING    (1U << 1)   /**< FIFO Underrun Warning IRQ */
#define CDC_IRQ_BUS_ERROR                (1U << 2)   /**< Bus error IRQ */
#define CDC_IRQ_REGISTER_RELOAD          (1U << 3)   /**< Register reload IRQ */
#define CDC_IRQ_SLAVE_TIMING_NO_SIGNAL   (1U << 4)   /**< Slave timing no signal IRQ */
#define CDC_IRQ_SLAVE_TIMING_NOT_IN_SYNC (1U << 5)   /**< Slave timing not in sync IRQ */
#define CDC_IRQ_FIFO_UNDERRUN_KILLING    (1U << 6)   /**< FIFO underrun killing IRQ */
#define CDC_IRQ_CRC                      (1U << 7)   /**< CRC IRQ */
#define CDC_IRQ_ROTATION_FIFO_ERROR      (1U << 8)   /**< Rotation FIFO error interrupt */

/**
 * enum  CDC_PIXEL_FORMAT
 * CDC Supported Pixel format.
 */
typedef enum _CDC_PIXEL_FORMAT
{
    CDC_PIXEL_FORMAT_ARGB8888,                /**< 32-bit ARGB */
    CDC_PIXEL_FORMAT_RGB888,                  /**< 24-bit RGB (A = 255) */
    CDC_PIXEL_FORMAT_RGB565,                  /**< 16-bit RGB (A = 255) */
    CDC_PIXEL_FORMAT_RGBA8888,                /**< 32-bit RGBA */
    CDC_PIXEL_FORMAT_AL44,                    /**< 8-bit alpha + luminance (lower channel on R, G and B) */
    CDC_PIXEL_FORMAT_AL8,                     /**< 8-bit single channel (value on A, R, G and B) */
    CDC_PIXEL_FORMAT_ARGB1555,                /**< 16-bit ARGB with 1 bit alpha */
    CDC_PIXEL_FORMAT_ARGB4444                 /**< 16-bit ARGB with 4 bits alpha */
} CDC_PIXEL_FORMAT;

/**
 * enum Layers supported
 * CDC layers supported
 */
typedef enum _CDC_LAYER
{
    CDC_LAYER_1,            /**< cdc layer 1 */
    CDC_LAYER_2,            /**< cdc layer 2 */
    CDC_LAYER_3             /**< cdc layer 3 */
}CDC_LAYER;

/**
  \fn          static inline uint32_t cdc_get_layer_count(CDC_Type *cdc)
  \brief       Get the cdc layer count.
  \param[in]   cdc   Pointer to the cdc register map.
  \return      layer count.
*/
static inline uint32_t cdc_get_layer_count(CDC_Type *cdc)
{
    return cdc->CDC_LCNT;
}

/**
  \fn          static inline void cdc_set_sync_size(CDC_Type *cdc, uint32_t sync_size)
  \brief       Set the cdc sync size.
  \param[in]   cdc         Pointer to the cdc register map.
  \param[in]   sync_size   sync size value to set.
               sync_size[31..16] H Sync width-1 (pixels).
               sync_size[15..0]  V Sync height-1 (lines).
  \return      none.
*/
static inline void cdc_set_sync_size(CDC_Type *cdc, uint32_t sync_size)
{
    cdc->CDC_SYNC_SIZE_CFG = sync_size;
}

/**
  \fn          static inline uint32_t cdc_get_sync_size(CDC_Type *cdc)
  \brief       Get the cdc sync size.
  \param[in]   cdc     Pointer to the cdc register map.
  \return      current sync size value.
               bit[31..16] H Sync width-1 (pixels).
               bit[15..0]  V Sync height-1 (lines).
*/
static inline uint32_t cdc_get_sync_size(CDC_Type *cdc)
{
    return cdc->CDC_SYNC_SIZE_CFG;
}

/**
  \fn          static inline void cdc_set_back_porch(CDC_Type *cdc, uint32_t back_porch)
  \brief       Set the cdc back porch.
  \param[in]   cdc          Pointer to the cdc register map.
  \param[in]   back_porch   back porch value to set.
               back_porch[31..16] Accumulated width including sync and back porch - 1(pixels).
               back_porch[15..0]  Accumulated height including sync and back porch -1(lines).
  \return      none.
*/
static inline void cdc_set_back_porch(CDC_Type *cdc, uint32_t back_porch)
{
    cdc->CDC_BP_CFG = back_porch;
}

/**
  \fn          static inline uint32_t cdc_get_back_porch(CDC_Type *cdc)
  \brief       Get the cdc back porch.
  \param[in]   cdc     Pointer to the cdc register map.
  \return      current back porch value.
               bit[31..16] Accumulated width including sync and back porch - 1(pixels).
               bit[15..0]  Accumulated height including sync and back porch -1(lines).
*/
static inline uint32_t cdc_get_back_porch(CDC_Type *cdc)
{
    return cdc->CDC_BP_CFG;
}

/**
  \fn          static inline void cdc_set_active_width(CDC_Type *cdc, uint32_t active_width)
  \brief       Set the cdc active width.
  \param[in]   cdc            Pointer to the cdc register map.
  \param[in]   active_width   active width value to set.
               active_width[31..16] Accumulated width including sync, back porch and active width -1 (pixels).
               active_width[15..0]  Accumulated height including sync, back porch and active height -1 (lines).
  \return      none.
*/
static inline void cdc_set_active_width(CDC_Type *cdc, uint32_t active_width)
{
    cdc->CDC_ACTW_CFG = active_width;
}

/**
  \fn          static inline uint32_t  cdc_get_active_width(CDC_Type *cdc)
  \brief       Get the cdc active width.
  \param[in]   cdc      Pointer to the cdc register map.
  \return      current active width value.
               bit[31..16] Accumulated width including sync, back porch and active width -1 (pixels).
               bit[15..0]  Accumulated height including sync, back porch and active height -1 (lines).
*/
static inline uint32_t  cdc_get_active_width(CDC_Type *cdc)
{
    return cdc->CDC_ACTW_CFG;
}

/**
  \fn          static inline void cdc_set_total_width(CDC_Type *cdc, uint32_t total_width)
  \brief       Set the cdc total width.
  \param[in]   cdc           Pointer to the cdc register map.
  \param[in]   total_width   total width value to set.
               total_width[31..16] Total width, including sync, back porch, active width and front porch -1 (pixels).
               total_width[15..0]  Total height, including sync, back porch, active height and front porch -1 (lines).
  \return      none.
*/
static inline void cdc_set_total_width(CDC_Type *cdc, uint32_t total_width)
{
    cdc->CDC_TOTALW_CFG = total_width;
}

/**
  \fn          static inline uint32_t  cdc_get_active_width(CDC_Type *cdc)
  \brief       Get the cdc total width.
  \param[in]   cdc     Pointer to the cdc register map.
  \return      current total width value.
               bit[31..16] Total width, including sync, back porch, active width and front porch -1 (pixels).
               bit[15..0]  Total height, including sync, back porch, active height and front porch -1 (lines).
*/
static inline uint32_t  cdc_get_total_width(CDC_Type *cdc)
{
    return cdc->CDC_TOTALW_CFG;
}


/**
  \fn          static inline void cdc_global_enable(CDC_Type *cdc)
  \brief       cdc global enable.
  \param[in]   cdc     Pointer to the cdc register map.
  \return      none.
*/
static inline void cdc_global_enable(CDC_Type *cdc)
{
    cdc->CDC_GLB_CTRL |= CDC_GLOBAL_EN_MASK;
}

/**
  \fn          static inline void cdc_global_disable(CDC_Type *cdc)
  \brief       cdc global disable.
  \param[in]   cdc      Pointer to the cdc register map.
  \return      none.
*/
static inline void cdc_global_disable(CDC_Type *cdc)
{
    cdc->CDC_GLB_CTRL &= ~CDC_GLOBAL_EN_MASK;
}

/**
  \fn          static inline void cdc_shadow_reload(CDC_Type *cdc)
  \brief       cdc global shadow reload.
  \param[in]   cdc      Pointer to the cdc register map.
  \return      none.
*/
static inline void cdc_shadow_reload(CDC_Type *cdc)
{
    cdc->CDC_SRCTRL |= CDC_IMMEDIATE_RELOAD_MASK;
}

/**
  \fn          static inline void cdc_irq_enable(CDC_Type *cdc, uint32_t irqs)
  \brief       Enable cdc interrupts.
  \param[in]   cdc     Pointer to the cdc register map.
  \param[in]   irqs    cdc interrupts to enable refer bitmask CDC_IRQ_* macros.
  \return      none.
*/
static inline void cdc_irq_enable(CDC_Type *cdc, uint32_t irqs)
{
    cdc->CDC_IRQ_MASK0 |= irqs;
}

/**
  \fn          static inline void cdc_irq_disable(CDC_Type *cdc, uint32_t irqs)
  \brief       Disable cdc interrupts.
  \param[in]   cdc      Pointer to the cdc register map.
  \param[in]   irqs     cdc interrupts to disable refer bitmask CDC_IRQ_* macros.
  \return      none.
*/
static inline void cdc_irq_disable(CDC_Type *cdc, uint32_t irqs)
{
    cdc->CDC_IRQ_MASK0 &= ~irqs;
}

/**
  \fn          static inline void cdc_irq_clear(CDC_Type *cdc, uint32_t irqs)
  \brief       Clear cdc interrupts.
  \param[in]   cdc      Pointer to the cdc register map.
  \param[in]   irqs     cdc interrupts to clear refer bitmask CDC_IRQ_* macros.
  \return      none.
*/
static inline void cdc_irq_clear(CDC_Type *cdc, uint32_t irqs)
{
    cdc->CDC_IRQ_CLEAR0 |= irqs;
}

/**
  \fn          static inline uint32_t cdc_get_irq_clear(CDC_Type *cdc)
  \brief       Get cdc irq clear reg value.
  \param[in]   cdc      Pointer to the cdc register map.
  \return      current cdc irq clear reg value refer bitmask CDC_IRQ_* macros.
*/
static inline uint32_t cdc_get_irq_clear(CDC_Type *cdc)
{
    return cdc->CDC_IRQ_CLEAR0;
}

/**
  \fn          static inline uint32_t cdc_get_irq_status(CDC_Type *cdc)
  \brief       Get the cdc irq status.
  \param[in]   cdc     Pointer to the cdc register map.
  \return      current irq status value refer bitmask CDC_IRQ_* macros.
*/
static inline uint32_t cdc_get_irq_status(CDC_Type *cdc)
{
    return cdc->CDC_IRQ_STATUS0;
}

/**
  \fn          static inline void cdc_set_line_irq_position(CDC_Type *cdc, uint16_t line_pos)
  \brief       Set the cdc line irq position.
  \param[in]   cdc        Pointer to the cdc register map.
  \param[in]   line_pos   line irq position value to set.
               bit[15..0] Line IRQ position.
  \return      none.
*/
static inline void cdc_set_line_irq_position(CDC_Type *cdc, uint16_t line_pos)
{
    cdc->CDC_LINE_IRQ_POS = line_pos;
}

/**
  \fn          static inline uint32_t cdc_get_position_status(CDC_Type *cdc)
  \brief       Get the cdc current X and Y position.
  \param[in]   cdc     Pointer to the cdc register map.
  \return      current X and Y position value.
               bit[31..16] Current X position.
               bit[15..0] Current Y position.
*/
static inline uint32_t cdc_get_position_status(CDC_Type *cdc)
{
    return cdc->CDC_POS_STAT;
}

/**
  \fn          static inline uint32_t cdc_get_vblank_status(CDC_Type *cdc)
  \brief       Get the cdc current vblank status.
  \param[in]   cdc     Pointer to the cdc register map.
  \return      current vblank status value.
*/
static inline uint32_t cdc_get_vblank_status(CDC_Type *cdc)
{
    return (((cdc->CDC_SYNC_BLANK_STAT) & CDC_VBLANK_STATUS_MASK) >> CDC_VBLANK_STATUS);
}

/**
  \fn          static inline uint32_t cdc_get_hblank_status(CDC_Type *cdc)
  \brief       Get the cdc current hblank status.
  \param[in]   cdc      Pointer to the cdc register map.
  \return      current hblank status value.
*/
static inline uint32_t cdc_get_hblank_status(CDC_Type *cdc)
{
    return (((cdc->CDC_SYNC_BLANK_STAT) & CDC_HBLANK_STATUS_MASK) >> CDC_HBLANK_STATUS);
}

/**
  \fn          static inline uint32_t cdc_get_vsync_status(CDC_Type *cdc)
  \brief       Get the cdc current vsync status.
  \param[in]   cdc      Pointer to the cdc register map.
  \return      current vsync status value.
*/
static inline uint32_t cdc_get_vsync_status(CDC_Type *cdc)
{
    return (((cdc->CDC_SYNC_BLANK_STAT) & CDC_VSYNC_STATUS_MASK) >> CDC_VSYNC_STATUS);
}

/**
  \fn          static inline uint32_t cdc_get_hsync_status(CDC_Type *cdc)
  \brief       Get the cdc current hsync status.
  \param[in]   cdc     Pointer to the cdc register map.
  \return      current hsync status value.
*/
static inline uint32_t cdc_get_hsync_status(CDC_Type *cdc)
{
    return (((cdc->CDC_SYNC_BLANK_STAT) & CDC_HSYNC_STATUS_MASK) >> CDC_HSYNC_STATUS);
}

/**
  \fn          void cdc_layer_on(CDC_Type *cdc, CDC_LAYER layer)
  \brief       cdc layer on.
  \param[in]   cdc      Pointer to the cdc register map.
  \param[in]   layer    cdc layer to on.
  \return      none.
*/
void cdc_layer_on(CDC_Type *cdc, CDC_LAYER layer);

/**
  \fn          void cdc_layer_off(CDC_Type *cdc, CDC_LAYER layer)
  \brief       cdc layer off.
  \param[in]   cdc     Pointer to the cdc register map.
  \param[in]   layer   cdc layer to off.
  \return      none.
*/
void cdc_layer_off(CDC_Type *cdc, CDC_LAYER layer);

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
void cdc_set_win_hpos(CDC_Type *cdc, CDC_LAYER layer, uint32_t hpos);

/**
  \fn          uint32_t cdc_get_win_hpos(CDC_Type *cdc, CDC_LAYER layer)
  \brief       Get cdc window Horizontal Position.
  \param[in]   cdc     Pointer to the cdc register map.
  \param[in]   layer   cdc layer to get window Horizontal Position.
  \return      current window Horizontal Position value.
               bit[31..16] Horizontal Stop Position.
               bit[15..0]  Horizontal Start Position.
*/
uint32_t cdc_get_win_hpos(CDC_Type *cdc, CDC_LAYER layer);

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
void cdc_set_win_vpos(CDC_Type *cdc, CDC_LAYER layer, uint32_t vpos);

/**
  \fn          uint32_t cdc_get_win_vpos(CDC_Type *cdc, CDC_LAYER layer)
  \brief       Get cdc window Vertical Position.
  \param[in]   cdc     Pointer to the cdc register map.
  \param[in]   layer   cdc layer to get window Vertical Position.
  \return      current window Vertical Position value.
               bit[31..16] Vertical Stop Position.
               bit[15..0]  Vertical Start Position.
*/
uint32_t cdc_get_win_vpos(CDC_Type *cdc, CDC_LAYER layer);

/**
  \fn          void cdc_set_pixel_format(CDC_Type *cdc, CDC_LAYER layer, CDC_PIXEL_FORMAT pixel_format)
  \brief       Set cdc pixel format.
  \param[in]   cdc            Pointer to the cdc register map.
  \param[in]   layer          cdc layer to set pixel format.
  \param[in]   pixel_format   pixel format to set.
               bit[2..0] Pixel format.
  \return      none.
*/
void cdc_set_pixel_format(CDC_Type *cdc, CDC_LAYER layer, CDC_PIXEL_FORMAT pixel_format);

/**
  \fn          CDC_PIXEL_FORMAT cdc_get_pixel_format(CDC_Type *cdc, CDC_LAYER layer)
  \brief       Get cdc pixel format.
  \param[in]   cdc     Pointer to the cdc register map.
  \param[in]   layer   cdc layer to get pixel format.
  \return      current pixel format value.
               bit[2..0] Pixel format.
*/
CDC_PIXEL_FORMAT cdc_get_pixel_format(CDC_Type *cdc, CDC_LAYER layer);

/**
  \fn          void cdc_set_constant_alpha(CDC_Type *cdc, CDC_LAYER layer, uint32_t alpha_constant)
  \brief       Set cdc alpha constant.
  \param[in]   cdc              Pointer to the cdc register map.
  \param[in]   layer            cdc layer to set alpha constant.
  \param[in]   alpha_constant   alpha constant to set.
               bit[7..0] Constant alpha.
  \return      none.
*/
void cdc_set_constant_alpha(CDC_Type *cdc, CDC_LAYER layer, uint32_t alpha_constant);

/**
  \fn          uint32_t cdc_get_constant_alpha(CDC_Type *cdc, CDC_LAYER layer)
  \brief       Get cdc alpha constant.
  \param[in]   cdc     Pointer to the cdc register map.
  \param[in]   layer   cdc layer to get alpha constant.
  \return      current alpha constant value.
               bit[7..0] Constant alpha.
*/
uint32_t cdc_get_constant_alpha(CDC_Type *cdc, CDC_LAYER layer);

/**
  \fn          void cdc_set_fb_address(CDC_Type *cdc, CDC_LAYER layer, uint32_t fb)
  \brief       Set cdc frame buffer address.
  \param[in]   cdc     Pointer to the cdc register map.
  \param[in]   layer   cdc layer to set frame buffer address.
  \param[in]   addr    frame buffer address to set.
  \return      none.
*/
void cdc_set_fb_address(CDC_Type *cdc, CDC_LAYER layer, uint32_t addr);

/**
  \fn          uint32_t  cdc_get_fb_address(CDC_Type *cdc, CDC_LAYER layer)
  \brief       Get cdc frame buffer address.
  \param[in]   cdc     Pointer to the cdc register map.
  \param[in]   layer   cdc layer to get frame buffer address.
  \return      current frame buffer address value.
*/
uint32_t  cdc_get_fb_address(CDC_Type *cdc, CDC_LAYER layer);

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
void cdc_set_fb_length(CDC_Type *cdc, CDC_LAYER layer, uint32_t fb_length);

/**
  \fn          uint32_t  cdc_get_fb_length(CDC_Type *cdc, CDC_LAYER layer)
  \brief       Get cdc frame buffer length.
  \param[in]   cdc     Pointer to the cdc register map.
  \param[in]   layer   cdc layer to get frame buffer length.
  \return      current frame buffer length value.
               bit[31..16] Pitch of color FB in bytes (signed).
               bit[15..0]  Line length of color FB in bytes – 1 + (width of bus in bytes).
*/
uint32_t  cdc_get_fb_length(CDC_Type *cdc, CDC_LAYER layer);

/**
  \fn          void cdc_set_fb_lines(CDC_Type *cdc, CDC_LAYER layer, uint32_t fb_lines)
  \brief       Set cdc frame buffer lines.
  \param[in]   cdc      Pointer to the cdc register map.
  \param[in]   layer    cdc layer to set frame buffer lines.
  \param[in]   fb_lines frame buffer lines to set.
               bit[15..0] Number of lines in the color FB.
  \return      none.
*/
void cdc_set_fb_lines(CDC_Type *cdc, CDC_LAYER layer, uint32_t fb_lines);

/**
  \fn          uint32_t  cdc_get_fb_lines(CDC_Type *cdc, CDC_LAYER layer)
  \brief       Get cdc frame buffer lines.
  \param[in]   cdc      Pointer to the cdc register map.
  \param[in]   layer    cdc layer to get frame buffer lines.
  \return      current frame buffer lines value.
               bit[15..0] Number of lines in the color FB.
*/
uint32_t  cdc_get_fb_lines(CDC_Type *cdc, CDC_LAYER layer);

#ifdef __cplusplus
}
#endif
#endif /* CDC_H_ */
