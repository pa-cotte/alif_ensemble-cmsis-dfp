/* Copyright (C) 2022 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/**************************************************************************//**
 * @file     Driver_CDC200.c
 * @author   Girish BN and Prasanna Ravi
 * @email    girish.bn@alifsemi.com and prasanna.ravi@alifsemi.com
 * @version  V1.0.0
 * @date     30-Sep-2021
 * @brief    Display controller CDC200 driver source file.
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

#include "Driver_CDC200.h"
#include "Driver_CDC_Private.h"
#include "sys_ctrl_cdc.h"
#include "cdc.h"
#include "system_utils.h"
#include "RTE_Device.h"
#include "display.h"

#define ARM_CDC200_DRV_VERSION    ARM_DRIVER_VERSION_MAJOR_MINOR(1, 0) /*driver version*/

#if !(RTE_CDC200)
#error "CDC200 is not enabled in the RTE_Device.h"
#endif

#if (!defined(RTE_Drivers_CDC200))
#error "CDC200 not configured in RTE_Components.h!"
#endif

/*Error checking on RTE parameters*/
#if ((RTE_PANEL_HSYNC_TIME - 1) < 0) || ((RTE_PANEL_HSYNC_TIME - 1) > 0xFFFF)
#error "CDC200 error on horizontal sync size"
#endif

#if ((RTE_PANEL_VSYNC_LINE - 1) < 0) || ((RTE_PANEL_VSYNC_LINE - 1) > 0xFFFF)
#error "CDC200 error on vertical sync size"
#endif

#if ((RTE_PANEL_HSYNC_TIME + RTE_PANEL_HBP_TIME - 1) < 0) || ((RTE_PANEL_HSYNC_TIME + RTE_PANEL_HBP_TIME - 1) > 0xFFFF)
#error "CDC200 error on horizontal back porch"
#endif

#if ((RTE_PANEL_VSYNC_LINE + RTE_PANEL_VBP_LINE - 1) < 0) || ((RTE_PANEL_VSYNC_LINE + RTE_PANEL_VBP_LINE - 1) > 0xFFFF)
#error "CDC200 error on vertical back porch"
#endif

#if ((RTE_PANEL_HSYNC_TIME + RTE_PANEL_HBP_TIME + RTE_PANEL_HACTIVE_TIME - 1) < 0) \
                           || ((RTE_PANEL_HSYNC_TIME + RTE_PANEL_HBP_TIME + RTE_PANEL_HACTIVE_TIME - 1) > 0xFFFF)
#error "CDC200 error on horizontal active width"
#endif

#if ((RTE_PANEL_VSYNC_LINE + RTE_PANEL_VBP_LINE + RTE_PANEL_VACTIVE_LINE - 1) < 0) \
                           || ((RTE_PANEL_VSYNC_LINE + RTE_PANEL_VBP_LINE + RTE_PANEL_VACTIVE_LINE - 1) > 0xFFFF)
#error "CDC200 error on vertical active width"
#endif

#if ((RTE_PANEL_HSYNC_TIME + RTE_PANEL_HBP_TIME + RTE_PANEL_HACTIVE_TIME + RTE_PANEL_HFP_TIME - 1) < 0) \
                           || ((RTE_PANEL_HSYNC_TIME + RTE_PANEL_HBP_TIME + RTE_PANEL_HACTIVE_TIME + RTE_PANEL_HFP_TIME - 1) > 0xFFFF)
#error "CDC200 error on horizontal total width"
#endif

#if ((RTE_PANEL_VSYNC_LINE + RTE_PANEL_VBP_LINE + RTE_PANEL_VACTIVE_LINE + RTE_PANEL_VFP_LINE - 1) < 0) \
                           || ((RTE_PANEL_VSYNC_LINE + RTE_PANEL_VBP_LINE + RTE_PANEL_VACTIVE_LINE + RTE_PANEL_VFP_LINE - 1) > 0xFFFF)
#error "CDC200 error on vertical total width"
#endif

#if (RTE_MIPI_DSI)
#include "Driver_MIPI_DSI.h"
/*MIPI DSI driver instance*/
extern ARM_DRIVER_MIPI_DSI Driver_MIPI_DSI;
/*MIPI DSI driver callback*/
void MIPI_DSI_Event_Callback (uint32_t int_event);
#endif

/*Driver Version*/
static const ARM_DRIVER_VERSION DriverVersion =
{
    ARM_CDC200_API_VERSION,
    ARM_CDC200_DRV_VERSION
};

/* Driver Capabilities */
static const ARM_CDC200_CAPABILITIES DriverCapabilities =
{
    0, /* Not supports reentrant_operation */
    1, /* DPI interface supported */
    0  /* reserved (must be zero) */
};

//
//  Functions
//
/**
  \fn          ARM_DRIVER_VERSION CDC200_GetVersion (void)
  \brief       Get CDC200 driver version.
  \return      \ref ARM_DRIVER_VERSION.
*/
static ARM_DRIVER_VERSION CDC200_GetVersion (void)
{
    return DriverVersion;
}

/**
  \fn          ARM_CDC200_CAPABILITIES CDC200_GetCapabilities (void)
  \brief       Get CDC200 driver capabilities.
  \return      \ref ARM_CDC200_CAPABILITIES.
*/
static ARM_CDC200_CAPABILITIES CDC200_GetCapabilities (void)
{
    return DriverCapabilities;
}

/**
  \fn          static int32_t CDC200_Init (ARM_CDC200_SignalEvent_t cb_event, CDC_RESOURCES *cdc)
  \brief       Initialize CDC200 Interface.
  \param[in]   cb_event Pointer to ARM_CDC200_SignalEvent_t.
  \param[in]   cdc Pointer to CDC resources.
  \return      \ref execution_status.
*/
static int32_t CDC200_Init (ARM_CDC200_SignalEvent_t cb_event, CDC_RESOURCES *cdc)
{
    int32_t ret = ARM_DRIVER_OK;

    if (cdc->state.initialized == 1)
    {
        return ARM_DRIVER_OK;
    }

    if (!cb_event)
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    cdc->cb_event = cb_event;

#if (RTE_MIPI_DSI)
    /*Initializing MIPI DSI, if the LCD Panel is MIPI DSI LCD Panel*/
    ret = Driver_MIPI_DSI.Initialize(MIPI_DSI_Event_Callback);
    if (ret != ARM_DRIVER_OK)
    {
        return ret;
    }
#endif

    cdc->state.initialized = 1;

    return ret;
}

/**
  \fn          static int32_t CDC200_Uninit (CDC_RESOURCES *cdc)
  \brief       uninitialize CDC200 Interface.
  \param[in]   cdc Pointer to CDC resources.
  \return      \ref execution_status.
  */
static int32_t CDC200_Uninit (CDC_RESOURCES *cdc)
{
    int32_t ret = ARM_DRIVER_OK;
    cdc->cb_event = NULL;

    if (cdc->state.initialized == 0)
    {
        return ARM_DRIVER_OK;
    }

    if (cdc->state.powered == 1)
    {
        return ARM_DRIVER_ERROR;
    }

#if (RTE_MIPI_DSI)
    /*Uninitializing MIPI DSI, if the LCD Panel is MIPI DSI LCD Panel*/
    ret = Driver_MIPI_DSI.Uninitialize();
    if (ret != ARM_DRIVER_OK)
    {
        return ret;
    }

#endif

    cdc->state.initialized = 0;
    return ret;
}

/**
  \fn          static int32_t CDC200_PowerCtrl (ARM_POWER_STATE state, CDC_RESOURCES *cdc)
  \brief       Control CDC200 Interface Power.
  \param[in]   state Power state.
  \param[in]   cdc Pointer to CDC resources.
  \return      \ref execution_status.
  */
static int32_t CDC200_PowerCtrl (ARM_POWER_STATE state, CDC_RESOURCES *cdc)
{
    uint32_t pixclk, htotal, vtotal;
    int pixclk_div;
    int32_t ret = ARM_DRIVER_OK;

    if (cdc->state.initialized == 0)
    {
        return ARM_DRIVER_ERROR;
    }

    switch (state)
    {
        case ARM_POWER_OFF:

            if (cdc->state.powered == 0)
            {
                return ARM_DRIVER_OK;
            }

#if (RTE_MIPI_DSI)
            /*Disable MIPI DSI*/
            ret = Driver_MIPI_DSI.PowerControl(ARM_POWER_OFF);
            if (ret != ARM_DRIVER_OK)
            {
                return ret;
            }
#endif
            /*Disabling Line IRQ*/
            NVIC_DisableIRQ(CDC_SCANLINE0_IRQ_IRQn);
            NVIC_ClearPendingIRQ(CDC_SCANLINE0_IRQ_IRQn);

            /* Disabling pixel clock */
            clear_cdc_pixel_clk();

            /* Disabling Source clock */
            disable_dpi_periph_clk();

            cdc->state.powered = 0;
            break;

        case ARM_POWER_FULL:

            if (cdc->state.powered == 1)
            {
                return ARM_DRIVER_OK;
            }

#if (RTE_MIPI_DSI)
            /*Enable MIPI DSI*/
            ret = Driver_MIPI_DSI.PowerControl(ARM_POWER_FULL);
            if (ret != ARM_DRIVER_OK)
            {
                return ret;
            }
#endif

            /* LCD Manufacturer provides the Frame timing values
             *     HTOTAL = WIDTH + HSYNC + HFP + HBP
             *     VTOTAL = HEIGHT + VSYNC + VFP + VBP
             * Calculate the pixel clock for DPI controller
             *     PIXCLK = FPS x HTOTAL x VTOTAL
             * Calculate the pixel clock divider
             *     PIXCLK_DIV = CDC200_PIXCLK_SOURCE / PIXCLK
             */
            htotal = (cdc->frame_info->hsync_time
                      + cdc->frame_info->hbp_time
                      + cdc->frame_info->hfp_time
                      + cdc->frame_info->hactive_time);

            vtotal = (cdc->frame_info->vsync_line
                      + cdc->frame_info->vbp_line
                      + cdc->frame_info->vfp_line
                      + cdc->frame_info->vactive_line);

            pixclk = (htotal * vtotal * RTE_CDC200_DPI_FPS);

            pixclk_div = (int) ((float)CDC200_PIXCLK / pixclk + 0.5f);

            /*Checking clk divider is less than 2 because 0 and 1 are illegal value*/
            if (pixclk_div < 2 || pixclk_div > 511)
            {
                return ARM_DRIVER_ERROR_PARAMETER;
            }

            /* Enabling Source clock */
            enable_dpi_periph_clk();

            /* Configuring pixel clock */
            set_cdc_pixel_clk(CDC_PIX_CLKSEL_400MZ, pixclk_div);

            /*Enabling Line IRQ*/
            NVIC_ClearPendingIRQ(CDC_SCANLINE0_IRQ_IRQn);
            NVIC_SetPriority(CDC_SCANLINE0_IRQ_IRQn, cdc->irq_priority);
            NVIC_EnableIRQ(CDC_SCANLINE0_IRQ_IRQn);

            cdc->state.powered = 1;
            break;

        case ARM_POWER_LOW:
        default:
            return ARM_DRIVER_ERROR_UNSUPPORTED;
    }

    return ARM_DRIVER_OK;
}

/**
 \fn            static int32_t Set_Display_Controller_Image_Dimension (CDC_RESOURCES *cdc)
 \brief         Configuring the display controller for Image dimension.
 \param[in]     cdc Pointer to CDC resources.
 \return        \ref execution_status.
 */
static int32_t Set_Display_Controller_Image_Dimension (CDC_RESOURCES *cdc)
{
    CDC_FRAME_INFO *frame_info = (CDC_FRAME_INFO *)cdc->frame_info;
    uint32_t pixel_format = cdc->pixel_format;
    uint16_t width = frame_info->hactive_time;
    uint16_t height = frame_info->vactive_line;
    uint32_t fb_length;

    switch (pixel_format)
    {
        case CDC_PIXEL_FORMAT_ARGB8888:/*ARGB8888*/
        case CDC_PIXEL_FORMAT_RGBA8888:/*RGBA8888*/
        {
            /* In ARGB8888/RGBA8888 standard One pixel handled by 4 byte, so width size multiplied with 4 */
            fb_length = (((width * 4) << 16) | ((width * 4) + BUS_WIDTH));
            break;
        }
        case CDC_PIXEL_FORMAT_RGB888: /*RGB888*/
        {
            /* In RGB888 standard One pixel handled by 3 byte, so width size multiplied with 3 */
            fb_length = (((width * 3) << 16) | ((width * 3) + BUS_WIDTH));

            /* Set Alpha constant*/
            cdc_set_constant_alpha(cdc->regs, CDC_LAYER_1, ALPHA_CONSTANT);
            break;
        }
        case CDC_PIXEL_FORMAT_RGB565: /*RGB565*/
        {
            /* In RGB565 standard One pixel handled by 2 byte, so width size multiplied with 2 */
            fb_length = (((width * 2) << 16) | ((width * 2) + BUS_WIDTH));

            /* Set Alpha constant*/
            cdc_set_constant_alpha(cdc->regs, CDC_LAYER_1, ALPHA_CONSTANT);
            break;
        }
        case CDC_PIXEL_FORMAT_ARGB4444:/*ARGB4444*/
        case CDC_PIXEL_FORMAT_ARGB1555:/*ARGB1555*/
        {
            /* In ARGB4444/ARGB1555 standard One pixel handled by 2 byte, so width size multiplied with 2 */
            fb_length = (((width * 2) << 16) | ((width * 2) + BUS_WIDTH));
            break;
        }
        case CDC_PIXEL_FORMAT_AL44:
        case CDC_PIXEL_FORMAT_AL8:
        default:
            return ARM_DRIVER_ERROR_UNSUPPORTED;

    }

    cdc_set_fb_length(cdc->regs, CDC_LAYER_1, fb_length);

    cdc_set_pixel_format(cdc->regs, CDC_LAYER_1, pixel_format);

    cdc_set_fb_lines(cdc->regs, CDC_LAYER_1, height);

    return ARM_DRIVER_OK;
}

/**
 \fn            static int32_t Display_Controller_Setup (uint32_t image_buff_address, CDC_RESOURCES *cdc)
 \brief         Configuring the display controller.
 \param[in]     image_buff_address: Image stored memory starting address.
 \param[in]     cdc Pointer to CDC resources.
 \return        \ref execution_status.
 */
static int32_t Display_Controller_Setup (uint32_t image_buff_address, CDC_RESOURCES *cdc)
{
    CDC_FRAME_INFO *frame_info = (CDC_FRAME_INFO *)cdc->frame_info;
    uint32_t sync_size = (((frame_info->hsync_time - 1) << 16 ) + (frame_info->vsync_line - 1));
    uint32_t back_porch = (frame_info->hbp_time << 16) + frame_info->vbp_line + sync_size;
    uint32_t active_width = (frame_info->hactive_time << 16) + frame_info->vactive_line + back_porch;
    uint32_t total_width = (frame_info->hfp_time <<16) + frame_info->vfp_line + active_width;
    uint32_t layer_window_h = ((active_width & 0xFFFF0000U) | ((back_porch >> 16)+1));
    uint32_t layer_window_v = (((active_width & 0xFFFFU) << 16) | ((back_porch & 0xFFFFU) + 1));
    uint32_t ret = ARM_DRIVER_OK;

    if (cdc->state.powered == 0)
    {
        return ARM_DRIVER_ERROR;
    }

    if (image_buff_address == 0)
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    /*Disable CDC200*/
    cdc_global_disable(cdc->regs);

    /*Disable IRQ*/
    cdc_irq_disable(cdc->regs, CDC_IRQ_LINE  | \
            CDC_IRQ_FIFO_UNDERRUN_WARNING    | \
            CDC_IRQ_BUS_ERROR                | \
            CDC_IRQ_REGISTER_RELOAD          | \
            CDC_IRQ_SLAVE_TIMING_NO_SIGNAL   | \
            CDC_IRQ_SLAVE_TIMING_NOT_IN_SYNC | \
            CDC_IRQ_FIFO_UNDERRUN_KILLING    | \
            CDC_IRQ_CRC                      | \
            CDC_IRQ_ROTATION_FIFO_ERROR);

    /*Set timing registers*/
    cdc_set_sync_size(cdc->regs, sync_size);
    cdc_set_back_porch(cdc->regs, back_porch);
    cdc_set_active_width(cdc->regs, active_width);
    cdc_set_total_width(cdc->regs, total_width);

    /*Set scanline irq position*/
    cdc_set_line_irq_position(cdc->regs, active_width + 1);

    /*Enable layer*/
    cdc_layer_on(cdc->regs, CDC_LAYER_1);

    /*Set clolor FB address*/
    cdc_set_fb_address(cdc->regs, CDC_LAYER_1, image_buff_address);

    /*set layer Window*/
    cdc_set_win_hpos(cdc->regs, CDC_LAYER_1, layer_window_h);
    cdc_set_win_vpos(cdc->regs, CDC_LAYER_1, layer_window_v);

    /*Configure image dimension*/
    ret = Set_Display_Controller_Image_Dimension (cdc);
    if (ret != ARM_DRIVER_OK)
    {
        return ret;
    }

    /*Force reload of all shadowed registers*/
    cdc_shadow_reload(cdc->regs);

    return ret;
}

/**
 \fn            static int32_t CDC200_control (ARM_CDC200_CONTROL control, uint32_t arg, CDC_RESOURCES *cdc)
 \brief         Control the display controller.
 \param[in]     control CDC200 Configuration.
 \param[in]     arg Argument of operation (optional).
 \param[in]     cdc Pointer to CDC resources.
 \return        \ref execution_status.
 */
static int32_t CDC200_control (ARM_CDC200_CONTROL control, uint32_t arg, CDC_RESOURCES *cdc)
{
    uint32_t ret = ARM_DRIVER_OK;

    if (cdc->state.powered == 0)
    {
        return ARM_DRIVER_ERROR;
    }

    switch (control)
    {
        case CDC200_CONFIGURE_DISPLAY:
            /*Setup display controller*/
            ret = Display_Controller_Setup(arg, cdc);
            if (ret != ARM_DRIVER_OK)
            {
                return ret;
            }

#if (RTE_MIPI_DSI)
            /*MIPI DSI Configure Host*/
            ret = Driver_MIPI_DSI.Control(DSI_CONFIGURE_HOST, 0);
            if (ret != ARM_DRIVER_OK)
            {
                return ret;
            }

            /*Start Command mode and  Configure LCD Panel*/
            ret = Driver_MIPI_DSI.StartCommandMode();
            if (ret != ARM_DRIVER_OK)
            {
                return ret;
            }

            /*MIPI DSI Configure DPI*/
            ret = Driver_MIPI_DSI.Control(DSI_CONFIGURE_DPI, 0);
            if (ret != ARM_DRIVER_OK)
            {
                return ret;
            }
#endif

            cdc->state.configured = 1;
            break;
        case CDC200_FRAMEBUF_UPDATE:
            /*Update the buffer start address for new buffer content*/
            cdc_set_fb_address(cdc->regs, CDC_LAYER_1, arg);
            cdc_shadow_reload(cdc->regs);
            break;

        case CDC200_SCANLINE0_EVENT:
            /*Enable/Disable Scanline0 IRQ*/
            if(arg == ENABLE)
            {
                cdc_irq_enable(cdc->regs, CDC_IRQ_LINE);
            }
            else
            {
                cdc_irq_disable(cdc->regs, CDC_IRQ_LINE);
            }
            break;

        default:
            return ARM_DRIVER_ERROR_UNSUPPORTED;
    }

    return ret;
}

/**
 \fn            static int32_t CDC200_GetVerticalPos (CDC_RESOURCES *cdc)
 \brief         Get current vertical position count.
 \param[in]     cdc Pointer to CDC resources.
 \return        return current vertical position.
 */
static int32_t CDC200_GetVerticalPos (CDC_RESOURCES *cdc)
{
    CDC_FRAME_INFO *frame_info = (CDC_FRAME_INFO *)cdc->frame_info;

    return (int) (cdc_get_position_status(cdc->regs) & 0xFFFF) - frame_info->vsync_line - frame_info->vbp_line;
}
/**
 \fn            static int32_t CDC200_Start (CDC_RESOURCES *cdc)
 \brief         Start the display controller.
 \param[in]     cdc Pointer to CDC resources.
 \return        \ref execution_status.
 */
static int32_t CDC200_Start (CDC_RESOURCES *cdc)
{
    uint32_t ret = ARM_DRIVER_OK;

    if (cdc->state.configured == 0)
    {
        return ARM_DRIVER_ERROR;
    }

#if (RTE_MIPI_DSI)
    /*Start MIPI DSI and LCD Panel*/
    ret = Driver_MIPI_DSI.StartVideoMode();
    if (ret != ARM_DRIVER_OK)
    {
        return ret;
    }
#endif

    /*Enable Display Controller CDC200*/
    cdc_global_enable(cdc->regs);

    return ret;
}

/**
 \fn            static int32_t CDC200_Stop (CDC_RESOURCES *cdc)
 \brief         Stop the display controller.
 \param[in]     cdc Pointer to CDC resources.
 \return        \ref execution_status.
 */
static int32_t CDC200_Stop (CDC_RESOURCES *cdc)
{
    uint32_t ret = ARM_DRIVER_OK;

    if (cdc->state.powered == 0)
    {
        return ARM_DRIVER_ERROR;
    }

    /*Disable Display Controller CDC200*/
    cdc_global_disable(cdc->regs);

#if (RTE_MIPI_DSI)
    /*Stop MIPI DSI and LCD Panel*/
    ret = Driver_MIPI_DSI.Stop();
    if (ret != ARM_DRIVER_OK)
    {
        return ret;
    }
#endif

    return ret;
}

/**
  \fn          static void CDC200_ISR (CDC_RESOURCES *cdc)
  \brief       CDC200 interrupt service routine
  \param[in]   cdc  Pointer to CDC resources
*/
static void CDC200_ISR (CDC_RESOURCES *cdc)
{
    uint32_t irq_st = cdc_get_irq_status(cdc->regs);

    if (!(cdc->cb_event))
    {
        return;
    }

    if (irq_st & CDC_IRQ_LINE)
    {
        cdc->cb_event(ARM_CDC_SCANLINE0_EVENT);
        cdc_irq_clear(cdc->regs, CDC_IRQ_LINE);
    }

    (void)cdc_get_irq_clear(cdc->regs);
}

#if (RTE_CDC200)

CDC_FRAME_INFO FRAME_INFO =
{
    .hsync_time   = RTE_PANEL_HSYNC_TIME,
    .hbp_time     = RTE_PANEL_HBP_TIME,
    .hfp_time     = RTE_PANEL_HFP_TIME,
    .hactive_time = RTE_PANEL_HACTIVE_TIME,
    .vsync_line   = RTE_PANEL_VSYNC_LINE,
    .vbp_line     = RTE_PANEL_VBP_LINE,
    .vfp_line     = RTE_PANEL_VFP_LINE,
    .vactive_line = RTE_PANEL_VACTIVE_LINE,
};

CDC_RESOURCES CDC_INFO =
{
    .regs           = (CDC_Type*) CDC_BASE,
    .cb_event       = NULL,
    .frame_info     = &FRAME_INFO,
    .pixel_format   = RTE_CDC200_PIXEL_FORMAT,
    .irq_priority   = RTE_CDC200_IRQ_PRI,
    .state          = {0},
};

#if (RTE_MIPI_DSI)
/**
  \fn          void MIPI_DSI_Event_Callback (uint32_t int_event)
  \brief       Signal MIPI DSI Events.
  \param[in]   int_event  \ref MIPI DSI event types.
  \return      none.
*/
void MIPI_DSI_Event_Callback (uint32_t int_event)
{
    ARG_UNUSED(int_event);
    CDC_INFO.cb_event(ARM_CDC_DSI_ERROR_EVENT);
}
#endif

/**
  \fn          int32_t CDC200_Initialize (ARM_CDC200_SignalEvent_t cb_event)
  \brief       Initialize CDC200 Interface.
  \param[in]   cb_event Pointer to ARM_CDC200_SignalEvent_t.
  \return      \ref execution_status.
  */
static int32_t CDC200_Initialize (ARM_CDC200_SignalEvent_t cb_event)
{
    return CDC200_Init(cb_event, &CDC_INFO);
}

/**
  \fn          int32_t CDC200_Uninitialize (void)
  \brief       Uninitialize CDC200 Interface.
  \return      \ref execution_status.
  */
static int32_t CDC200_Uninitialize (void)
{
    return CDC200_Uninit(&CDC_INFO);
}

/**
  \fn          int32_t CDC200_PowerControl (ARM_POWER_STATE state)
  \brief       Control CDC200 Interface Power.
  \param[in]   state  Power state.
  \return      \ref execution_status.
  */
static int32_t CDC200_PowerControl (ARM_POWER_STATE state)
{
    return CDC200_PowerCtrl(state, &CDC_INFO);
}

/**
 \fn            int32_t CDC200_Control (ARM_CDC200_CONTROL control, uint32_t arg)
 \brief         Control the display controller.
 \param[in]     control CDC200 Configuration.
 \param[in]     arg Argument of operation (optional).
 \return        \ref execution_status.
 */
static int32_t CDC200_Control (ARM_CDC200_CONTROL control, uint32_t arg)
{
    return CDC200_control(control, arg, &CDC_INFO);
}

/**
 \fn            int32_t CDC200_GetVerticalPosition (void)
 \brief         Get current vertical position count.
 \return        return current vertical Position.
 */
static int32_t CDC200_GetVerticalPosition (void)
{
    return CDC200_GetVerticalPos(&CDC_INFO);
}

/**
 \fn            int32_t CDC200_StartDisplay (void)
 \brief         Start the display controller.
 \return        \ref execution_status.
 */
static int32_t CDC200_StartDisplay (void)
{
    return CDC200_Start(&CDC_INFO);
}

/**
 \fn            int32_t CDC200_StopDisplay (void)
 \brief         Stop the display controller.
 \return        \ref execution_status.
 */
static int32_t CDC200_StopDisplay (void)
{
    return CDC200_Stop(&CDC_INFO);
}

/**
  \fn          void CDC200_SCANLINE0_IRQHandler (void)
  \brief       CDC200 SCANLINE0 IRQ Handler.
*/
void CDC_SCANLINE0_IRQHandler(void)
{
    CDC200_ISR(&CDC_INFO);
}

extern ARM_DRIVER_CDC200 Driver_CDC200;

ARM_DRIVER_CDC200 Driver_CDC200 =
{
    CDC200_GetVersion,
    CDC200_GetCapabilities,
    CDC200_Initialize,
    CDC200_Uninitialize,
    CDC200_PowerControl,
    CDC200_Control,
    CDC200_GetVerticalPosition,
    CDC200_StartDisplay,
    CDC200_StopDisplay
};
#endif
