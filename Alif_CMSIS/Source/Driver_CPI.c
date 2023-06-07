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
 * @file         Driver_CPI.c
 * @author       Tanay Rami and Chandra Bhushan Singh
 * @email        tanay@alifsemi.com and chandrabhushan.singh@alifsemi.com
 * @version      V1.0.0
 * @date         27-March-2023
 * @brief        CMSIS-Driver for Camera Controller
 * @bug          None.
 * @Note         None.
 ******************************************************************************/

/* System Includes */
#include "RTE_Device.h"

/* Project Includes */
#include "Camera_Sensor.h"

/* CPI Includes */
#include "cpi.h"
#include "Driver_CPI_Private.h"

/* CMSIS CPI driver Includes */
#include "Driver_CPI.h"

#if !(RTE_CPI || RTE_LPCPI)
#error "CAMERA is not enabled in the RTE_Device.h"
#endif

#if !defined(RTE_Drivers_CPI)
#error "CAMERA not configured in RTE_Components.h!"
#endif

#if (RTE_MIPI_CSI2)
#include "Driver_MIPI_CSI2.h"
extern ARM_DRIVER_MIPI_CSI2 Driver_MIPI_CSI2;

/* Check for data mode and CSI IPI color code compatibility */
#if(RTE_CPI && RTE_ARX3A0_CAMERA_SENSOR_CPI_ENABLE)
#if ((RTE_ARX3A0_CAMERA_SENSOR_CPI_DATA_MODE == 3) && !(RTE_CPI_COLOR_MODE != 0 || \
        RTE_CPI_COLOR_MODE != 1 || RTE_CPI_COLOR_MODE != 2))
#error "The RTE_CPI_COLOR_MODE must be RAW6, RAW7 or RAW8 for 8 bit CPI data mode."
#endif
#if ((RTE_ARX3A0_CAMERA_SENSOR_CPI_DATA_MODE == 4) && !(RTE_CPI_COLOR_MODE == 3 || \
        RTE_CPI_COLOR_MODE == 4 || RTE_CPI_COLOR_MODE == 5 || RTE_CPI_COLOR_MODE == 6 || \
        RTE_CPI_COLOR_MODE == 7 || RTE_CPI_COLOR_MODE == 8 || RTE_CPI_COLOR_MODE == 9 || \
        RTE_CPI_COLOR_MODE == 10))
#error "The RTE_CPI_COLOR_MODE must be RAW10, RAW12, RAW14, RAW16, RGB444, RGB555 or RGB565 for 16 bit CPI data mode."
#endif
#if ((RTE_ARX3A0_CAMERA_SENSOR_CPI_DATA_MODE == 5) && !(RTE_CPI_COLOR_MODE == 11 || \
        RTE_CPI_COLOR_MODE == 12 || RTE_CPI_COLOR_MODE == 13))
#error "The RTE_CPI_COLOR_MODE must be RGB666, XRGB888 or RGBX888 for 32 bit CPI data mode."
#endif
#endif

/**
  \fn        void ARM_MIPI_CSI2_Event_Callback (uint32_t int_event)
  \brief     Signal MIPI CSI2 Events.
  \param[in] int_event   \ref MIPI CSI2 event types.
  \return    none.
*/
void ARM_MIPI_CSI2_Event_Callback (uint32_t int_event);
#endif

#define ARM_CPI_DRV_VERSION    ARM_DRIVER_VERSION_MAJOR_MINOR(1, 0)  /* driver version */

/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion = {
    ARM_CPI_API_VERSION,
    ARM_CPI_DRV_VERSION
};

/* Driver Capabilities */
static const ARM_CPI_CAPABILITIES DriverCapabilities = {
    1,  /* Supports CPI Snapshot mode,
           In this mode CPI will capture one frame
           then it gets stop. */
    1, /* Supports CPI video mode,
           In this mode CPI will capture frame
           continuously. */
    0  /* Reserved (must be zero) */
};

/**
  \fn        ARM_DRIVER_VERSION CPI_GetVersion(void)
  \brief     get Camera version
  \return    driver version
*/
static ARM_DRIVER_VERSION CPI_GetVersion(void)
{
    return DriverVersion;
}

/**
  \fn        ARM_CPI_CAPABILITIES CPI_GetCapabilities(void)
  \brief     get CPI capabilites
  \return    driver capabilites
*/
static ARM_CPI_CAPABILITIES CPI_GetCapabilities(void)
{
    return DriverCapabilities;
}

/**
  \fn        static void CPI_ConfigureInterrupt(CPI_RESOURCES *CPI, uint32_t events)
  \brief     Configure and enable CPI interrupt.
  \param[in] CPI   Pointer to CPI resources structure
  \param[in] events     possible camera events
  \return    none
*/
static void CPI_ConfigureInterrupt(CPI_RESOURCES *CPI, uint32_t events)
{
    uint32_t irqs = 0;

    irqs |= (events & ARM_CPI_EVENT_CAMERA_CAPTURE_STOPPED) ? CAM_INTR_STOP : 0;
    irqs |= (events & ARM_CPI_EVENT_CAMERA_FRAME_VSYNC_DETECTED) ? CAM_INTR_VSYNC : 0;
    irqs |= (events & ARM_CPI_EVENT_CAMERA_FRAME_HSYNC_DETECTED) ? CAM_INTR_HSYNC : 0;
    irqs |= (events & ARM_CPI_EVENT_ERR_CAMERA_INPUT_FIFO_OVERRUN) ? CAM_INTR_INFIFO_OVERRUN : 0;
    irqs |= (events & ARM_CPI_EVENT_ERR_CAMERA_OUTPUT_FIFO_OVERRUN) ? CAM_INTR_OUTFIFO_OVERRUN : 0;
    irqs |= (events & ARM_CPI_EVENT_ERR_HARDWARE) ? CAM_INTR_BRESP_ERR : 0;

    cpi_enable_interrupt(CPI->regs, irqs);
}

/**
  \fn        int32_t CPI_SetConfiguration(CPI_RESOURCES *CPI, CAMERA_SENSOR_DEVICE *camera_sensor)
  \brief     Set following CPI configurations considering the camera sensor configuration :-
                 - Interface/
                 - Pixel Clock/HSYNC/VSYC Polarity/
                 - /CSI halt/Vsync wait/VSYNC Mode
                 - /Data Mode/MSB/LSB/CODE10ON8/Data Mask
  \param[in] CPI        Pointer to CPI resources structure
  \param[in] cam_sensor Pointer to Camera Sensor Device resources structure
   \return   \ref execution_status.
*/
static int32_t CPI_SetConfiguration(CPI_RESOURCES *CPI, CAMERA_SENSOR_DEVICE *camera_sensor)
{
    /* Set Camera Sensor Interface. */
    if(camera_sensor->Config->interface == CPI_INTERFACE_PARALLEL)
    {
        cpi_enable_parallel_interface(CPI->regs);
    }
    else if(camera_sensor->Config->interface == CPI_INTERFACE_MIPI_CSI && \
            CPI->drv_instance == CPI_INSTANCE_CPI0)
    {
        cpi_enable_csi_interface(CPI->regs);
    }

    /* Enable/Disable Camera Sensor VSYNC wait. */
    cpi_set_vsync_wait(CPI->regs, camera_sensor->Config->vsync_wait);

    /* Set Camera Sensor VSYNC Mode. */
    cpi_set_data_synchronization(CPI->regs, camera_sensor->Config->vsync_mode);

    /* Enable/Disable CPI row round up to 64 bit. */
    cpi_set_row_roundup(CPI ->regs, CPI->row_roundup);

    /* Set Camera Sensor Pixel Clock Polarity. */
    cpi_set_pixelclk_polarity(CPI->regs,camera_sensor->Config->pixelclk_pol);

    /* Set Camera Sensor HSYNC Polarity. */
    cpi_set_hsync_polarity(CPI->regs, camera_sensor->Config->hsync_pol);

    /* Set Camera Sensor VSYNC Polarity. */
    cpi_set_vsync_polarity(CPI->regs, camera_sensor->Config->vsync_pol);

    /* Set Camera Sensor Data Mode. */
    cpi_set_sensor_data_mode(CPI->regs, camera_sensor->Config->data_mode);

    if(camera_sensor->Config->data_mode == CPI_DATA_MODE_BIT_8)
    {
        /* Set Camera Sensor code10on8. */
        cpi_set_code10on8bit_coding(CPI->regs, camera_sensor->Config->code10on8);
    }

    if(camera_sensor->Config->data_mode <= CPI_DATA_MODE_BIT_8)
    {
        /* Set CPI MSB/LSB first (for how data to be stored in memory). */
        cpi_set_data_field(CPI->regs, camera_sensor->Config->data_field);
    }

    if(camera_sensor->Config->data_mode == CPI_DATA_MODE_BIT_16 && \
            CPI->drv_instance == CPI_INSTANCE_CPI0)
    {
        /* Set Camera Sensor Data Mask. */
        cpi_set_sensor_data_mask(CPI->regs, camera_sensor->Config->data_mask);
    }

    return ARM_DRIVER_OK;
}

/**
  \fn         int32_t CPI_SetAllConfigurations(CPI_RESOURCES *CPI, CAMERA_SENSOR_DEVICE *camera_sensor)
  \brief      Set all CPI Configurations to respective registers
                  - set Camera Sensor Configuration
                      - Interface/
                      - Pixel Clock/HSYNC/VSYC Polarity/
                      - /CSI halt/Vsync wait/VSYNC Mode
                      - /Data Mode/MSB/LSB/CODE10ON8/Data Mask
                  - set CPI Configurations:
                      - FIFO/Color mode
                      - Frame/Frame Buffer Start Address/
  \param[in] CPI        Pointer to CPI resources structure
  \param[in] cam_sensor Pointer to Camera Sensor Device resources structure
  \return    \ref execution_status
*/
static int32_t CPI_SetAllConfigurations(CPI_RESOURCES *CPI, CAMERA_SENSOR_DEVICE *camera_sensor)
{
    int32_t ret                = ARM_DRIVER_OK;
    uint16_t frame_height      = (CPI->cnfg->frame.height - 1);
    uint8_t w_wmark            = DEFAULT_WRITE_WMARK;

    cpi_set_csi_halt(CPI->regs, CPI->halt_en);

    if(CPI->halt_en == CPI_CSI_IPI_HALT_FN_ENABLE)
    {
        w_wmark = CPI->cnfg->fifo->write_watermark;
    }

    /* Set Camera Sensor configurations for CPI. */
    ret = CPI_SetConfiguration(CPI, camera_sensor);
    if(ret != ARM_DRIVER_OK)
    {
        return ret;
    }

    /* CPI set fifo configuration */
    cpi_set_fifo_control(CPI->regs, CPI->cnfg->fifo->read_watermark, w_wmark);

    /* CPI set frame configuration */
    cpi_set_frame_config(CPI->regs, CPI->cnfg->frame.width, frame_height);

    if(camera_sensor->Config->interface == CPI_INTERFACE_MIPI_CSI)
    {
        /* CPI set color mode configuration */
        cpi_set_mipi_csi_ipi_color_mode(CPI->regs, CPI->cnfg->color);
    }

    return ARM_DRIVER_OK;
}

/**
  \fn         int32_t CPI_StartCapture(CPI_RESOURCES *CPI)
  \brief      Start CPI
              This function will
                  - check CPI capture status
                  - set frame buffer start address
                  - start capture in Snapshot /video mode.
                      -clear control register
                      -activate software reset
                      -clear control register
                      -enable snapshot or video mode with FIFO clock source selection
                      and start capture
  \param[in] CPI   Pointer to CPI resources structure
  \return    \ref execution_status
*/
static int32_t CPI_StartCapture(CPI_RESOURCES *CPI)
{
    /* Check CPI is busy in capturing? */
    if(cpi_get_capture_status(CPI->regs) != CPI_VIDEO_CAPTURE_STATUS_NOT_CAPTURING)
    {
        return ARM_DRIVER_ERROR_BUSY;
    }

    /* Set Frame Buffer Start Address Register */
    cpi_set_framebuff_start_addr(CPI->regs, CPI->cnfg->framebuff_saddr);

    if(CPI->capture_mode == CPI_MODE_SELECT_SNAPSHOT)
    {
        /* Start Camera Capture in Snapshot mode */
        cpi_start_snapshot_mode(CPI->regs);
    }
    else
    {
        /* Start Camera Capture in video mode */
        cpi_start_video_mode(CPI->regs);
    }

    return ARM_DRIVER_OK;
}

/**
  \fn        int32_t CPI_StopCapture(CPI_RESOURCES *CPI)
  \brief     Stop CPI
             This function will
                 - disable CPI interrupt.
                 - clear control register to stop capturing.
  \param[in] CPI   Pointer to CPI resources structure
  \return    \ref execution_status
*/
static int32_t CPI_StopCapture(CPI_RESOURCES *CPI)
{
    /* Disable CPI Interrupt. */
    cpi_disable_interrupt(CPI->regs, CAM_INTR_STOP | CAM_INTR_HSYNC | CAM_INTR_VSYNC |
                                     CAM_INTR_INFIFO_OVERRUN | CAM_INTR_OUTFIFO_OVERRUN |
                                     CAM_INTR_BRESP_ERR);

    /* Stop Clear CPI control */
    cpi_stop_capture(CPI->regs);

    return ARM_DRIVER_OK;
}

/**
  \fn         int32_t CPIx_Initialize(CPI_RESOURCES *CPI, CAMERA_SENSOR_DEVICE *cam_sensor,
                                                          ARM_CAMERA_RESOLUTION cam_resolution,
                                                          ARM_CPI_SignalEvent_t cb_event)
  \brief      Initialize Camera Sensor and CPI.
              this function will
                  - set the user callback event
                  - call Camera Sensor initialize
                  - if MIPI CSI is enabled, call CSI initialize
  \param[in] CPI       Pointer to CPI resources structure
  \param[in] cam_sensor     Pointer to Camera Sensor Device resources structure
  \param[in] cam_resolution Camera Resolution \ref ARM_CAMERA_RESOLUTION
  \param[in] cb_event       Pointer to Camera Event \ref ARM_CAMERA_CONTROLLER_SignalEvent_t
  \return    \ref execution_status
*/
static int32_t CPIx_Initialize(CPI_RESOURCES *CPI, CAMERA_SENSOR_DEVICE *cam_sensor,
                                                   ARM_CAMERA_RESOLUTION cam_resolution,
                                                   ARM_CPI_SignalEvent_t cb_event)
{
    uint16_t frame_width    = 0;
    uint16_t frame_height   = 0;
    int32_t ret             = ARM_DRIVER_OK;

    if (CPI->status.initialized == 1)
    {
        /* Driver is already initialized */
        return ARM_DRIVER_OK;
    }

    if (!cb_event)
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    switch(cam_resolution)
    {
        case CAMERA_RESOLUTION_VGA_640x480:
        {
            frame_width     = 640;
            frame_height    = 480;
            break;
        }

        case CAMERA_RESOLUTION_560x560:
        {
            frame_width     = 560;
            frame_height    = 560;
            break;
        }

        case CAMERA_RESOLUTION_480x480:
        {
            frame_width     = 480;
            frame_height    = 480;
            break;
        }

        default:
        {
            return ARM_DRIVER_ERROR_PARAMETER;
        }
    }

    /* Set the user callback event. */
    CPI->cb_event = cb_event;

    /* Call Camera Sensor specific init */
    ret = cam_sensor->Ops->Init(cam_resolution);
    if(ret != ARM_DRIVER_OK)
    {
        return ret;
    }

#if (RTE_MIPI_CSI2)
    /*Initializing MIPI CSI2 if the sensor is MIPI CSI2 sensor*/
    ret = Driver_MIPI_CSI2.Initialize(ARM_MIPI_CSI2_Event_Callback, RTE_ARX3A0_CAMERA_SENSOR_CPI_FREQ);
    if(ret != ARM_DRIVER_OK)
    {
        return ret;
    }
#endif

    /* Check if Camera Sensor requires any additional width and height? */
    if(cam_sensor->Info->additional_width)
    {
        frame_width += cam_sensor->Info->additional_width;
    }

    if(cam_sensor->Info->additional_height)
    {
        frame_height += cam_sensor->Info->additional_height;
    }

    /* CPI Frame Configuration. */
    CPI->cnfg->frame.width             = frame_width;
    CPI->cnfg->frame.height            = frame_height;

    /* Set the driver flag as initialized. */
    CPI->status.initialized = 1;

    return ARM_DRIVER_OK;
}

/**
  \fn        int32_t CPIx_Uninitialize(CPI_RESOURCES *CPI, CAMERA_SENSOR_DEVICE *camera_sensor)
  \brief     Un-Initialize Camera Sensor and CPI.
                 - Un-initialize Camera Sensor
                 - If MIPI CSI is enabled, call CSI uninitialize
  \param[in] CPI   Pointer to CPI resources structure
  \param[in] cam_sensor Pointer to Camera Sensor Device resources structure
  \return    \ref execution_status
*/
static int32_t CPIx_Uninitialize(CPI_RESOURCES *CPI, CAMERA_SENSOR_DEVICE *camera_sensor)
{
    int32_t ret = ARM_DRIVER_OK;

    if (CPI->status.initialized == 0)
    {
        /* Driver is uninitialized */
        return ARM_DRIVER_OK;
    }

    if (CPI->status.powered == 1)
    {
        /* Driver is not powered off */
        return ARM_DRIVER_ERROR;
    }

    /* Call Camera Sensor specific uninit */
    camera_sensor->Ops->Uninit();

#if (RTE_MIPI_CSI2)
    /*Uninitializing MIPI CSI2 if the sensor is MIPI CSI2 sensor*/
    ret = Driver_MIPI_CSI2.Uninitialize();
    if(ret != ARM_DRIVER_OK)
    {
        return ret;
    }
#endif
    /* Reset driver flags. */
    CPI->status.initialized = 0;

    return ret;
}

/**
  \fn        int32_t CPIx_PowerControl(CPI_RESOURCES *CPI, ARM_POWER_STATE state)
  \brief     Camera power control.
  \param[in] CPI   Pointer to CPI resources structure
  \param[in] state      Power state
  \return    \ref execution_status
*/
static int32_t CPIx_PowerControl(CPI_RESOURCES *CPI, ARM_POWER_STATE state)
{
    int32_t ret = ARM_DRIVER_OK;

    if (CPI->status.initialized == 0)
    {
        /* Driver is not initialized */
        return ARM_DRIVER_ERROR;
    }

    switch (state)
    {
        case ARM_POWER_OFF:
        {
            if (CPI->status.powered == 0)
            {
                /* Driver is already powered off */
                return ARM_DRIVER_OK;
            }

            /* Disable Camera IRQ */
            NVIC_DisableIRQ(CPI->irq_num);

            /* Clear Any Pending Camera IRQ */
            NVIC_ClearPendingIRQ(CPI->irq_num);

            if(CPI->drv_instance == CPI_INSTANCE_CPI0)
            {
                disable_cpi_periph_clk();
            }
            else
            {
                disable_lpcpi_periph_clk();
            }

#if (RTE_MIPI_CSI2)
            /*Disable MIPI CSI2*/
            ret = Driver_MIPI_CSI2.PowerControl(ARM_POWER_OFF);
            if(ret != ARM_DRIVER_OK)
            {
                return ret;
            }
#endif

            /* Reset the power status of Camera. */
            CPI->status.powered = 0;
            break;
        }

        case ARM_POWER_FULL:
        {
            if (CPI->status.powered == 1)
            {
                /* Driver is already powered ON */
                return ARM_DRIVER_OK;
            }

            if(CPI->drv_instance == CPI_INSTANCE_CPI0)
            {
                enable_cpi_periph_clk();
            }
            else
            {
                enable_lpcpi_periph_clk();
            }

            /* Disable CPI Interrupt. */
            cpi_disable_interrupt(CPI->regs, CAM_INTR_STOP | CAM_INTR_HSYNC | CAM_INTR_VSYNC |
                                             CAM_INTR_INFIFO_OVERRUN | CAM_INTR_OUTFIFO_OVERRUN |
                                             CAM_INTR_BRESP_ERR);

            /* Enable Camera IRQ */
            NVIC_ClearPendingIRQ(CPI->irq_num);
            NVIC_SetPriority(CPI->irq_num, CPI->irq_priority);
            NVIC_EnableIRQ(CPI->irq_num);

#if (RTE_MIPI_CSI2)
            /*Enable MIPI CSI2*/
            ret = Driver_MIPI_CSI2.PowerControl(ARM_POWER_FULL);
            if(ret != ARM_DRIVER_OK)
            {
                return ret;
            }
#endif

            /* Set the power flag enabled */
            CPI->status.powered = 1;
            break;
        }

        case ARM_POWER_LOW:

        default:
        {
            return ARM_DRIVER_ERROR_UNSUPPORTED;
        }
    }

    return ret;
}

/**
  \fn         int32_t CPIx_Capture(CPI_RESOURCES *CPI, CAMERA_SENSOR_DEVICE *camera_sensor,
                                                       void *framebuffer_startaddr,
                                                       CPI_MODE_SELECT mode
  \brief      Start Camera Sensor and CPI (in Snapshot mode or video mode).
              In Snapshot mode, CPI will capture one frame then it gets stop.
              In Video mode, CPI will capture video data continuously.
              This function will
                  - call Camera Sensor Start
                  - set frame buffer start address in CPI
                  - set CPI Capture mode as Snapshot mode or video mode.
                  - start capturing
  \param[in] CPI              Pointer to CPI resources structure
  \param[in] cam_sensor            Pointer to Camera Sensor Device resources structure
  \param[in] framebuffer_startaddr Pointer to frame buffer start address,
                                   where camera captured image will be stored.
  /param[in] mode                  0: Capture video frames continuously
                                   1: Capture one frame and stop
  \return    \ref execution_status
*/
static int32_t CPIx_Capture(CPI_RESOURCES *CPI, CAMERA_SENSOR_DEVICE *camera_sensor,
                                                     void *framebuffer_startaddr,
                                                     CPI_MODE_SELECT mode)
{
    int32_t ret = ARM_DRIVER_OK;

    if(CPI->status.sensor_configured == 0)
    {
        return ARM_DRIVER_ERROR;
    }

    if(!framebuffer_startaddr)
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    /* Check CPI is busy in capturing? */
    if(cpi_get_capture_status(CPI->regs) != CPI_VIDEO_CAPTURE_STATUS_NOT_CAPTURING)
    {
        return ARM_DRIVER_ERROR_BUSY;
    }

#if (RTE_MIPI_CSI2)
    ret = Driver_MIPI_CSI2.StartIPI();
    if(ret != ARM_DRIVER_OK)
    {
        return ret;
    }
#endif

    /* Call Camera Sensor specific Start */
    ret = camera_sensor->Ops->Start();
    if(ret != ARM_DRIVER_OK)
    {
        goto Error_Stop_CSI;
    }

    /* Update Frame Buffer Start Address */
    CPI->cnfg->framebuff_saddr = LocalToGlobal(framebuffer_startaddr);

    /* Set capture mode */
    CPI->capture_mode = mode;

    /* CPI start capturing */
    ret = CPI_StartCapture(CPI);
    if(ret != ARM_DRIVER_OK)
    {
        goto Error_Stop_CSI;
    }

    return ARM_DRIVER_OK;

Error_Stop_Camera_Sensor:
    /* Stop CPI */
    ret = camera_sensor->Ops->Stop();
    if(ret != ARM_DRIVER_OK)
    {
        return ret;
    }

Error_Stop_CSI:
    /*Stop MIPI CSI2 IPI interface*/
    ret = Driver_MIPI_CSI2.StopIPI();
    if(ret != ARM_DRIVER_OK)
    {
        return ret;
    }

    return ARM_DRIVER_ERROR;
}

/**
  \fn        int32_t CPIx_Stop(CPI_RESOURCES *CPI, CAMERA_SENSOR_DEVICE *cam_sensor)
  \brief     Stop Camera Sensor and CPI.
  \param[in] CPI   Pointer to CPI resources structure
  \param[in] cam_sensor Pointer to Camera Sensor Device resources structure
  \return    \ref execution_status
*/
static int32_t CPIx_Stop(CPI_RESOURCES *CPI, CAMERA_SENSOR_DEVICE *camera_sensor)
{
    int32_t ret = ARM_DRIVER_OK;

    /* Call Camera Sensor specific Stop */
    ret = camera_sensor->Ops->Stop();
    if(ret != ARM_DRIVER_OK)
    {
        return ret;
    }

#if (RTE_MIPI_CSI2)
    /*Stop MIPI CSI2 IPI interface*/
    ret = Driver_MIPI_CSI2.StopIPI();
    if(ret != ARM_DRIVER_OK)
    {
        return ret;
    }
#endif

    /* Stop CPI */
    ret = CPI_StopCapture(CPI);
    if(ret != ARM_DRIVER_OK)
    {
        return ret;
    }

    return ARM_DRIVER_OK;
}

/**
  \fn         int32_t CPIx_Control(CPI_RESOURCES *CPI,CAMERA_SENSOR_DEVICE *cam_sensor,
                                                      uint32_t control, uint32_t arg)
  \brief     Control CPI and Camera Sensor.
  \param[in] CPI   Pointer to CPI resources structure
  \param[in] cam_sensor Pointer to Camera Sensor Device resources structure
  \param[in] control    Operation
  \param[in] arg        Argument of operation
  \return    \ref execution_status
*/
static int32_t CPIx_Control(CPI_RESOURCES *CPI, CAMERA_SENSOR_DEVICE *camera_sensor,
                                                uint32_t control, uint32_t arg)
{
    int32_t ret = ARM_DRIVER_OK;
    uint32_t cam_sensor_control = 0;

    if (CPI->status.initialized == 0)
    {
        return ARM_DRIVER_ERROR;
    }

    switch(control)
    {
        case CPI_CONFIGURE:
        {
            /* Set all CPI Configurations. */
            ret = CPI_SetAllConfigurations(CPI, camera_sensor);
            if(ret != ARM_DRIVER_OK)
            {
                return ret;
            }
            break;
        }

        case CPI_CAMERA_SENSOR_CONFIGURE:
        {
            /* Camera Sensor configure */
            cam_sensor_control = 1;
            break;
        }

        case CPI_EVENTS_CONFIGURE:
        {
            /* CPI Events configure */
            CPI_ConfigureInterrupt(CPI, arg);
            break;
        }

        case CPI_CAMERA_SENSOR_GAIN:
        {
            /* Camera Sensor gain */
            ret = camera_sensor->Ops->Control(control, arg);
            if(ret != ARM_DRIVER_OK)
            {
                return ret;
            }
            break;
        }

        default:
        {
            return ARM_DRIVER_ERROR_UNSUPPORTED;
        }
    }

    /* Call Camera Sensor specific Control if required. */
    if(cam_sensor_control)
    {

#if (RTE_MIPI_CSI2)
        /*Configure MIPI CSI2 host and IPI interface*/
        ret = Driver_MIPI_CSI2.ConfigureHost(CSI2_EVENT_PHY_FATAL | CSI2_EVENT_PKT_FATAL | CSI2_EVENT_PHY |
                                            CSI2_EVENT_LINE | CSI2_EVENT_IPI_FATAL | CSI2_EVENT_BNDRY_FRAME_FATAL |
                                            CSI2_EVENT_SEQ_FRAME_FATAL | CSI2_EVENT_CRC_FRAME_FATAL |
                                            CSI2_EVENT_PLD_CRC_FATAL | CSI2_EVENT_DATA_ID | CSI2_EVENT_ECC_CORRECT);
        if(ret != ARM_DRIVER_OK)
        {
            return ret;
        }

        ret = Driver_MIPI_CSI2.ConfigureIPI();
        if(ret != ARM_DRIVER_OK)
        {
            return ret;
        }
#endif

        camera_sensor->Ops->Control(control, arg);
        CPI->status.sensor_configured = 1;
    }

    return ret;
}

/**
  \fn        int32_t CPIx_IRQHandler(CPI_RESOURCES *CPI)
  \brief     Camera interrupt handler.
                 This function will
                     - check CPI received interrupt status.
                     - update events based on interrupt status.
                     - call the user callback function if any event occurs.
                     - clear interrupt status.
  \param[in] CPI   Pointer to CPI resources structure
  \return    \ref execution_status
*/
static void CPIx_IRQHandler(CPI_RESOURCES *CPI)
{
    uint32_t irqs          = 0u;
    uint32_t event         = 0U;
    uint32_t intr_status = 0U;

    intr_status = cpi_get_interrupt_status(CPI->regs);

    /* received capture stop interrupt? */
    if(intr_status & CAM_INTR_STOP)
    {
        irqs |= CAM_INTR_STOP;
        event |= ARM_CPI_EVENT_CAMERA_CAPTURE_STOPPED;
    }

    /* received hsync detected interrupt? */
    if(intr_status & CAM_INTR_HSYNC)
    {
        irqs |= CAM_INTR_HSYNC;
        event |= ARM_CPI_EVENT_CAMERA_FRAME_HSYNC_DETECTED;
    }

    /* received vsync detected interrupt? */
    if(intr_status & CAM_INTR_VSYNC)
    {
        irqs |= CAM_INTR_VSYNC;
        event |= ARM_CPI_EVENT_CAMERA_FRAME_VSYNC_DETECTED;
    }

    /* received fifo over-run interrupt? */
    if(intr_status & CAM_INTR_INFIFO_OVERRUN)
    {
        irqs |= CAM_INTR_INFIFO_OVERRUN;
        event |= ARM_CPI_EVENT_ERR_CAMERA_INPUT_FIFO_OVERRUN;
    }

    /* received fifo under-run interrupt? */
    if(intr_status & CAM_INTR_OUTFIFO_OVERRUN)
    {
        irqs |= CAM_INTR_OUTFIFO_OVERRUN;
        event |= ARM_CPI_EVENT_ERR_CAMERA_OUTPUT_FIFO_OVERRUN;
    }

    /* received bus error interrupt? */
    if(intr_status & CAM_INTR_BRESP_ERR)
    {
        irqs |= CAM_INTR_BRESP_ERR;
        event |= ARM_CPI_EVENT_ERR_HARDWARE;
    }

    /* call the user callback if any event occurs */
    if ((event != 0U) && (CPI->cb_event != NULL) )
    {
        CPI->cb_event(event);
    }

    /* clear interrupt by writing one(W1C) */
    cpi_irq_handler_clear_intr_status(CPI->regs, irqs);
    (void)cpi_get_interrupt_status(CPI->regs);
}

/* CPI Driver Instance */
#if (RTE_CPI)

/* Check MT9M114 Camera Sensor is enable? */
#if (RTE_MT9M114_CAMERA_SENSOR_CPI_ENABLE)
    extern CAMERA_SENSOR_DEVICE cpi_mt9m114_camera_sensor;
    CAMERA_SENSOR_DEVICE *cpi_sensor = (CAMERA_SENSOR_DEVICE *)&cpi_mt9m114_camera_sensor;
#elif (RTE_ARX3A0_CAMERA_SENSOR_CPI_ENABLE)
    extern CAMERA_SENSOR_DEVICE cpi_arx3a0_camera_sensor;
    CAMERA_SENSOR_DEVICE *cpi_sensor = (CAMERA_SENSOR_DEVICE *)&cpi_arx3a0_camera_sensor;
#else
#error "Camera Sensor not enabled in RTE_Device.h!"
#endif

/* CPI FIFO Water mark Configuration. */
static CPI_FIFO_CONFIG fifo_config =
{
    .read_watermark = RTE_CPI_FIFO_READ_WATERMARK,
    .write_watermark = RTE_CPI_FIFO_WRITE_WATERMARK,
};

/* CPI color code Configuration. */
static CPI_CONFIG config =
{
    .fifo = &fifo_config,
    .color = RTE_CPI_COLOR_MODE,
};

/* CPI Device Resource */
static CPI_RESOURCES CPI_CTRL =
{
    .regs             = (LPCPI_Type *) CPI_BASE,
    .irq_num          = CAM_IRQ_IRQn,
    .irq_priority     = RTE_CPI_IRQ_PRI,
    .drv_instance     = CPI_INSTANCE_CPI0,
    .halt_en          = RTE_CPI_CSI_HALT,
    .row_roundup      = RTE_CPI_ROW_ROUNDUP,
    .cnfg             = &config,
};

#if (RTE_MIPI_CSI2)
/**
  \fn        void ARM_MIPI_CSI2_Event_Callback (uint32_t int_event)
  \brief     Signal MIPI CSI2 Events.
  \param[in] int_event   \ref MIPI CSI2 event types.
  \return    none.
*/
void ARM_MIPI_CSI2_Event_Callback (uint32_t int_event)
{
    CPI_CTRL.cb_event (ARM_CPI_EVENT_MIPI_CSI2_ERROR);
}
#endif

/* wrapper functions for CPI */
static int32_t CPI_Initialize(ARM_CAMERA_RESOLUTION cam_resolution,
                                 ARM_CPI_SignalEvent_t cb_event)
{
    return CPIx_Initialize(&CPI_CTRL, cpi_sensor, cam_resolution, cb_event);
}

static int32_t CPI_Uninitialize(void)
{
    return CPIx_Uninitialize(&CPI_CTRL, cpi_sensor);
}

static int32_t CPI_PowerControl(ARM_POWER_STATE state)
{
    return CPIx_PowerControl(&CPI_CTRL, state);
}

static int32_t CPI_CaptureFrame(void *framebuffer_startaddr)
{
    return CPIx_Capture(&CPI_CTRL, cpi_sensor, framebuffer_startaddr, CPI_MODE_SELECT_SNAPSHOT);
}

static int32_t CPI_CaptureVideo(void *framebuffer_startaddr)
{
    return CPIx_Capture(&CPI_CTRL, cpi_sensor, framebuffer_startaddr, CPI_MODE_SELECT_VIDEO);
}

static int32_t CPI_Stop(void)
{
    return CPIx_Stop(&CPI_CTRL, cpi_sensor);
}

static int32_t CPI_Control(uint32_t control, uint32_t arg)
{
    return CPIx_Control(&CPI_CTRL, cpi_sensor, control, arg);
}

void CAM_IRQHandler(void)
{
    CPIx_IRQHandler(&CPI_CTRL);
}

extern ARM_DRIVER_CPI Driver_CPI;
ARM_DRIVER_CPI Driver_CPI =
{
    CPI_GetVersion,
    CPI_GetCapabilities,
    CPI_Initialize,
    CPI_Uninitialize,
    CPI_PowerControl,
    CPI_CaptureFrame,
    CPI_CaptureVideo,
    CPI_Stop,
    CPI_Control,
};

#endif /* End of RTE_CPI */

/* LPCPI Driver Instance */
#if (RTE_LPCPI)

/* Check MT9M114 Camera Sensor is enable? */
#if (RTE_MT9M114_CAMERA_SENSOR_LPCPI_ENABLE)
    extern CAMERA_SENSOR_DEVICE lpcpi_mt9m114_camera_sensor;
    CAMERA_SENSOR_DEVICE *lpcpi_sensor = (CAMERA_SENSOR_DEVICE *)&lpcpi_mt9m114_camera_sensor;
#else
#error "Camera Sensor not enabled in RTE_Device.h!"
#endif

/* LPCPI FIFO Water mark Configuration. */
static CPI_FIFO_CONFIG fifo_cnfg =
{
    .read_watermark = RTE_LPCPI_FIFO_READ_WATERMARK,
    .write_watermark = RTE_LPCPI_FIFO_WRITE_WATERMARK,
};

/* LPCPI color code Configuration. */
static CPI_CONFIG cnfg =
{
    .fifo = &fifo_cnfg,
};

    /* LPCPI Device Resource */
static CPI_RESOURCES LPCPI_CTRL =
{
    .regs             = (LPCPI_Type *) LPCPI_BASE,
    .irq_num          = LPCPI_IRQ_IRQn,
    .irq_priority     = RTE_LPCPI_IRQ_PRI,
    .drv_instance     = CPI_INSTANCE_LPCPI,
    .cnfg             = &cnfg,
};

/* wrapper functions for LPCPI */
static int32_t LPCPI_Initialize(ARM_CAMERA_RESOLUTION cam_resolution,
                                 ARM_CPI_SignalEvent_t cb_event)
{
    return CPIx_Initialize(&LPCPI_CTRL, lpcpi_sensor, cam_resolution, cb_event);
}

static int32_t LPCPI_Uninitialize(void)
{
    return CPIx_Uninitialize(&LPCPI_CTRL, lpcpi_sensor);
}

static int32_t LPCPI_PowerControl(ARM_POWER_STATE state)
{
    return CPIx_PowerControl(&LPCPI_CTRL, state);
}

static int32_t LPCPI_CaptureFrame(void *framebuffer_startaddr)
{
    return CPIx_Capture(&LPCPI_CTRL, lpcpi_sensor, framebuffer_startaddr, CPI_MODE_SELECT_SNAPSHOT);
}

static int32_t LPCPI_CaptureVideo(void *framebuffer_startaddr)
{
    return CPIx_Capture(&LPCPI_CTRL, lpcpi_sensor, framebuffer_startaddr, CPI_MODE_SELECT_VIDEO);
}

static int32_t LPCPI_Stop(void)
{
    return CPIx_Stop(&LPCPI_CTRL, lpcpi_sensor);
}

static int32_t LPCPI_Control(uint32_t control, uint32_t arg)
{
    return CPIx_Control(&LPCPI_CTRL, lpcpi_sensor, control, arg);
}

void LPCPI_IRQHandler(void)
{
    CPIx_IRQHandler(&LPCPI_CTRL);
}

extern ARM_DRIVER_CPI Driver_LPCPI;
ARM_DRIVER_CPI Driver_LPCPI =
{
    CPI_GetVersion,
    CPI_GetCapabilities,
    LPCPI_Initialize,
    LPCPI_Uninitialize,
    LPCPI_PowerControl,
    LPCPI_CaptureFrame,
    LPCPI_CaptureVideo,
    LPCPI_Stop,
    LPCPI_Control,
};

#endif /* End of RTE_LPCPI */

/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
