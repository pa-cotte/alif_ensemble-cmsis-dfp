/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
#include <Driver_MIPI_CSI2.h>
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */
/**************************************************************************//**
 * @file     Driver_MIPI_CSI2.c
 * @author   Prasanna Ravi and Chandra Bhushan Singh
 * @email    prasanna.ravi@alifsemi.com and chandrabhushan.singh@alifsemi.com
 * @version  V1.0.0
 * @date     15-April-2023
 * @brief    CMSIS-Driver for MIPI CSI2.
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

/* System Includes */
#include "RTE_Device.h"
#include "RTE_Components.h"
#include CMSIS_device_header

/* CSI includes */
#include "Driver_MIPI_CSI2.h"
#include "csi.h"
#include "DPHY_init.h"
#include "sys_ctrl_csi.h"
#include "Driver_CSI_Private.h"

#if !(RTE_MIPI_CSI2)
#error "MIPI CSI2 is not enabled in the RTE_Device.h"
#endif
#if (!defined(RTE_Drivers_MIPI_CSI2))
#error "MIPI CSI2 not configured in RTE_Components.h!"
#endif

#define ARM_MIPI_CSI2_DRV_VERSION    ARM_DRIVER_VERSION_MAJOR_MINOR(1, 0)

/*Driver version*/
static const ARM_DRIVER_VERSION DriverVersion =
{
    ARM_MIPI_CSI2_API_VERSION,
    ARM_MIPI_CSI2_DRV_VERSION
};

/* Driver Capabilities */
static const ARM_MIPI_CSI2_CAPABILITIES DriverCapabilities =
{
    0, /* Not supports reentrant_operation */
    1, /* IPI interface supported*/
    0, /* IDI interface not supported*/
    0  /* reserved (must be zero) */
};

/**
  \fn          ARM_DRIVER_VERSION MIPI_CSI2_GetVersion (void)
  \brief       Get MIPI CSI2 driver version.
  \return      \ref ARM_DRIVER_VERSION
*/
static ARM_DRIVER_VERSION MIPI_CSI2_GetVersion (void)
{
    return DriverVersion;
}

/**
  \fn          ARM_MIPI_CSI2_CAPABILITIES MIPI_CSI2_GetCapabilities (void)
  \brief       Get MIPI CSI2 driver capabilities
  \return      \ref ARM_MIPI_DPHY_CAPABILITIES
*/
static ARM_MIPI_CSI2_CAPABILITIES MIPI_CSI2_GetCapabilities (void)
{
    return DriverCapabilities;
}

/**
  \fn          int32_t CSI2_Initialize (ARM_MIPI_CSI2_SignalEvent_t cb_event,
                                        uint32_t frequency,
                                        CSI_RESOURCES *CSI2)
  \brief       Initialize MIPI CSI2 Interface.
  \param[in]   cb_event Pointer to ARM_MIPI_CSI2_SignalEvent_t
  \param[in]   frequency to configure DPHY PLL.
  \param[in]   CSI2 Pointer to CSI resources
  \return      \ref execution_status
*/
static int32_t CSI2_Initialize (ARM_MIPI_CSI2_SignalEvent_t cb_event,
                                uint32_t frequency,
                                CSI_RESOURCES *CSI2)
{
    int32_t ret = ARM_DRIVER_OK;

    if(CSI2->status.initialized == 1)
    {
        /* Driver already initialized */
        return ARM_DRIVER_OK;
    }

    if (!cb_event)
    {
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    CSI2->cb_event = cb_event;

    /*DPHY initialization*/
    ret  = CSI2_DPHY_Initialize(frequency);
    if(ret != ARM_DRIVER_OK)
    {
        return ret;
    }

    CSI2->status.initialized = 1;

    return ret;
}

/**

  \fn          int32_t CSI2_Uninitialize (CSI_RESOURCES *CSI2)
  \brief       uninitialize MIPI CSI2 Interface.
  \param[in]   CSI2 Pointer to CSI resources
  \return      \ref execution_status
*/
static int32_t CSI2_Uninitialize (CSI_RESOURCES *CSI2)
{
    int32_t ret = ARM_DRIVER_OK;
    CSI2->cb_event = NULL;

    if (CSI2->status.initialized == 0)
    {
        /* Driver is already uninitialized */
        return ARM_DRIVER_OK;
    }

    if (CSI2->status.powered == 1)
    {
        /* Driver is not powered off */
        return ARM_DRIVER_ERROR;
    }

    /*DPHY Uninitialization*/
    ret  = CSI2_DPHY_Uninitialize();
    if(ret != ARM_DRIVER_OK)
    {
        return ret;
    }

    /* Reset driver flags. */
    CSI2->status.initialized = 0;

    return ret;
}

/**

  \fn          int32_t CSI2_PowerControl (ARM_POWER_STATE state, CSI_RESOURCES *CSI2)
  \brief       Control CSI2 Interface Power.
  \param[in]   state  Power state
  \param[in]   CSI2  Pointer to CSI resources
  \return      \ref execution_status
*/
static int32_t CSI2_PowerControl (ARM_POWER_STATE state, CSI_RESOURCES *CSI2)
{
    if (CSI2->status.initialized == 0)
    {
        /* Driver is not initialized */
        return ARM_DRIVER_ERROR;
    }

    switch (state)
    {
        case ARM_POWER_OFF:
        {
            if (CSI2->status.powered == 0)
            {
                /* Driver is already powered off */
                return ARM_DRIVER_OK;
            }

            /*Disabling the IRQs*/
            csi_clear_phy_pkt_discard_intr_mask(CSI2->regs);
            csi_clear_phy_pkt_construction_intr_mask(CSI2->regs);
            csi_clear_phy_intr_mask(CSI2->regs);
            csi_clear_phy_line_construction_intr_mask(CSI2->regs);
            csi_clear_ipi_intr_mask(CSI2->regs);
            csi_clear_frame_bndry_err_intr_mask(CSI2->regs);
            csi_clear_frame_seq_err_intr_mask(CSI2->regs);
            csi_clear_frame_crc_err_intr_mask(CSI2->regs);
            csi_clear_frame_payload_err_intr_mask(CSI2->regs);
            csi_clear_dt_err_intr_mask(CSI2->regs);
            csi_clear_ecc_intr_mask(CSI2->regs);

            NVIC_DisableIRQ (CSI2->irq);
            NVIC_ClearPendingIRQ (CSI2->irq);

            clear_csi_pixel_clk();

            CSI2->status.powered = 0;

            break;
        }

        case ARM_POWER_FULL:
        {
            if (CSI2->status.powered == 1)
            {
                /* Driver is already powered ON */
                return ARM_DRIVER_OK;
            }

            NVIC_ClearPendingIRQ (CSI2->irq);
            NVIC_SetPriority (CSI2->irq, CSI2->irq_priority);
            NVIC_EnableIRQ (CSI2->irq);

            set_csi_pixel_clk(CSI_PIX_CLKSEL_400MZ, CSI2->csi_pixclk_div);

            CSI2->status.powered = 1;

            break;
        }

        case ARM_POWER_LOW:

        default:
        {
            return ARM_DRIVER_ERROR_UNSUPPORTED;
        }
    }

    return ARM_DRIVER_OK;
}

/**

  \fn          int32_t CSI2_ConfigureHost (uint32_t int_event, CSI_RESOURCES *CSI2)
  \brief       Configure CSI2 Host Interface.
  \param[in]   int_event interrupt event to be enabled.
  \param[in]   CSI2  Pointer to CSI resources
  \return      \ref execution_status
*/
static int32_t CSI2_ConfigureHost (uint32_t intr_event, CSI_RESOURCES *CSI2)
{
    if(CSI2->status.powered != 1)
    {
        return ARM_DRIVER_ERROR;
    }

    csi_enable_software_reset_state(CSI2->regs);
    csi_set_n_active_lanes(CSI2->regs, (CSI2->n_lanes - 1));

    if(intr_event & CSI2_EVENT_PHY_FATAL)
    {
        csi_set_phy_pkt_discard_intr_mask(CSI2->regs);
    }

    if(intr_event & CSI2_EVENT_PKT_FATAL)
    {
        csi_set_phy_pkt_construction_intr_mask(CSI2->regs);
    }

    if(intr_event & CSI2_EVENT_PHY)
    {
        csi_set_phy_intr_mask(CSI2->regs);
    }

    if(intr_event & CSI2_EVENT_LINE)
    {
        csi_set_phy_line_construction_intr_mask(CSI2->regs);
    }

    if(intr_event & CSI2_EVENT_IPI_FATAL)
    {
        csi_set_ipi_intr_mask(CSI2->regs);
    }

    if(intr_event & CSI2_EVENT_BNDRY_FRAME_FATAL)
    {
        csi_set_frame_bndry_err_intr_mask(CSI2->regs);
    }

    if(intr_event & CSI2_EVENT_SEQ_FRAME_FATAL)
    {
        csi_set_frame_seq_err_intr_mask(CSI2->regs);
    }

    if(intr_event & CSI2_EVENT_CRC_FRAME_FATAL)
    {
        csi_set_frame_crc_err_intr_mask(CSI2->regs);
    }

    if(intr_event & CSI2_EVENT_PLD_CRC_FATAL)
    {
        csi_set_frame_payload_err_intr_mask(CSI2->regs);
    }

    if(intr_event & CSI2_EVENT_DATA_ID)
    {
        csi_set_dt_err_intr_mask(CSI2->regs);
    }

    if(intr_event & CSI2_EVENT_ECC_CORRECT)
    {
        csi_set_ecc_intr_mask(CSI2->regs);
    }

    return ARM_DRIVER_OK;
}

/**

  \fn          void MIPI_CSI2_ISR (CSI_RESOURCES *CSI2)
  \brief       MIPI CSI2 interrupt service routine
  \param[in]   CSI2  Pointer to CSI resources
*/
static void MIPI_CSI2_ISR (CSI_RESOURCES *CSI2)
{
    uint32_t global_event_status = 0U;
    uint32_t event               = 0U;

    global_event_status = csi_get_interrupt_status(CSI2->regs);

    /*If cb_event is not registered Disable and clear the interrupt*/
    if(!CSI2->cb_event)
    {
        NVIC_DisableIRQ (CSI2->irq);
    }

    if(global_event_status & CSI_IRQ_PHY_FATAL)
    {
        (void)csi_get_phy_pkt_discard_intr_status(CSI2->regs);
        event |= CSI2_EVENT_PHY_FATAL;
    }

    if(global_event_status & CSI_IRQ_PKT_FATAL)
    {
        (void)csi_get_phy_pkt_construction_intr_status(CSI2->regs);
        event |= CSI2_EVENT_PKT_FATAL;
    }

    if(global_event_status & CSI_IRQ_BNDRY_FRAME_FATAL)
    {
        (void)csi_get_frame_bndry_err_intr_status(CSI2->regs);
        event |= CSI2_EVENT_BNDRY_FRAME_FATAL;
    }

    if(global_event_status & CSI_IRQ_SEQ_FRAME_FATAL)
    {
        (void)csi_get_frame_seq_err_intr_status(CSI2->regs);
        event |= CSI2_EVENT_SEQ_FRAME_FATAL;
    }

    if(global_event_status & CSI_IRQ_CRC_FRAME_FATAL)
    {
        (void)csi_get_frame_crc_err_intr_status(CSI2->regs);
        event |= CSI2_EVENT_CRC_FRAME_FATAL;
    }

    if(global_event_status & CSI_IRQ_PLD_CRC_FATAL)
    {
        (void)csi_get_frame_payload_err_intr_status(CSI2->regs);
        event |= CSI2_EVENT_PLD_CRC_FATAL;
    }

    if(global_event_status & CSI_IRQ_DATA_ID)
    {
        (void)csi_get_dt_err_intr_status(CSI2->regs);
        event |= CSI2_EVENT_DATA_ID;
    }

    if(global_event_status & CSI_IRQ_ECC_CORRECT)
    {
        (void)csi_get_ecc_intr_status(CSI2->regs);
        event |= CSI2_EVENT_ECC_CORRECT;
    }

    if(global_event_status & CSI_IRQ_PHY)
    {
        (void)csi_get_phy_intr_status(CSI2->regs);
        event |= CSI2_EVENT_PHY;
    }

    if(global_event_status & CSI_IRQ_LINE)
    {
        (void)csi_get_phy_line_construction_intr_status(CSI2->regs);
        event |= CSI2_EVENT_LINE;
    }

    if(global_event_status & CSI_IRQ_IPI_FATAL)
    {
        (void)csi_get_ipi_intr_status(CSI2->regs);
        event |= CSI2_EVENT_IPI_FATAL;
    }

    if(event != 0 && CSI2->cb_event)
    {
        CSI2->cb_event(event);
    }
}

/**
  \fn          int32_t  CSI2_ConfigureIPI (CSI_RESOURCES *CSI2)
  \brief       Configure CSI2 IPI Interface.
  \param[in]   CSI2  Pointer to CSI resources
  \return      \ref execution_status
*/
static int32_t CSI2_ConfigureIPI (CSI_RESOURCES *CSI2)
{
    uint32_t packet_config;
    uint16_t hline_time;

    if(CSI2->status.powered != 1)
    {
        return ARM_DRIVER_ERROR;
    }

    csi_set_ipi_mode(CSI2->regs, CSI2->ipi_info->ipi_mode);

    csi_set_ipi_color_cop(CSI2->regs, CSI2->ipi_info->ipi_color_cop);

    if(CSI2->ipi_info->ipi_memflush == ENABLE)
    {
        csi_enable_ipi_mem_flush_auto(CSI2->regs);
    }
    else
    {
    	csi_set_ipi_mem_flush_manual(CSI2->regs);
    }

    csi_set_ipi_vc_id(CSI2->regs, CSI2->vc_id);

    csi_set_ipi_data_type(CSI2->regs, CSI2->pixel_data_type);

    csi_set_ipi_sync_event_type(CSI2->regs, CSI2->ipi_info->adv_features->sync_evnt_mode);

    csi_set_ipi_line_event_selection(CSI2->regs, CSI2->ipi_info->adv_features->event_sel);

    if(CSI2->ipi_info->adv_features->event_sel == ENABLE)
    {
        if(CSI2->ipi_info->adv_features->en_video == ENABLE)
        {
            packet_config |= CSI_IPI_EVENT_SELECTION_EN_VIDEO;
        }
        if(CSI2->ipi_info->adv_features->en_line_start == ENABLE)
        {
            packet_config |= CSI_IPI_EVENT_SELECTION_EN_LINE_START;
        }
        if(CSI2->ipi_info->adv_features->en_null == ENABLE)
        {
            packet_config |= CSI_IPI_EVENT_SELECTION_EN_NULL;
        }
        if(CSI2->ipi_info->adv_features->en_blanking == ENABLE)
        {
            packet_config |= CSI_IPI_EVENT_SELECTION_EN_BLANKING;
        }
        if(CSI2->ipi_info->adv_features->en_embedded == ENABLE)
        {
            packet_config |= CSI_IPI_EVENT_SELECTION_EN_EMBEDDED;
        }

        csi_set_packet_configuration(CSI2->regs, packet_config);
    }

    if(CSI2->ipi_info->adv_features->ipi_dt_overwrite == ENABLE)
    {
        csi_enable_ipi_dt_overwrite(CSI2->regs);
        csi_set_ipi_dt_overwrite(CSI2->regs, CSI2->ipi_info->adv_features->ipi_dt);
    }
    else
    {
        csi_disable_ipi_dt_overwrite(CSI2->regs);
    }

    hline_time = CSI2->ipi_info->frame_info->hactive_time + CSI2->ipi_info->frame_info->hsa_time
                 + CSI2->ipi_info->frame_info->hbp_time + CSI2->ipi_info->frame_info->hsd_time;

    csi_set_horizontal_timing(CSI2->regs, CSI2->ipi_info->frame_info->hsa_time,
                                          CSI2->ipi_info->frame_info->hbp_time,
                                          CSI2->ipi_info->frame_info->hsd_time,
                                          hline_time);

    csi_set_vertical_timing(CSI2->regs, CSI2->ipi_info->frame_info->vsa_line,
                                        CSI2->ipi_info->frame_info->vbp_line,
                                        CSI2->ipi_info->frame_info->vfp_line,
                                        CSI2->ipi_info->frame_info->vactive_line);

    csi_disable_software_reset_state(CSI2->regs);

    CSI2->status.csi_configured = 1;

    return ARM_DRIVER_OK;
}

/**

  \fn          int32_t  CSI2_StartIPI (CSI_RESOURCES *CSI2)
  \brief       Enable CSI2 IPI Interface.
  \param[in]   CSI2  Pointer to CSI resources
  \return      \ref execution_status
*/
static int32_t CSI2_StartIPI (CSI_RESOURCES *CSI2)
{
    if (CSI2->status.csi_configured == 0)
    {
        return ARM_DRIVER_ERROR;
    }

    csi_enable_ipi_mode(CSI2->regs);

    return ARM_DRIVER_OK;
}

/**

  \fn          int32_t  CSI2_StopIPI (CSI_RESOURCES *CSI2)
  \brief       Disable CSI2 IPI Interface.
  \param[in]   CSI2  Pointer to CSI resources
  \return      \ref execution_status
*/
static int32_t CSI2_StopIPI (CSI_RESOURCES *CSI2)
{
    if (CSI2->status.powered == 0)
    {
        return ARM_DRIVER_ERROR;
    }

    csi_disable_ipi_mode(CSI2->regs);

    return ARM_DRIVER_OK;
}

/* CSI frame configuration */
CSI_FRAME_INFO CSI_FRAME_CFG =
{
    .hsa_time           = RTE_MIPI_CSI2_IPI_HSA_TIME,
    .hbp_time           = RTE_MIPI_CSI2_IPI_HBP_TIME,
    .hsd_time           = RTE_MIPI_CSI2_IPI_HSD_TIME,
    .hactive_time       = RTE_MIPI_CSI2_IPI_HACTIVE_TIME,
    .vsa_line           = RTE_MIPI_CSI2_IPI_VSA_LINE,
    .vbp_line           = RTE_MIPI_CSI2_IPI_VBP_LINE,
    .vfp_line           = RTE_MIPI_CSI2_IPI_VFP_LINE,
    .vactive_line       = RTE_MIPI_CSI2_IPI_VACTIVE_LINE,
};

/* CSI IPI advanced configuration */
CSI_IPI_ADV_INFO CSI_IPI_ADV_FEATURES_CFG =
{
    .sync_evnt_mode     = RTE_MIPI_CSI2_SYNC_ET_MODE,
    .event_sel          = RTE_MIPI_CSI2_SYNC_ET_SEL,
    .en_embedded        = RTE_MIPI_CSI2_EN_EMBEDDED,
    .en_blanking        = RTE_MIPI_CSI2_EN_BLANKING,
    .en_null            = RTE_MIPI_CSI2_EN_NULL,
    .en_line_start      = RTE_MIPI_CSI2_EN_LINE_START,
    .en_video           = RTE_MIPI_CSI2_EN_VIDEO,
    .ipi_dt             = RTE_MIPI_CSI2_EN_DT,
    .ipi_dt_overwrite   = RTE_MIPI_CSI2_EN_DT_OVERWRITE,
};

/* CSI IPI configuration */
CSI_IPI_INFO CSI_IPI_CFG =
{
    .ipi_mode           = RTE_MIPI_CSI2_IPI_MODE,
    .ipi_color_cop      = RTE_MIPI_CSI2_COLOR_COP,
    .ipi_memflush       = RTE_MIPI_CSI2_MEMFLUSH,
    .frame_info         = &CSI_FRAME_CFG,
    .adv_features       = &CSI_IPI_ADV_FEATURES_CFG,
};

/* CSI resources */
CSI_RESOURCES CSI2 =
{
    .regs               = (CSI_Type *)CSI_BASE,
    .cb_event           = NULL,
    .csi_pixclk_div     = RTE_MIPI_CSI2_PIXCLK_DIV,
    .n_lanes            = RTE_MIPI_CSI2_N_LANES,
    .vc_id              = RTE_MIPI_CSI2_VC_ID,
    .pixel_data_type    = RTE_MIPI_CSI2_DATA_TYPE,
    .ipi_info           = &CSI_IPI_CFG,
    .irq                = (IRQn_Type)CSI_IRQ_IRQn,
    .irq_priority       = RTE_MIPI_CSI2_IRQ_PRI,
};

static int32_t MIPI_CSI2_Initialize (ARM_MIPI_CSI2_SignalEvent_t cb_event,
                                     uint32_t frequency)
{
    return CSI2_Initialize(cb_event, frequency, &CSI2);
}

static int32_t MIPI_CSI2_Uninitialize (void)
{
    return CSI2_Uninitialize(&CSI2);
}

static int32_t MIPI_CSI2_PowerControl (ARM_POWER_STATE state)
{
    return CSI2_PowerControl (state, &CSI2);
}

static int32_t MIPI_CSI2_ConfigureHost (uint32_t int_event)
{
    return CSI2_ConfigureHost (int_event, &CSI2);
}

static int32_t MIPI_CSI2_ConfigureIPI (void)
{
    return CSI2_ConfigureIPI(&CSI2);
}

static int32_t MIPI_CSI2_StartIPI(void)
{
    return CSI2_StartIPI(&CSI2);
}

static int32_t MIPI_CSI2_StopIPI(void)
{
    return CSI2_StopIPI(&CSI2);
}
void CSI_IRQHandler (void)
{
    MIPI_CSI2_ISR(&CSI2);
}

/**
\brief Access structure of the  MIPI CSI2 Driver.
*/
extern ARM_DRIVER_MIPI_CSI2 Driver_MIPI_CSI2;
ARM_DRIVER_MIPI_CSI2 Driver_MIPI_CSI2 =
{
    MIPI_CSI2_GetVersion,
    MIPI_CSI2_GetCapabilities,
    MIPI_CSI2_Initialize,
    MIPI_CSI2_Uninitialize,
    MIPI_CSI2_PowerControl,
    MIPI_CSI2_ConfigureHost,
    MIPI_CSI2_ConfigureIPI,
    MIPI_CSI2_StartIPI,
    MIPI_CSI2_StopIPI,
};

/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
