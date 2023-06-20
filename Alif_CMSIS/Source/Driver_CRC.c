/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/* Project Includes */
#include "Driver_CRC.h"
#include "crc.h"
#include "Driver_CRC_Private.h"

#if !(RTE_CRC0 || RTE_CRC1)
#error "CRC is not enabled in the RTE_device.h"
#endif

#if (defined(RTE_Drivers_CRC0) && !RTE_CRC0)
#error "CRC0 not configured in RTE_Device.h!"
#endif

#if (defined(RTE_Drivers_CRC1) && !RTE_CRC1)
#error "CRC1 not configured in RTE_Device.h!"
#endif

#define ARM_CRC_DRV_VERSION    ARM_DRIVER_VERSION_MAJOR_MINOR(1, 0)  /*  Driver version */

/*Driver version*/
static const ARM_DRIVER_VERSION DriverVersion = {
    ARM_CRC_API_VERSION,
    ARM_CRC_DRV_VERSION
};

/*Driver Capabilities   */
static const ARM_CRC_CAPABILITIES DriverCapabilities = {
    1,  /* Supports CRC_8_CCITT */
    1,  /* Supports CRC_16 */
    1,  /* Supports CRC_16_CCITT */
    1,  /* Supports CRC_32 */
    1,  /* Supports CRC_32C */
    0   /* Reserved ( must be ZERO) */
};

/**
 @fn           ARM_DRIVER_VERSION CRC_GetVersion(void)
 @brief        get CRC version
 @param        none
 @return       driver version
 */
static ARM_DRIVER_VERSION CRC_GetVersion(void)
{
    return DriverVersion;
}

/**
 @fn           ARM_CRC_CAPABILITIES CRC_GetCapabilities(void)
 @brief        get CRC Capabilites
 @param        none
 @return       driver Capabilites
 */
static ARM_CRC_CAPABILITIES CRC_GetCapabilities(void)
{
    return DriverCapabilities;
}

/**
 @fn           Control_Bit(uint32_t control, uint32_t arg, CRC_RESOURCES *CRC)
 @brief        To enable or disable the Reflect, Invert, Bit, Byte, Custom polynomial bit of CRC
 @param[in]    control : To check CRC Reflect, Invert, Bit, Byte, Custom polynomial bits of CRC
                         are enabled.
 @param[in]    arg     : To enable or disable the Reflect, Invert, Bit, Byte,
                         Custom polynomial bits of CRC
 @param[in]    CRC     : Pointer to CRC resources
 @return       none
 */
__STATIC_INLINE void Control_Bit (uint32_t control, uint32_t arg, CRC_RESOURCES *CRC)
{
    /* To select the CRC byte swap */
    if (control & ARM_CRC_ENABLE_BYTE_SWAP )
    {
        if(arg)
            crc_enable_byte_swap(CRC->regs);
        else
            crc_disable_byte_swap(CRC->regs);
    }

    /*To select the CRC bit swap */
    if (control & ARM_CRC_ENABLE_BIT_SWAP)
    {
        if(arg)
            crc_enable_bit_swap(CRC->regs);
        else
            crc_disable_bit_swap(CRC->regs);
    }

    /*To select the CRC custom polynomial */
    if(control & ARM_CRC_ENABLE_CUSTOM_POLY)
    {
        if(arg)
            crc_enable_custom_poly(CRC->regs);
        else
            crc_disable_custom_poly(CRC->regs);
    }

    /*To select the CRC Invert */
    if(control & ARM_CRC_ENABLE_INVERT)
    {
        if(arg)
            crc_enable_invert(CRC->regs);
        else
            crc_disable_invert(CRC->regs);
    }

    /*To select the CRC reflect */
    if(control & ARM_CRC_ENABLE_REFLECT)
    {
        if(arg)
            crc_enable_reflect(CRC->regs);
        else
            crc_disable_reflect(CRC->regs);
    }
}

/**
@fn          int32_t CRC_Initialize (CRC_RESOURCES *CRC)
@brief       Initialize the CRC interface
@param[in]   CRC : Pointer to CRC resources
 @return     ARM_DRIVER_ERROR_PARAMETER : if CRC device is invalid
             ARM_DRIVER_OK              : if CRC successfully initialized or already initialized
 */
static int32_t CRC_Initialize(CRC_RESOURCES *CRC)
{
    int ret = ARM_DRIVER_OK;
#if RTE_Drivers_DMA
    atomic_init(&CRC->dma_cb_val, 0);
#endif
    /* Setting the state */
    CRC->state.initialized = 1;

    return ret;
}

/**
@fn          int32_t CRC_Uninitialize (CRC_RESOURCES *CRC)
@brief       Clear the CRC configuration
@param[in]   CRC : Pointer to CRC resources
 @return     ARM_DRIVER_ERROR_PARAMETER : if CRC device is invalid
             ARM_DRIVER_OK              : if CRC successfully initialized or already initialized
 */
static int32_t CRC_Uninitialize(CRC_RESOURCES *CRC)
{
    int ret = ARM_DRIVER_OK;

    /* Clear the CRC configuration */
    crc_clear_config(CRC->regs);

    /* Reset the state */
    CRC->state.initialized = 0;

    return ret;
}

/**
 @fn           int32_t CRC_PowerControl (ARM_POWER_STATE state,
                                          CRC_RESOURCES *CRC)
 @brief        CMSIS-DRIVER CRC power control
 @param[in]    state : Power state
 @param[in]    CRC   : Pointer to CRC resources
 @return       ARM_DRIVER_ERROR_PARAMETER  : if CRC device is invalid
               ARM_DRIVER_OK               : if CRC successfully uninitialized or already not initialized
 */
static int32_t CRC_PowerControl(ARM_POWER_STATE status,
                                CRC_RESOURCES *CRC)
{
    switch(status)
    {
    case ARM_POWER_OFF:

        /* Reset the power state */
        CRC->state.powered = 0;

    break;

    case ARM_POWER_FULL:

    if(CRC->state.initialized == 0)
    {
        /* error:Driver is not initialized */
        return ARM_DRIVER_ERROR;
    }

    if(CRC->state.powered == 1)
    {
        return ARM_DRIVER_OK;
    }
    /* Set the power state enabled */
    CRC->state.powered = 1;

    break;

    case ARM_POWER_LOW:
    default:
        return ARM_DRIVER_ERROR_UNSUPPORTED;
    }
    return ARM_DRIVER_OK;
}

/**
 @fn           int32_t CRC_Control (uint32_t control,
                                    uint32_t arg,
                                    CRC_RESOURCES *CRC)
 @brief        CMSIS-Driver CRC control.
               Control CRC Interface.
 @param[in]    control : Operation \ref Driver_CRC.h : CRC control codes
 @param[in]    arg     : Argument of operation (optional)
 @param[in]    CRC     : Pointer to CRC resources
 @return       ARM_DRIVER_ERROR_PARAMETER  : if CRC device is invalid
               ARM_DRIVER_OK               : if CRC successfully uninitialized or already not initialized
 */
static int32_t CRC_Control (uint32_t control,
                            uint32_t arg,
                            CRC_RESOURCES *CRC)
{
    int32_t ret = ARM_DRIVER_OK;

    if(CRC->state.initialized == 0)
        return ARM_DRIVER_ERROR;

    if(CRC->state.powered == 0)
        return ARM_DRIVER_ERROR;

    if(control & ARM_CRC_CONTROL_MASK)
    {
        /* To enable or disable the Reflect, Invert, Bit, Byte, Custom polynomial bits of CRC*/
        Control_Bit(control, arg, CRC);
    }
#if RTE_Drivers_DMA
    else if (control & ARM_CRC_ENABLE_DMA)
    {
        if (arg) // Enable DMA
        {
            if (CRC->dma_cfg)
            {
                if (CRC->dma_cfg->dma_handle == -1)
                {
                    ret = CRC->dma_cfg->dma_drv->Initialize();
                    if (ret != ARM_DRIVER_OK) {
                        return ret;
                    }

                    ret = CRC->dma_cfg->dma_drv->PowerControl(ARM_POWER_FULL);
                    if (ret != ARM_DRIVER_OK) {
                        (void)CRC->dma_cfg->dma_drv->Uninitialize();
                        CRC->dma_cfg->dma_handle = -1;
                        return ret;
                    }

                    ret = CRC->dma_cfg->dma_drv->Allocate(&CRC->dma_cfg->dma_handle);
                    if (ret != ARM_DRIVER_OK) {
                        (void)CRC->dma_cfg->dma_drv->PowerControl(ARM_POWER_OFF);
                        (void)CRC->dma_cfg->dma_drv->Uninitialize();
                        CRC->dma_cfg->dma_handle = -1;
                        return ret;
                    }
                    ret = CRC->dma_cfg->dma_drv->Control(&CRC->dma_cfg->dma_handle, ARM_DMA_NO_DEV_HANDSHAKE, 0);
                    if (ret != ARM_DRIVER_OK) {
                        (void)CRC->dma_cfg->dma_drv->DeAllocate(&CRC->dma_cfg->dma_handle);
                        (void)CRC->dma_cfg->dma_drv->PowerControl(ARM_POWER_OFF);
                        (void)CRC->dma_cfg->dma_drv->Uninitialize();
                        CRC->dma_cfg->dma_handle = -1;
                        return ret;
                    }
                    CRC->state.dma_enabled = true;
                }
            } else
            {
                return ARM_DRIVER_ERROR;
            }
        } else
        {   // Disable DMA
            if (CRC->dma_cfg)
            {
                if (CRC->dma_cfg->dma_handle > -1)
                {
                    CRC->state.dma_enabled = false;
                    ret = CRC->dma_cfg->dma_drv->DeAllocate(&CRC->dma_cfg->dma_handle);
                    (void)CRC->dma_cfg->dma_drv->PowerControl(ARM_POWER_OFF);
                    (void)CRC->dma_cfg->dma_drv->Uninitialize(); // DMA Uninitialize returns always ARM_DRIVER_OK
                    CRC->dma_cfg->dma_handle = -1;
                }
            } else
            {
                return ARM_DRIVER_ERROR;
            }
        }
    }
#endif // RTE_Drivers_DMA
    else
    {
        switch (control)
        {
        case ARM_CRC_ALGORITHM_SEL:

            /* clear 8,16 and 32 bit algorithm */
            crc_clear_algo(CRC->regs);

            /* clear the 8, 16, 32 bit algorithm size */
            crc_clear_algo_size(CRC->regs);

            switch(arg)
            {
            case ARM_CRC_ALGORITHM_SEL_8_BIT_CCITT:

                /* To enable 8 bit CRC algorithm and size */
                crc_enable_8bit(CRC->regs);

            break;

            case ARM_CRC_ALGORITHM_SEL_16_BIT:

                /* To enable 16 bit CRC algorithm and size */
                crc_enable_16bit(CRC->regs);

            break;

            case ARM_CRC_ALGORITHM_SEL_16_BIT_CCITT:

                /* To enable 16 bit CCITT CRC algorithm and size */
                crc_enable_16bit_ccitt(CRC->regs);

            break;

            case ARM_CRC_ALGORITHM_SEL_32_BIT:

                /* To enable 32 bit CRC algorithm and size */
                crc_enable_32bit(CRC->regs);

            break;

            case ARM_CRC_ALGORITHM_SEL_32_BIT_CUSTOM_POLY:

                /* To enable 32 bit poly custom CRC algorithm and size */
                crc_enable_32bit_custom_poly(CRC->regs);

            break;
            default:
            ret = ARM_DRIVER_ERROR_UNSUPPORTED;
            }
       break;
       default:
       ret = ARM_DRIVER_ERROR_UNSUPPORTED;
       }
    }
    return ret;
}

/**
@fn         int32_t CRC_Seed (uint32_t value, CRC_RESOURCES *CRC)
@brief      CMSIS-DRIVER CRC Seed value
            Enable the Init bit [0th bit] of the control register to load the seed value
@param[in]  seed_value : Seed value depending on whether the data is 8 bit or 16 or 32 bit
@param[in]  CRC        : pointer to CRC resources
@return     \ref execution_status
*/
static int32_t CRC_Seed (uint32_t seed_value, CRC_RESOURCES *CRC)
{
    int32_t ret = ARM_DRIVER_OK;

    if(CRC->state.powered == 0)
    {
        return ARM_DRIVER_ERROR;
    }

    /* Adding 8 bit or 16 bit or 32 bit seed value to the Seed register of CRC */
    crc_set_seed(CRC->regs, seed_value);

    /* Write the Init value in control register to load the Seed value in Seed register */
    crc_enable(CRC->regs);

    return ret;
}

/**
@fn         int32_t CRC_PolyCustom (uint32_t value, CRC_RESOURCES *CRC)
@brief      To add the polynomial value to polycustom register
            Enable the Init bit [0th bit] of the conrol register to load the
            polynomial value
@param[in]  polynomial : Polynomial data for 8 bit or 16 or 32 bit
@param[in]  CRC        : pointer to CRC resources
@return     \ref execution_status
*/
static int32_t CRC_PolyCustom (uint32_t value, CRC_RESOURCES *CRC)
{
    int32_t ret = ARM_DRIVER_OK;

    if(CRC->state.powered == 0)
    {
        return ARM_DRIVER_ERROR;
    }

    /* Adding Polynomial value to the poly_custom register of CRC */
    crc_set_custom_poly(CRC->regs, value);

    /* Write the Init value in control register to load the polynomial value in Poly_custom register */
    crc_enable(CRC->regs);

    return ret;
}

#if RTE_Drivers_DMA
static int32_t CRC_Compute_DMA (ARM_DMA_PARAMS *params, CRC_RESOURCES *CRC, uint32_t *data_out)
{
    int32_t ret = ARM_DRIVER_OK;
    // CRC->dma_cfg null was tested earlier and this method can't be called if CRC->dma_cfg is null
    ret = CRC->dma_cfg->dma_drv->Start(&CRC->dma_cfg->dma_handle, params);
    if (ret == ARM_DRIVER_OK) {
        while (CRC->dma_cb_val == 0)
        {
            __WFE();
        };
        if (CRC->dma_cb_val != ARM_DMA_EVENT_COMPLETE) {
            ret = ARM_DRIVER_ERROR;
        } else {
            *data_out = (CRC->regs->CRC_OUT);
        }
    }
    return ret;
}
#endif

/**
@fn         int32_t CRC_Compute (void *data_in, uint32_t len, uint32_t *data_out, CRC_RESOURCES *CRC)
@brief      1.To calculate the CRC result for 8 bit 16 bit and 32 bit CRC algorithm.
            2.For 8 bit and 16 bit CRC algorithm our hardware can able to calculate the CRC
              result for both aligned and unaligned CRC input data by loading the CRC inputs
              in DATA_IN_8 bit register.
            3. For 32 bit CRC our hardware will support for aligned data to calculate the CRC Result.
            4. For unaligned data CRC_calculate_Unaligned function will calculate the CRC result for
               unaligned CRC input
            5. In CRC_calculate_Unaligned function load the aligned CRC result from the hardware ,
               unaligned CRC input,length of unaligned input data and the polynomial for the 32 bit CRC
@param[in]  data_in : it is a pointer which holds the address of user input
            len     : Length of the input data
            data_out : To get the CRC output
@param[in]  CRC  : pointer to CRC resources
@return     \ref execution_status
*/
static int32_t CRC_Compute (const void *data_in, uint32_t len, uint32_t *data_out, CRC_RESOURCES *CRC)
{
    int32_t ret = ARM_DRIVER_OK;
    uint32_t custom;
    uint32_t crc_result;
    int32_t algo_size;
    int32_t aligned_length,unaligned_length;
    uint32_t polynomial_status;
#if RTE_Drivers_DMA
    ARM_DMA_PARAMS params;

    if (CRC->state.dma_enabled)
    {
        params.peri_reqno = (int8_t)-1;
        params.dir = ARM_DMA_MEM_TO_DEV;
        params.cb_event   = CRC->dma_cb;
        params.src_addr = data_in;
        params.dst_addr = &CRC->regs->CRC_DATA_IN_8_0;
        params.burst_size = BS_BYTE_1;
        params.burst_len  = 1;
        params.num_bytes  = len;
        CRC->dma_cb_val = 0;
    }
#endif
    if (CRC->state.powered == 0)
    {
         /* error:Driver is not initialized */
         return ARM_DRIVER_ERROR;
    }

    if (data_in == NULL || data_out == NULL || len == 0)
    {
        /* error: pointer is not valid */
        return ARM_DRIVER_ERROR_PARAMETER;
    }

    /* To check whether the algorithm size is 8 bit or 16 or 32 bit */
    algo_size = crc_get_algorithm_size(CRC->regs);

    switch(algo_size)
    {
    /* For 8 bit CRC */
    case CRC_8_BIT_SIZE:

#if RTE_Drivers_DMA
        if (CRC->state.dma_enabled)
        {
            ret = CRC_Compute_DMA(&params, CRC, data_out);
        }
        else
#endif
        {
            crc_calculate_8bit(CRC->regs, data_in, len, data_out);
        }
        break;

    /* For 16 bit CRC */
    case CRC_16_BIT_SIZE:
#if RTE_Drivers_DMA
        if (CRC->state.dma_enabled)
        {
            ret = CRC_Compute_DMA(&params, CRC, data_out);
        }
        else
#endif
        {
            crc_calculate_16bit(CRC->regs, data_in, len, data_out);
        }
        break;

    /* For 32 bit CRC*/
    case CRC_32_BIT_SIZE:
        aligned_length   = len-(len % 4);
        unaligned_length = (len % 4);
#if RTE_Drivers_DMA
        if (CRC->state.dma_enabled)
        {
            params.burst_size = BS_BYTE_4;
            params.dst_addr = &CRC->regs->CRC_DATA_IN_32_0;
            ret = CRC_Compute_DMA(&params, CRC, data_out);
            *data_out = ~(CRC->regs->CRC_OUT);
        }
        else
#endif
        {
            crc_calculate_32bit(CRC->regs, data_in, len, data_out);
        }

        polynomial_status = crc_custom_poly_enabled(CRC->regs);

        /* Check for the custom polynomial bit  */
        if (polynomial_status)
        {
            /* assign the user polynomial */
            custom = crc_get_custom_poly(CRC->regs);
            *data_out = ~(*data_out);

            /* reflect the output */
            *data_out = crc_bit_reflect(*data_out);

            /* Complement the output */
            *data_out = ~(*data_out);
        }
        else
        {
            /* Assign the 32 bit CRC standard polynomial */
            custom = CRC_STANDARD_POLY;
        }

        /* Calculate the CRC for unaligned data */
        crc_result = crc_calculate_unaligned(*data_out, data_in + aligned_length, unaligned_length, custom);

        *data_out = crc_result;  /* Store the CRC result */

        break;
    }
    return ret;
}


#if RTE_Drivers_DMA
/**
  \fn          static void  CRC_DMACallback (uint32_t event, int8_t peri_num, CRC_RESOURCES *CRC)
  \brief       Callback function from DMA for CRC
  \param[in]   event     Event from DMA
  \param[in]   peri_num  Peripheral number
  \param[in]   CRC       Pointer to CRC resources
*/
static void CRC_DMACallback (uint32_t event, int8_t peri_num, CRC_RESOURCES *CRC)
{
    (void)peri_num;
    CRC->dma_cb_val = event;
}
#endif

/* CRC0 Driver instance */
#if (RTE_CRC0)
#if RTE_Drivers_DMA

#ifndef CRC0_DMA
#define CRC0_DMA 0
#endif

static void CRC0_DMACallback (uint32_t event, int8_t peri_num);

#if defined CRC0_DMA && ((CRC0_DMA == 0 && (RTE_DMA0)) || (CRC0_DMA == 1 && (RTE_DMA1)) || (CRC0_DMA == 2 && (RTE_DMA2)))
static DMA_PERIPHERAL_CONFIG CRC0_DMA_CONFIG = {
    .dma_drv            = &ARM_Driver_DMA_(CRC0_DMA),
    .dma_periph_req     = -1,
    .dma_handle         = -1
};
#endif
#endif // RTE_Drivers_DMA

static CRC_RESOURCES CRC0_RES = {
    .regs          = (CRC_Type*) CRC0_BASE,
    .state         = {},
#if RTE_Drivers_DMA
#if defined CRC0_DMA && ((CRC0_DMA == 0 && (RTE_DMA0)) || (CRC0_DMA == 1 && (RTE_DMA1)) || (CRC0_DMA == 2 && (RTE_DMA2)))
    .dma_cfg       = &CRC0_DMA_CONFIG,
    .dma_cb        = CRC0_DMACallback,
#endif
#endif
};

/* Function Name: CRC0_Initialize */
static int32_t CRC0_Initialize(void)
{
    return (CRC_Initialize(&CRC0_RES));
}

/* Function Name: CRC0_Uninitialize */
static int32_t CRC0_Uninitialize(void)
{
    return (CRC_Uninitialize(&CRC0_RES));
}

/* Function Name: CRC0_PowerControl */
static int32_t CRC0_PowerControl(ARM_POWER_STATE status)
{
    return (CRC_PowerControl(status, &CRC0_RES));
}

/* Function Name: CRC0_Control */
static int32_t CRC0_Control(uint32_t control, uint32_t arg)
{
    return (CRC_Control(control, arg, &CRC0_RES));
}

/* Function Name: CRC0_Seed */
static int32_t CRC0_Seed(uint32_t value)
{
    return (CRC_Seed(value, &CRC0_RES));
}

/* Function Name: CRC0_PolyCustom */
static int32_t CRC0_PolyCustom(uint32_t value)
{
    return (CRC_PolyCustom(value, &CRC0_RES));
}

/* Function Name: CRC0_Compute */
static int32_t CRC0_Compute(const void *data_in, uint32_t len, uint32_t *data_out)
{
    return (CRC_Compute(data_in, len, data_out, &CRC0_RES));
}

#if RTE_Drivers_DMA
/**
  \fn          static void  CRC0_DMACallback (uint32_t event, int8_t peri_num)
  \param[in]   event     Event from DMA
  \param[in]   peri_num  Peripheral number
  \brief       Callback function from DMA for CRC0
*/
static void CRC0_DMACallback (uint32_t event, int8_t peri_num)
{
    CRC_DMACallback (event, peri_num, &CRC0_RES);
}
#endif
extern ARM_DRIVER_CRC Driver_CRC0;
ARM_DRIVER_CRC Driver_CRC0 = {
    CRC_GetVersion,
    CRC_GetCapabilities,
    CRC0_Initialize,
    CRC0_Uninitialize,
    CRC0_PowerControl,
    CRC0_Control,
    CRC0_Seed,
    CRC0_PolyCustom,
    CRC0_Compute,
};

#endif /* RTE_CRC0 */

/* CRC1 driver instance */
#if (RTE_CRC1)
#if RTE_Drivers_DMA

#ifndef CRC1_DMA
#define CRC1_DMA 0
#endif

static void CRC1_DMACallback (uint32_t event, int8_t peri_num);

#if defined CRC1_DMA && ((CRC1_DMA == 0 && (RTE_DMA0)) || (CRC1_DMA == 1 && (RTE_DMA1)) || (CRC1_DMA == 2 && (RTE_DMA2)))
static DMA_PERIPHERAL_CONFIG CRC1_DMA_CONFIG = {
    .dma_drv            = &ARM_Driver_DMA_(CRC1_DMA),
    .dma_periph_req     = -1,
    .dma_handle         = -1
};
#endif
#endif // RTE_Drivers_DMA

static CRC_RESOURCES CRC1_RES = {
    .regs          = (CRC_Type*) CRC1_BASE,
    .state         = {},
#if RTE_Drivers_DMA
#if defined CRC1_DMA && ((CRC1_DMA == 0 && (RTE_DMA0)) || (CRC1_DMA == 1 && (RTE_DMA1)) || (CRC1_DMA == 2 && (RTE_DMA2)))
    .dma_cfg       = &CRC1_DMA_CONFIG,
    .dma_cb        = CRC1_DMACallback,
#endif
#endif
};

/* Function Name: CRC1_Initialize */
static int32_t CRC1_Initialize(void)
{
    return (CRC_Initialize(&CRC1_RES));
}

/* Function Name: CRC1_Uninitialize */
static int32_t CRC1_Uninitialize(void)
{
    return (CRC_Uninitialize(&CRC1_RES));
}

/* Function Name: CRC1_PowerControl */
static int32_t CRC1_PowerControl(ARM_POWER_STATE status)
{
    return (CRC_PowerControl(status, &CRC1_RES));
}

/* Function Name: CRC1_Control */
static int32_t CRC1_Control(uint32_t control, uint32_t arg)
{
    return (CRC_Control(control, arg, &CRC1_RES));
}

/* Function Name: CRC1_Seed */
static int32_t CRC1_Seed(uint32_t value)
{
    return (CRC_Seed(value, &CRC1_RES));
}

/* Function Name: CRC1_PolyCustom */
static int32_t CRC1_PolyCustom(uint32_t value)
{
    return (CRC_PolyCustom(value, &CRC1_RES));
}

/* Function Name: CRC1_Compute */
static int32_t CRC1_Compute(const void *data_in, uint32_t len, uint32_t *data_out)
{
    return (CRC_Compute(data_in, len, data_out, &CRC1_RES));
}

#if RTE_Drivers_DMA
/**
  \fn          static void  CRC1_DMACallback (uint32_t event, int8_t peri_num)
  \param[in]   event     Event from DMA
  \param[in]   peri_num  Peripheral number
  \brief       Callback function from DMA for CRC0
*/
static void CRC1_DMACallback (uint32_t event, int8_t peri_num)
{
    CRC_DMACallback (event, peri_num, &CRC1_RES);
}
#endif
extern ARM_DRIVER_CRC Driver_CRC1;
ARM_DRIVER_CRC Driver_CRC1 = {
    CRC_GetVersion,
    CRC_GetCapabilities,
    CRC1_Initialize,
    CRC1_Uninitialize,
    CRC1_PowerControl,
    CRC1_Control,
    CRC1_Seed,
    CRC1_PolyCustom,
    CRC1_Compute
};

#endif /* RTE_CRC1 */
