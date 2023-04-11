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
 * @file     Driver_HWSEM.c
 * @author   Khushboo Singh
 * @email    khushboo.singh@alifsemi.com
 * @version  V1.0.0
 * @date     16-June-2022
 * @brief    ARM CMSIS-Driver for Hardware Semaphore.
 * @bug      None.
 * @Note     None.
 ******************************************************************************/

/* Driver specific include */
#include "hwsem.h"
#include "Driver_HWSEM_Private.h"

#define HWSEM ((HWSEM_Type *) HWSEM_BASE)

#define ARM_HWSEM_DRV_VERSION    ARM_DRIVER_VERSION_MAJOR_MINOR(1, 0) /* Driver version */

/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion = {
    ARM_HWSEM_API_VERSION,
    ARM_HWSEM_DRV_VERSION
};

/* Driver Capabilities */
static const ARM_HWSEM_CAPABILITIES DriverCapabilities =
{
    1,         /* Supports HWSEM callback */
    0
};


/**
  \fn           static ARM_DRIVER_VERSION Hwsem_GetVersion(void)
  \brief        Returns the Driver Version
  \param[in]    void
  \return       ARM_DRIVER_VERSION : Driver version
 */
static ARM_DRIVER_VERSION Hwsem_GetVersion(void)
{
    return DriverVersion;
}

/**
  \fn           static ARM_HWSEM_CAPABILITIES Hwsem_GetCapabilities(void)
  \brief        Returns the Driver Capabilities
  \param[in]    void
  \return       ARM_HWSEM_CAPABILITIES : Driver Capabilities
 */
static ARM_HWSEM_CAPABILITIES Hwsem_GetCapabilities(void)
{
    return DriverCapabilities;
}

/**
  \fn           static int32_t Initialize(ARM_HWSEM_SignalEvent_t cb_event,
                uint8_t HWSEM_RESOURCES *hwsem)
  \brief        Initializes the driver instance with semaphore id and call back event
  \param[in]    cb_event : call back function provided by user
  \param[in]    sem_id   : Semaphore id provided by user
  \param[in]    hwsem    : Pointer to Resources of the particular driver instance
  \return       ARM_DRIVER_OK : If no error is there
  \             ARM_DRIVER_ERROR_BUSY : Already initialized
 */
static int32_t Initialize(ARM_HWSEM_SignalEvent_t cb_event, HWSEM_RESOURCES *hwsem)
{
    /* Confirm that driver is not already in use */
    if (hwsem->state.initialized)
    {
        return ARM_DRIVER_ERROR_BUSY;
    }

    if (cb_event != NULL)
    {
        hwsem->cb_event = cb_event;
    }

    hwsem->state.initialized = 1;

    return ARM_DRIVER_OK;
}

/**
  \fn           static int32_t Uninitialize(uint8_t HWSEM_RESOURCES *hwsem)
  \brief        Uninitializes the driver instance
  \param[in]    sem_id   : Semaphore id provided by user
  \param[in]    hwsem    : Pointer to Resources of the particular driver instance
  \return       ARM_DRIVER_OK : If no error, otherwise ARM_DRIVER_ERROR_PARAMETER
 */
static int32_t Uninitialize(HWSEM_RESOURCES *hwsem)
{
    /* Confirm that driver is initialized */
    if (!(hwsem->state.initialized))
    {
        return ARM_DRIVER_ERROR;
    }

    hwsem->cb_event = NULL;

    hwsem->state.initialized = 0;

    return ARM_DRIVER_OK;
}

/**
  \fn           static int32_t Lock(uint8_t HWSEM_RESOURCES *hwsem)
  \brief        Acquire lock of the semaphore
  \param[in]    sem_id   : Semaphore id provided by user
  \param[in]    hwsem    : Pointer to Resources of the particular driver instance
  \return       ARM_DRIVER_OK : If no error
  \             ARM_DRIVER_ERROR_BUSY : if semaphore is not available
  \             ARM_DRIVER_ERROR : If Driver is not initialized
 */
static int32_t Lock(HWSEM_RESOURCES *hwsem)
{
    /* Confirm that driver is initialized */
    if (!(hwsem->state.initialized))
    {
        return ARM_DRIVER_ERROR;
    }

    if (hwsem_request(HWSEM, hwsem->sem_id, HWSEM_MASTERID) == HWSEM_MASTERID)
    {
        return ARM_DRIVER_OK;
    }

    /* If the semaphore is not available, enable the IRQ */
    if (hwsem->cb_event != NULL)
    {
        /* Enable Hw Sem IRQ*/
        NVIC_ClearPendingIRQ(hwsem->irq);
        NVIC_SetPriority(hwsem->irq, hwsem->irq_priority);
        NVIC_EnableIRQ(hwsem->irq);
    }
    return ARM_DRIVER_ERROR_BUSY;
}

/**
  \fn           static int32_t UnLock(uint8_t HWSEM_RESOURCES *hwsem)
  \brief        Release the semaphore
  \param[in]    sem_id   : Semaphore id provided by user
  \param[in]    hwsem    : Pointer to Resources of the particular driver instance
  \return       ARM_DRIVER_OK : If no error
  \             ARM_DRIVER_ERROR : if semaphore id is not same or driver is not initialized
 */
static int32_t UnLock(HWSEM_RESOURCES *hwsem)
{
    /* Confirm that driver is initialized */
    if (!(hwsem->state.initialized))
    {
        return ARM_DRIVER_ERROR;
    }

    /* Check if semaphore is locked */
    if (hwsem_getcount(HWSEM, hwsem->sem_id) > 0)
    {
        /* Release the semaphore */
        hwsem_release(HWSEM, hwsem->sem_id, HWSEM_MASTERID);

        return ARM_DRIVER_OK;
    }
    else
    {
        return ARM_DRIVER_ERROR;
    }
}

/**
  \fn           static uint32_t GetCount(uint8_t HWSEM_RESOURCES *hwsem)
  \brief        Get the semaphore count
  \param[in]    hwsem      : Pointer to Resources of the particular driver instance
  \return       Semaphore count : If no error
  \             ARM_DRIVER_ERROR : if driver is not initialized
 */
static int32_t GetCount(HWSEM_RESOURCES *hwsem)
{
    /* Confirm that driver is initialized */
    if (!(hwsem->state.initialized))
    {
        return ARM_DRIVER_ERROR;
    }

    return hwsem_getcount(HWSEM, hwsem->sem_id);
}

/**
  \fn          void HWSEM_IRQHandler(HWSEM_RESOURCES *hwsem)
  \brief       HWSEM instance specific part of Hw Semaphore Interrupt handler.
  \param[in]   hwsem    :   Pointer to the HWSEM device instance
 */
static void ARM_HWSEM_IRQHandler(HWSEM_RESOURCES *hwsem)
{
    /* Disable Hw Sem IRQ*/
    NVIC_DisableIRQ(hwsem->irq);
    NVIC_ClearPendingIRQ(hwsem->irq);

    /* Call the user provided call back function */
    hwsem->cb_event(HWSEM_AVAILABLE_CB_EVENT, hwsem->sem_id);
}

/* HWSEM0 Driver Instance */
#if (RTE_HWSEM0)

/* HWSEM0 Resources */
static HWSEM_RESOURCES HWSEM0 = {
    .cb_event = NULL,
    .irq =(IRQn_Type) HWSEM_IRQ0_IRQn,
    .irq_priority =(uint8_t)RTE_HWSEM0_IRQPRIORITY,
    .sem_id = HWSEMID0,
};

/* HWSEM0 Interrupt Handler */
void HWSEM_IRQ0Handler(void)
{
    ARM_HWSEM_IRQHandler(&HWSEM0);
}

static int32_t Hwsem0_Initialize(ARM_HWSEM_SignalEvent_t cb_event)
{
    return Initialize(cb_event, &HWSEM0);
}

static int32_t Hwsem0_Uninitialize(void)
{
    return Uninitialize(&HWSEM0);
}

static int32_t Hwsem0_Lock(void)
{
    return Lock(&HWSEM0);
}

static int32_t Hwsem0_UnLock(void)
{
    return UnLock(&HWSEM0);
}

static int32_t Hwsem0_GetCount(void)
{
    return GetCount(&HWSEM0);
}

/* HWSEM0 Access Struct */
ARM_DRIVER_HWSEM DRIVER_HWSEM0 =
{
    Hwsem_GetVersion,
    Hwsem_GetCapabilities,
    Hwsem0_Initialize ,
    Hwsem0_Uninitialize,
    Hwsem0_Lock,
    Hwsem0_UnLock,
    Hwsem0_GetCount
};

#endif

/* HSWEM1 Driver Instance */
#if (RTE_HWSEM1)

/* HWSEM1 Resources */
static HWSEM_RESOURCES HWSEM1 = {
    .cb_event = NULL,
    .irq =(IRQn_Type) HWSEM_IRQ1_IRQn,
    .irq_priority =(uint8_t) RTE_HWSEM1_IRQPRIORITY,
    .sem_id = HWSEMID1,
};

/* HWSEM1 Interrupt Handler */
void HWSEM_IRQ1Handler(void)
{
    ARM_HWSEM_IRQHandler(&HWSEM1);
}

static int32_t Hwsem1_Initialize(ARM_HWSEM_SignalEvent_t cb_event)
{
    return Initialize(cb_event, &HWSEM1);
}

static int32_t Hwsem1_Uninitialize(void)
{
    return Uninitialize(&HWSEM1);
}

static int32_t Hwsem1_Lock(void)
{
    return Lock(&HWSEM1);
}

static int32_t Hwsem1_UnLock(void)
{
    return UnLock(&HWSEM1);
}

static int32_t Hwsem1_GetCount(void)
{
    return GetCount(&HWSEM1);
}

/* HWSEM1 Access Struct */
ARM_DRIVER_HWSEM DRIVER_HWSEM1 =
{
    Hwsem_GetVersion,
    Hwsem_GetCapabilities,
    Hwsem1_Initialize ,
    Hwsem1_Uninitialize,
    Hwsem1_Lock,
    Hwsem1_UnLock,
    Hwsem1_GetCount
};

#endif

/* HWSEM2 Driver Instance */
#if (RTE_HWSEM2)

/* HWSEM2 Resources */
static HWSEM_RESOURCES HWSEM2 = {
    .cb_event = NULL,
    .irq =(IRQn_Type) HWSEM_IRQ2_IRQn,
    .irq_priority =(uint8_t) RTE_HWSEM2_IRQPRIORITY,
    .sem_id = HWSEMID2,
};

/* HWSEM2 Interrupt Handler */
void HWSEM_IRQ2Handler(void)
{
    ARM_HWSEM_IRQHandler(&HWSEM2);
}

static int32_t Hwsem2_Initialize(ARM_HWSEM_SignalEvent_t cb_event)
{
    return Initialize(cb_event, &HWSEM2);
}

static int32_t Hwsem2_Uninitialize(void)
{
    return Uninitialize(&HWSEM2);
}

static int32_t Hwsem2_Lock(void)
{
    return Lock(&HWSEM2);
}

static int32_t Hwsem2_UnLock(void)
{
    return UnLock(&HWSEM2);
}

static int32_t Hwsem2_GetCount(void)
{
    return GetCount(&HWSEM2);
}

/* HWSEM2 Access Struct */
ARM_DRIVER_HWSEM DRIVER_HWSEM2 =
{
    Hwsem_GetVersion,
    Hwsem_GetCapabilities,
    Hwsem2_Initialize ,
    Hwsem2_Uninitialize,
    Hwsem2_Lock,
    Hwsem2_UnLock,
    Hwsem2_GetCount
};

#endif

/* HWSEM3 Driver Instance */
#if (RTE_HWSEM3)

/* HWSEM3 Resources */
static HWSEM_RESOURCES HWSEM3 = {
    .cb_event = NULL,
    .irq =(IRQn_Type) HWSEM_IRQ3_IRQn,
    .irq_priority =(uint8_t) RTE_HWSEM3_IRQPRIORITY,
    .sem_id = HWSEMID3,
};

/* HWSEM3 Interrupt Handler */
void HWSEM_IRQ3Handler(void)
{
    ARM_HWSEM_IRQHandler(&HWSEM3);
}

static int32_t Hwsem3_Initialize(ARM_HWSEM_SignalEvent_t cb_event)
{
    return Initialize(cb_event, &HWSEM3);
}

static int32_t Hwsem3_Uninitialize(void)
{
    return Uninitialize(&HWSEM3);
}

static int32_t Hwsem3_Lock(void)
{
    return Lock(&HWSEM3);
}

static int32_t Hwsem3_UnLock(void)
{
    return UnLock(&HWSEM3);
}

static int32_t Hwsem3_GetCount(void)
{
    return GetCount(&HWSEM3);
}

/* HWSEM3 Access Struct */
ARM_DRIVER_HWSEM DRIVER_HWSEM3 =
{
    Hwsem_GetVersion,
    Hwsem_GetCapabilities,
    Hwsem3_Initialize ,
    Hwsem3_Uninitialize,
    Hwsem3_Lock,
    Hwsem3_UnLock,
    Hwsem3_GetCount
};

#endif

/* HWSEM4 Driver Instance */
#if (RTE_HWSEM4)

/* HWSEM4 Resources */
static HWSEM_RESOURCES HWSEM4 = {
    .cb_event = NULL,
    .irq =(IRQn_Type) HWSEM_IRQ4_IRQn,
    .irq_priority =(uint8_t) RTE_HWSEM4_IRQPRIORITY,
    .sem_id = HWSEMID4,
};

/* HWSEM4 Interrupt Handler */
void HWSEM_IRQ4Handler(void)
{
    ARM_HWSEM_IRQHandler(&HWSEM4);
}

static int32_t Hwsem4_Initialize(ARM_HWSEM_SignalEvent_t cb_event)
{
    return Initialize(cb_event, &HWSEM4);
}

static int32_t Hwsem4_Uninitialize(void)
{
    return Uninitialize(&HWSEM4);
}

static int32_t Hwsem4_Lock(void)
{
    return Lock(&HWSEM4);
}

static int32_t Hwsem4_UnLock(void)
{
    return UnLock(&HWSEM4);
}

static int32_t Hwsem4_GetCount(void)
{
    return GetCount(&HWSEM4);
}

/* HWSEM4 Access Struct */
ARM_DRIVER_HWSEM DRIVER_HWSEM4 =
{
    Hwsem_GetVersion,
    Hwsem_GetCapabilities,
    Hwsem4_Initialize ,
    Hwsem4_Uninitialize,
    Hwsem4_Lock,
    Hwsem4_UnLock,
    Hwsem4_GetCount
};

#endif

/* HWSEM5 Driver Instance */
#if (RTE_HWSEM5)

/* HWSEM5 Resources */
static HWSEM_RESOURCES HWSEM5 = {
    .cb_event = NULL,
    .irq =(IRQn_Type) HWSEM_IRQ5_IRQn,
    .irq_priority =(uint8_t) RTE_HWSEM5_IRQPRIORITY,
    .sem_id = HWSEMID5,
};

/* HWSEM5 Interrupt Handler */
void HWSEM_IRQ5Handler(void)
{
    ARM_HWSEM_IRQHandler(&HWSEM5);
}

static int32_t Hwsem5_Initialize(ARM_HWSEM_SignalEvent_t cb_event)
{
    return Initialize(cb_event, &HWSEM5);
}

static int32_t Hwsem5_Uninitialize(void)
{
    return Uninitialize(&HWSEM5);
}

static int32_t Hwsem5_Lock(void)
{
    return Lock(&HWSEM5);
}

static int32_t Hwsem5_UnLock(void)
{
    return UnLock(&HWSEM5);
}

static int32_t Hwsem5_GetCount(void)
{
    return GetCount(&HWSEM5);
}

/* HWSEM5 Access Struct */
ARM_DRIVER_HWSEM DRIVER_HWSEM5 =
{
    Hwsem_GetVersion,
    Hwsem_GetCapabilities,
    Hwsem5_Initialize ,
    Hwsem5_Uninitialize,
    Hwsem5_Lock,
    Hwsem5_UnLock,
    Hwsem5_GetCount
};

#endif

/* HWSEM6 Driver Instance */
#if (RTE_HWSEM6)

/* HWSEM6 Resources */
static HWSEM_RESOURCES HWSEM6 = {
    .cb_event = NULL,
    .irq =(IRQn_Type) HWSEM_IRQ6_IRQn,
    .irq_priority =(uint8_t) RTE_HWSEM6_IRQPRIORITY,
    .sem_id = HWSEMID6,
};

/* HWSEM6 Interrupt Handler */
void HWSEM_IRQ6Handler(void)
{
    ARM_HWSEM_IRQHandler(&HWSEM6);
}

static int32_t Hwsem6_Initialize(ARM_HWSEM_SignalEvent_t cb_event)
{
    return Initialize(cb_event, &HWSEM6);
}

static int32_t Hwsem6_Uninitialize(void)
{
    return Uninitialize(&HWSEM6);
}

static int32_t Hwsem6_Lock(void)
{
    return Lock(&HWSEM6);
}

static int32_t Hwsem6_UnLock(void)
{
    return UnLock(&HWSEM6);
}

static int32_t Hwsem6_GetCount(void)
{
    return GetCount(&HWSEM6);
}

/* HWSEM6 Access Struct */
ARM_DRIVER_HWSEM DRIVER_HWSEM6 =
{
    Hwsem_GetVersion,
    Hwsem_GetCapabilities,
    Hwsem6_Initialize ,
    Hwsem6_Uninitialize,
    Hwsem6_Lock,
    Hwsem6_UnLock,
    Hwsem6_GetCount
};

#endif

/* HWSEM0 Driver Instance */
#if (RTE_HWSEM7)

/* HWSEM7 Resources */
static HWSEM_RESOURCES HWSEM7 = {
    .cb_event = NULL,
    .irq =(IRQn_Type) HWSEM_IRQ7_IRQn,
    .irq_priority =(uint8_t) RTE_HWSEM7_IRQPRIORITY,
    .sem_id = HWSEMID7,
};

/* HWSEM7 Interrupt Handler */
void HWSEM_IRQ7Handler(void)
{
    ARM_HWSEM_IRQHandler(&HWSEM7);
}

static int32_t Hwsem7_Initialize(ARM_HWSEM_SignalEvent_t cb_event)
{
    return Initialize(cb_event, &HWSEM7);
}

static int32_t Hwsem7_Uninitialize(void)
{
    return Uninitialize(&HWSEM7);
}

static int32_t Hwsem7_Lock(void)
{
    return Lock(&HWSEM7);
}

static int32_t Hwsem7_UnLock(void)
{
    return UnLock(&HWSEM7);
}

static int32_t Hwsem7_GetCount(void)
{
    return GetCount(&HWSEM7);
}

/* HWSEM7 Access Struct */
ARM_DRIVER_HWSEM DRIVER_HWSEM7 =
{
    Hwsem_GetVersion,
    Hwsem_GetCapabilities,
    Hwsem7_Initialize ,
    Hwsem7_Uninitialize,
    Hwsem7_Lock,
    Hwsem7_UnLock,
    Hwsem7_GetCount
};

#endif

/* HWSEM8 Driver Instance */
#if (RTE_HWSEM8)

/* HWSEM8 Resources */
static HWSEM_RESOURCES HWSEM8 = {
    .cb_event = NULL,
    .irq =(IRQn_Type) HWSEM_IRQ8_IRQn,
    .irq_priority =(uint8_t) RTE_HWSEM8_IRQPRIORITY,
    .sem_id = HWSEMID8,
};

/* HWSEM8 Interrupt Handler */
void HWSEM_IRQ8Handler(void)
{
    ARM_HWSEM_IRQHandler(&HWSEM8);
}

static int32_t Hwsem8_Initialize(ARM_HWSEM_SignalEvent_t cb_event)
{
    return Initialize(cb_event, &HWSEM8);
}

static int32_t Hwsem8_Uninitialize(void)
{
    return Uninitialize(&HWSEM8);
}

static int32_t Hwsem8_Lock(void)
{
    return Lock(&HWSEM8);
}

static int32_t Hwsem8_UnLock(void)
{
    return UnLock(&HWSEM8);
}

static int32_t Hwsem8_GetCount(void)
{
    return GetCount(&HWSEM8);
}

/* HWSEM8 Access Struct */
ARM_DRIVER_HWSEM DRIVER_HWSEM8 =
{
    Hwsem_GetVersion,
    Hwsem_GetCapabilities,
    Hwsem8_Initialize ,
    Hwsem8_Uninitialize,
    Hwsem8_Lock,
    Hwsem8_UnLock,
    Hwsem8_GetCount
};

#endif

/* HWSEM9 Driver Instance */
#if (RTE_HWSEM9)

/* HWSEM9 Resources */
static HWSEM_RESOURCES HWSEM9 = {
    .cb_event = NULL,
    .irq =(IRQn_Type) HWSEM_IRQ9_IRQn,
    .irq_priority =(uint8_t) RTE_HWSEM9_IRQPRIORITY,
    .sem_id = HWSEMID9,
};

/* HWSEM9 Interrupt Handler */
void HWSEM_IRQ9Handler(void)
{
    ARM_HWSEM_IRQHandler(&HWSEM9);
}

static int32_t Hwsem9_Initialize(ARM_HWSEM_SignalEvent_t cb_event)
{
    return Initialize(cb_event, &HWSEM9);
}

static int32_t Hwsem9_Uninitialize(void)
{
    return Uninitialize(&HWSEM9);
}

static int32_t Hwsem9_Lock(void)
{
    return Lock(&HWSEM9);
}

static int32_t Hwsem9_UnLock(void)
{
    return UnLock(&HWSEM9);
}

static int32_t Hwsem9_GetCount(void)
{
    return GetCount(&HWSEM9);
}

/* HWSEM9 Access Struct */
ARM_DRIVER_HWSEM DRIVER_HWSEM9 =
{
    Hwsem_GetVersion,
    Hwsem_GetCapabilities,
    Hwsem9_Initialize ,
    Hwsem9_Uninitialize,
    Hwsem9_Lock,
    Hwsem9_UnLock,
    Hwsem9_GetCount
};

#endif

/* HWSEM10 Driver Instance */
#if (RTE_HWSEM10)

/* HWSEM10 Resources */
static HWSEM_RESOURCES HWSEM10 = {
    .cb_event = NULL,
    .irq =(IRQn_Type) HWSEM_IRQ10_IRQn,
    .irq_priority =(uint8_t) RTE_HWSEM10_IRQPRIORITY,
    .sem_id = HWSEMID10,
};

/* HWSEM10 Interrupt Handler */
void HWSEM_IRQ10Handler(void)
{
    ARM_HWSEM_IRQHandler(&HWSEM10);
}

static int32_t Hwsem10_Initialize(ARM_HWSEM_SignalEvent_t cb_event)
{
    return Initialize(cb_event, &HWSEM10);
}

static int32_t Hwsem10_Uninitialize(void)
{
    return Uninitialize(&HWSEM10);
}

static int32_t Hwsem10_Lock(void)
{
    return Lock(&HWSEM10);
}

static int32_t Hwsem10_UnLock(void)
{
    return UnLock(&HWSEM10);
}

static int32_t Hwsem10_GetCount(void)
{
    return GetCount(&HWSEM10);
}

/* HWSEM10 Access Struct */
ARM_DRIVER_HWSEM DRIVER_HWSEM10 =
{
    Hwsem_GetVersion,
    Hwsem_GetCapabilities,
    Hwsem10_Initialize ,
    Hwsem10_Uninitialize,
    Hwsem10_Lock,
    Hwsem10_UnLock,
    Hwsem10_GetCount
};

#endif

/* HWSEM11 Driver Instance */
#if (RTE_HWSEM11)

/* HWSEM11 Resources */
static HWSEM_RESOURCES HWSEM11 = {
    .cb_event = NULL,
    .irq =(IRQn_Type) HWSEM_IRQ11_IRQn,
    .irq_priority =(uint8_t) RTE_HWSEM11_IRQPRIORITY,
    .sem_id = HWSEMID11,
};

/* HWSEM11 Interrupt Handler */
void HWSEM_IRQ11Handler(void)
{
    ARM_HWSEM_IRQHandler(&HWSEM11);
}

static int32_t Hwsem11_Initialize(ARM_HWSEM_SignalEvent_t cb_event)
{
    return Initialize(cb_event, &HWSEM11);
}

static int32_t Hwsem11_Uninitialize(void)
{
    return Uninitialize(&HWSEM11);
}

static int32_t Hwsem11_Lock(void)
{
    return Lock(&HWSEM11);
}

static int32_t Hwsem11_UnLock(void)
{
    return UnLock(&HWSEM11);
}

static int32_t Hwsem11_GetCount(void)
{
    return GetCount(&HWSEM11);
}

/* HWSEM11 Access Struct */
ARM_DRIVER_HWSEM DRIVER_HWSEM11 =
{
    Hwsem_GetVersion,
    Hwsem_GetCapabilities,
    Hwsem11_Initialize ,
    Hwsem11_Uninitialize,
    Hwsem11_Lock,
    Hwsem11_UnLock,
    Hwsem11_GetCount
};

#endif

/* HWSEM12 Driver Instance */
#if (RTE_HWSEM12)

/* HWSEM12 Resources */
static HWSEM_RESOURCES HWSEM12 = {
    .cb_event = NULL,
    .irq =(IRQn_Type) HWSEM_IRQ12_IRQn,
    .irq_priority =(uint8_t) RTE_HWSEM12_IRQPRIORITY,
    .sem_id = HWSEMID12,
};

/* HWSEM12 Interrupt Handler */
void HWSEM_IRQ12Handler(void)
{
    ARM_HWSEM_IRQHandler(&HWSEM12);
}

static int32_t Hwsem12_Initialize(ARM_HWSEM_SignalEvent_t cb_event)
{
    return Initialize(cb_event, &HWSEM12);
}

static int32_t Hwsem12_Uninitialize(void)
{
    return Uninitialize(&HWSEM12);
}

static int32_t Hwsem12_Lock(void)
{
    return Lock(&HWSEM12);
}

static int32_t Hwsem12_UnLock(void)
{
    return UnLock(&HWSEM12);
}

static int32_t Hwsem12_GetCount(void)
{
    return GetCount(&HWSEM12);
}

/* HWSEM12 Access Struct */
ARM_DRIVER_HWSEM DRIVER_HWSEM12 =
{
    Hwsem_GetVersion,
    Hwsem_GetCapabilities,
    Hwsem12_Initialize ,
    Hwsem12_Uninitialize,
    Hwsem12_Lock,
    Hwsem12_UnLock,
    Hwsem12_GetCount
};

#endif

/* HWSEM13 Driver Instance */
#if (RTE_HWSEM13)

/* HWSEM13 Resources */
static HWSEM_RESOURCES HWSEM13 = {
    .cb_event = NULL,
    .irq =(IRQn_Type) HWSEM_IRQ13_IRQn,
    .irq_priority =(uint8_t) RTE_HWSEM13_IRQPRIORITY,
    .sem_id = HWSEMID13,
};

/* HWSEM13 Interrupt Handler */
void HWSEM_IRQ13Handler(void)
{
    ARM_HWSEM_IRQHandler(&HWSEM13);
}

static int32_t Hwsem13_Initialize(ARM_HWSEM_SignalEvent_t cb_event)
{
    return Initialize(cb_event, &HWSEM13);
}

static int32_t Hwsem13_Uninitialize(void)
{
    return Uninitialize(&HWSEM13);
}

static int32_t Hwsem13_Lock(void)
{
    return Lock(&HWSEM13);
}

static int32_t Hwsem13_UnLock(void)
{
    return UnLock(&HWSEM13);
}

static int32_t Hwsem13_GetCount(void)
{
    return GetCount(&HWSEM13);
}

/* HWSEM13 Access Struct */
ARM_DRIVER_HWSEM DRIVER_HWSEM13 =
{
    Hwsem_GetVersion,
    Hwsem_GetCapabilities,
    Hwsem13_Initialize ,
    Hwsem13_Uninitialize,
    Hwsem13_Lock,
    Hwsem13_UnLock,
    Hwsem13_GetCount
};

#endif

/* HWSEM14 Driver Instance */
#if (RTE_HWSEM14)

/* HWSEM14 Resources */
static HWSEM_RESOURCES HWSEM14 = {
    .cb_event = NULL,
    .irq =(IRQn_Type) HWSEM_IRQ14_IRQn,
    .irq_priority =(uint8_t) RTE_HWSEM14_IRQPRIORITY,
    .sem_id = HWSEMID14,
};

/* HWSEM14 Interrupt Handler */
void HWSEM_IRQ14Handler(void)
{
    ARM_HWSEM_IRQHandler(&HWSEM14);
}

static int32_t Hwsem14_Initialize(ARM_HWSEM_SignalEvent_t cb_event)
{
    return Initialize(cb_event, &HWSEM14);
}

static int32_t Hwsem14_Uninitialize(void)
{
    return Uninitialize(&HWSEM14);
}

static int32_t Hwsem14_Lock(void)
{
    return Lock(&HWSEM14);
}

static int32_t Hwsem14_UnLock(void)
{
    return UnLock(&HWSEM14);
}

static int32_t Hwsem14_GetCount(void)
{
    return GetCount(&HWSEM14);
}

/* HWSEM14 Access Struct */
ARM_DRIVER_HWSEM DRIVER_HWSEM14 =
{
    Hwsem_GetVersion,
    Hwsem_GetCapabilities,
    Hwsem14_Initialize ,
    Hwsem14_Uninitialize,
    Hwsem14_Lock,
    Hwsem14_UnLock,
    Hwsem14_GetCount
};

#endif

/* HWSEM15 Driver Instance */
#if (RTE_HWSEM15)

/* HWSEM15 Resources */
static HWSEM_RESOURCES HWSEM15 = {
    .cb_event = NULL,
    .irq =(IRQn_Type) HWSEM_IRQ15_IRQn,
    .irq_priority =(uint8_t) RTE_HWSEM15_IRQPRIORITY,
    .sem_id = HWSEMID15,
};

/* HWSEM15 Interrupt Handler */
void HWSEM_IRQ15Handler(void)
{
    ARM_HWSEM_IRQHandler(&HWSEM15);
}

static int32_t Hwsem15_Initialize(ARM_HWSEM_SignalEvent_t cb_event)
{
    return Initialize(cb_event, &HWSEM15);
}

static int32_t Hwsem15_Uninitialize(void)
{
    return Uninitialize(&HWSEM15);
}

static int32_t Hwsem15_Lock(void)
{
    return Lock(&HWSEM15);
}

static int32_t Hwsem15_UnLock(void)
{
    return UnLock(&HWSEM15);
}

static int32_t Hwsem15_GetCount(void)
{
    return GetCount(&HWSEM15);
}

/* HWSEM15 Access Struct */
ARM_DRIVER_HWSEM DRIVER_HWSEM15 =
{
    Hwsem_GetVersion,
    Hwsem_GetCapabilities,
    Hwsem15_Initialize ,
    Hwsem15_Uninitialize,
    Hwsem15_Lock,
    Hwsem15_UnLock,
    Hwsem15_GetCount
};

#endif
