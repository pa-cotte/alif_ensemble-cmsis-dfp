/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/*******************************************************************************
 * @file     pm.c
 * @author   Raj Ranjan
 * @email    raj.ranjan@alifsemi.com
 * @version  V1.0.0
 * @date     13-Feb-2023
 * @brief    Power Management Services
 * @bug      None.
 * @Note     None
 ******************************************************************************/

/* Includes ------------------------------------------------------------------*/

#include "pm.h"
#include "pm_internal.h"

#define VERSION(api, driver)        (((api) << 8) | (driver))
#define PM_DRV_VERSION              VERSION(1, 0) /*!< PM Driver Version */

/* WICCONTROL register : volatile static uint32_t *const WICCONTROL*/
#if   defined(M55_HP)
#define WICCONTROL                  (&(AON->RTSS_HP_CTRL ))
#define RESET_REASON_REG            (&(AON->RTSS_HP_RESET))
#elif defined(M55_HE)
#define WICCONTROL                  (&(AON->RTSS_HE_CTRL ))
#define RESET_REASON_REG            (&(AON->RTSS_HE_RESET))
#else
#error "Invalid CPU"
#endif

static void pm_set_cpd_low_power_state(CPDLP_STATE state);


/**
  @fn          uint16_t pm_get_version(void)
  @brief       Get PM driver version.
  @return      uint16_t
*/
uint16_t pm_get_version(void)
{
  return PM_DRV_VERSION;
}

/**
  @fn          void pm_set_cpd_low_power_state(CPDLP_STATE state)
  @brief       Set the Core power domain low power state
  @return      none
*/
static void pm_set_cpd_low_power_state(CPDLP_STATE state)
{
    switch(state)
    {
    case CPDLP_STATE_ON:
        PWRMODCTL->CPDLPSTATE = CPDLP_CLPSTATE_MASK(CPDLP_STATE_ON);
        PWRMODCTL->DPDLPSTATE = CPDLP_DLPSTATE_MASK(CPDLP_STATE_ON);
        break;
    case CPDLP_STATE_CLK_OFF:
        PWRMODCTL->CPDLPSTATE = CPDLP_CLPSTATE_MASK(CPDLP_STATE_CLK_OFF);
        PWRMODCTL->DPDLPSTATE = CPDLP_DLPSTATE_MASK(CPDLP_STATE_CLK_OFF);
        break;
    case CPDLP_STATE_OFF:
        PWRMODCTL->CPDLPSTATE = CPDLP_CLPSTATE_MASK(CPDLP_STATE_OFF);
        PWRMODCTL->DPDLPSTATE = CPDLP_DLPSTATE_MASK(CPDLP_STATE_OFF);
        break;
    case CPDLP_STATE_RET:  /* Fall through */
    default:
        break;
    }
}

/**
  @fn     void pm_core_set_deep_sleep(void)
  @brief  Power management API which performs deep sleep operation
  @return This function return nothing
 */
void pm_core_set_deep_sleep(void)
{
    /* Setting DEEPSLEEP bit */
    SCB->SCR       |=  SCB_SCR_SLEEPDEEP_Msk;
    *WICCONTROL    &= ~BIT(WICCONTROL_WIC_Pos);

    /*Data Synchronization Barrier completes all instructions before this */
    __DSB();

    /* Instruction Synchronization Barrier flushes the pipeline in the
     * processor, so that all instructions following the ISB are fetched from
     * cache or memory */
    __ISB();

    /* Put System into sleep mode */
    __WFI();

    /* Clearing DEEPSLEEP bit */
    SCB->SCR       &=  ~SCB_SCR_SLEEPDEEP_Msk;

    /* Data Synchronization Barrier completes all instructions before this */
    __DSB();

    /* Instruction Synchronization Barrier flushes the pipeline in the
     *  processor, so that all instructions following the ISB are fetched
     *  from cache or memory */
    __ISB();
}

/**
  @fn     void pm_core_set_iwic_sleep(void)
  @brief  Power management API which performs iwic sleep operation
  @return This function return nothing
 */
void pm_core_set_iwic_sleep(void)
{
    /* Setting DEEPSLEEP bit */
    SCB->SCR       |=  SCB_SCR_SLEEPDEEP_Msk;

    /* Set WICCONTROL register */
    *WICCONTROL    |= BIT(WICCONTROL_WIC_Pos);
    *WICCONTROL    |= BIT(WICCONTROL_IWIC_Pos);

    /* Low Power State */
    pm_set_cpd_low_power_state(CPDLP_STATE_CLK_OFF);

    /* Data Synchronization Barrier completes all instructions before this */
    __DSB();

    /* Instruction Synchronization Barrier flushes the pipeline in the
     * processor, so that all instructions following the ISB are fetched from
     * cache or memory */
    __ISB();

    /* Put System into sleep mode */
    __WFI();

    /* Low Power State */
    pm_set_cpd_low_power_state(CPDLP_STATE_ON);

    /* Clearing DEEPSLEEP bit */
    SCB->SCR       &=  ~SCB_SCR_SLEEPDEEP_Msk;

    /* Data Synchronization Barrier completes all instructions before this */
    __DSB();

    /* Instruction Synchronization Barrier flushes the pipeline in the
     *  processor, so that all instructions following the ISB are fetched
     *  from cache or memory */
    __ISB();

    /* Clearing WICCONTROL register */
    *WICCONTROL    &= ~BIT(WICCONTROL_WIC_Pos);
    *WICCONTROL    &= ~BIT(WICCONTROL_IWIC_Pos);
}

/**
  @fn     void pm_core_set_subsys_off(void)
  @brief Power management API which performs subs off operation
  @return This function return nothing
 */
void pm_core_set_subsys_off(void)
{
    /* Setting DEEPSLEEP bit  */
    SCB->SCR       |=  SCB_SCR_SLEEPDEEP_Msk;
    _EWIC->EWIC_CR |=  EWIC_EWIC_CR_EN_Msk;

    /* Clear WICCONTROL register */
    *WICCONTROL    |= BIT(WICCONTROL_WIC_Pos);
    *WICCONTROL    &= ~BIT(WICCONTROL_IWIC_Pos);

    /* Low Power State */
    pm_set_cpd_low_power_state(CPDLP_STATE_OFF);

    /* Flush the Instruction and Data Cache  */
    SCB_CleanDCache();

    /* Data Synchronization Barrier completes all instructions before this */
    __DSB();

    /* Instruction Synchronization Barrier flushes the pipeline in the
     * processor, so so that all instructions following the ISB are fetched
     * from cache or memory */
    __ISB();

    /* Put System into sleep mode */
    __WFI();

    /* Low Power State */
    pm_set_cpd_low_power_state(CPDLP_STATE_ON);

    /* Clearing DEEPSLEEP bit */
    SCB->SCR       &=  ~SCB_SCR_SLEEPDEEP_Msk;

    /* wiccontrol WIC bit clear, IWIC is intact */
    *WICCONTROL    &= ~BIT(WICCONTROL_WIC_Pos);

    /* Data Synchronization Barrier completes all instructions before this */
    __DSB();

    /* Instruction Synchronization Barrier flushes the pipeline in the
     * processor, so that all instructions following the ISB are fetched
     * from cache or memory */
    __ISB();
}

/**
  @fn    PM_RESET_REASON pm_core_get_reset_reason(void)
  @brief Get reset reason
  @return reset reason
 */
PM_RESET_REASON pm_core_get_reset_reason()
{
    return (PM_RESET_REASON) *RESET_REASON_REG;
}
