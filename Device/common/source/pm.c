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

/* We only need to preserve APCS callee-preserved registers */
typedef struct fp_state {
    double      d[8];
    uint32_t    fpscr;
    uint32_t    vpr;
} fp_state_t;

__attribute__((naked))
void save_fp_state(fp_state_t *state)
{
  __asm (
    "VSTM    R0, {D8-D15}\n\t"
    "VSTR    FPSCR, [R0, #32]\n\t"
    "VSTR    VPR, [R0, #36]\n\t"
    "BX      LR"
  );
}

__attribute__((naked))
void restore_fp_state(const fp_state_t *state)
{
  __asm (
    "VLDM    R0, {D8-D15}\n\t"
    "VLDR    FPSCR, [R0, #32]\n\t"
    "VLDR    VPR, [R0, #36]\n\t"
    "BX      LR"
  );
}
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
  @fn     void pm_core_set_deep_sleep(void)
  @brief  Power management API which performs deep sleep operation
  On current silicon, this does nothing on its own beyond basic sleep;
  but it activates power saving if a WIC is enabled.
  @return This function return nothing
 */
static void pm_core_enter_wic_sleep(bool iwic)
{
    /* Set up WICCONTROL so that deep sleep is the required WIC sleep type */
    *WICCONTROL = _VAL2FLD(WICCONTROL_WIC, 1) | _VAL2FLD(WICCONTROL_IWIC, iwic);

    /* Setting DEEPSLEEP bit */
	uint32_t scr = SCB->SCR;
    SCB->SCR = scr |= SCB_SCR_SLEEPDEEP_Msk;

    /*Data Synchronization Barrier completes all instructions before this */
    __DSB();

    /* Instruction Synchronization Barrier flushes the pipeline in the
     * processor, so that all instructions following the ISB are fetched from
     * cache or memory */
    __ISB();

    /* Put System into sleep mode */
    pm_core_enter_normal_sleep();

    /* Clearing DEEPSLEEP bit */
    SCB->SCR = scr &=~ SCB_SCR_SLEEPDEEP_Msk;

    /* Clear WICCONTROL to disable WIC sleep */
    *WICCONTROL = _VAL2FLD(WICCONTROL_WIC, 0);

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
  @return This function returns nothing
 */
void pm_core_enter_deep_sleep(void)
{
    /* Entering any WIC sleep could potentially cause state loss,
     * as it enables power saving on PDCORE. Unlike the other domains,
     * there is no separate mechanism to indicate "retention" beyond
     * setting CLPSTATE, so as we want a sleep call we need to
     * ensure CLPSTATE is RET or higher. (See M55 TRM section 7.5)
     *
     * Further, in our design, the IWIC is actually in the same power domain
     * as the CPU, so we need the IWIC+CPU to stay on to be able to wake,
     * which means our minimum is actually ON with clock off.
     *
     * But don't lower the entry value - caller may have a reason to suppress
     * low-power states.
     */
    uint32_t old_cpdlpstate = PWRMODCTL->CPDLPSTATE;
    if (_FLD2VAL(PWRMODCTL_CPDLPSTATE_CLPSTATE, old_cpdlpstate) > LPSTATE_ON_CLK_OFF) {
        PWRMODCTL->CPDLPSTATE = (old_cpdlpstate &~ PWRMODCTL_CPDLPSTATE_CLPSTATE_Msk) |
                                _VAL2FLD(PWRMODCTL_CPDLPSTATE_CLPSTATE, LPSTATE_ON_CLK_OFF);
    }

    /* Trigger the IWIC sleep */
    pm_core_enter_wic_sleep(true);

    /* Restore low power state (probably to all OFF) */
    PWRMODCTL->CPDLPSTATE = old_cpdlpstate;
}

/**
  @fn     void pm_shut_down_dcache(void)
  @brief  Preparation for pm_core_enter_subsys_off

  In preparation for removing power, we need to ensure the data
  cache is clean.
  This is a potentially slow operation, so it may be desirable
  to do so with interrupts enabled before hand. If this call is
  made before pm_core_enter_subsys_off, it significantly reduces
  the work that core does with interrupts disabled.

  @return This function returns a state indicator for pm_restore_dcache_enable
  */
uint32_t pm_shut_down_dcache(void)
{
    /* Stop new data cache allocations  */
    uint32_t orig_ccr = SCB->CCR;
    SCB->CCR = orig_ccr &~ SCB_CCR_DC_Msk;
    __DSB();
    __ISB();

    /* Check cache status */
    uint32_t orig_mscr = MEMSYSCTL->MSCR;

    if (orig_mscr & MEMSYSCTL_MSCR_DCACTIVE_Msk)
    {
        /* Make sure nothing gets dirty any more - this should stabilise DCCLEAN */
        MEMSYSCTL->MSCR = orig_mscr | MEMSYSCTL_MSCR_FORCEWT_Msk;
        __DSB();
        __ISB();

        if (!(MEMSYSCTL->MSCR & MEMSYSCTL_MSCR_DCCLEAN_Msk))
        {
            /* Clean if it is active, and not known to be clean */
            SCB_CleanDCache();
        }
        /* M55 TRM tells us not to modify the DCCLEAN bit; otherwise it seems like
         * we could set it here. Disable the cache and put FORCEWT back how it was.
         */
        MEMSYSCTL->MSCR = (MEMSYSCTL->MSCR &~ (MEMSYSCTL_MSCR_DCACTIVE_Msk | MEMSYSCTL_MSCR_FORCEWT_Msk))
                                            | (orig_mscr & MEMSYSCTL_MSCR_FORCEWT_Msk);
    }

#if (MEMSYSCTL_MSCR_DCACTIVE_Msk & SCB_CCR_DC_Msk) != 0
#error "I'm assuming the bits in these registers don't overlap"
#endif

    return (orig_ccr | orig_mscr) & (MEMSYSCTL_MSCR_DCACTIVE_Msk | SCB_CCR_DC_Msk);
}

/**
  @fn     void pm_restore_dcache_enable(uint32_t old_state)
  @brief  Restore dcache operational state

  If an "off" attempt returns, due to a wake event happening before
  power was shut down, we can undo the effect of
  pm_shut_down_dcache_ready_for_cpu_off and restore cache operation.

  Unlike the disable, this is a fast call, so it can be made before
  enabling interrupts.

  @return This function returns nothing
  */
void pm_restore_dcache_enable(uint32_t old_state)
{
	MEMSYSCTL->MSCR = (MEMSYSCTL->MSCR &~ MEMSYSCTL_MSCR_DCACTIVE_Msk) | (old_state & MEMSYSCTL_MSCR_DCACTIVE_Msk);
	SCB->CCR = (SCB->CCR &~ SCB_CCR_DC_Msk) | (old_state & SCB_CCR_DC_Msk);
}

/**
  @fn     void pm_core_set_subsys_off(void)
  @brief Power management API which performs subsystem off operation
  @return This function return nothing
 */
void pm_core_enter_subsys_off(void)
{
    const uint32_t SU11 = 1 << (11*2);
    const uint32_t SU10 = 1 << (10*2);

    /* We attempt to power off the subsystem by turning off all active
     * indications from the CPU, taking its power domains PDCORE, PDEPU,
     * PDRAMS and PDDEBUG to OFF. See Power chapter of M55 TRM for details.
     *
     * We assume all the LPSTATE indications are OFF as at boot, which will
     * permit everything to go off. We assume that if it's set higher, it's
     * because someone wants to block this. If they have modified it, and
     * don't intend to block this, they should put it back to OFF before
     * calling this.
     */

    fp_state_t fp_state;
    bool fp_saved = false;

    /* PDEPU OFF requires that we set the State Unknown 10 flag indicating it's
     * okay to forget the FP/MVE state (S/D/Q registers, FPSR and VPR)
     */
    uint32_t orig_cppwr = ICB->CPPWR;
    if (!(orig_cppwr & SU10)) {
        /* As we are going to say it's okay to lose EPU state, we should save it;
         * we can't independently turn EPU off on our silicon, but the CPU
         * could choose to reset the registers in response to SU10.
         */

        /* Only need to save if we have our own floating point context active, or lazy stack
         * preservation is active, indicating registers hold another context's state.
         */
        if ((__get_CONTROL() & CONTROL_FPCA_Msk) || (FPU->FPCCR & FPU_FPCCR_LSPACT_Msk)) {
            save_fp_state(&fp_state);
            fp_saved = true;
        }

        /* Indicate we're okay to lose MVE/FP state. Note that MVE/FP instructions will
         * fault after this, so we hope we're not doing anything that prompts the compiler
         * to generate MVE/FP code during this function.
         */
        ICB->CPPWR = orig_cppwr | (SU11 | SU10);
    }

    /* Stop new data cache allocations - lookup continues */
    uint32_t orig_ccr = SCB->CCR;
    SCB->CCR = orig_ccr &~ SCB_CCR_DC_Msk;
    __DSB();
    __ISB();

    /* Check cache status */
    uint32_t orig_mscr = MEMSYSCTL->MSCR;
    if (orig_mscr & MEMSYSCTL_MSCR_DCACTIVE_Msk)
    {
        /* Make sure nothing gets dirty any more - this should stabilise DCCLEAN */
        MEMSYSCTL->MSCR = orig_mscr | MEMSYSCTL_MSCR_FORCEWT_Msk;
        __DSB();
        __ISB();

        if (!(MEMSYSCTL->MSCR & MEMSYSCTL_MSCR_DCCLEAN_Msk))
        {
            /* Clean if data cache is active, and not known to be clean. This
             * could be done earlier by pm_shut_down_dcache, before disabling
             * IRQs. But if we're making this call, we're resigned to bad
             * interrupt latency - we might be needing a full reboot to
             * respond.
             */
            SCB_CleanDCache();
        }
    }

    /* Fully disable the caches, allowing PDRAMS OFF. (On B1 silicon it
     * is important that we don't let the M55 request MEM_RET state by having
     * PDRAMS at RET and PDCORE at OFF - the PPU will grant this,
     * and the M55 will wrongly think its cache has been retained, and skip
     * necessary auto-invalidation on the subsequent reset.) Restore FORCEWT now.
     */
    SCB->CCR = orig_ccr &~ (SCB_CCR_IC_Msk | SCB_CCR_DC_Msk);
    MEMSYSCTL->MSCR = (MEMSYSCTL->MSCR &~ (MEMSYSCTL_MSCR_ICACTIVE_Msk | MEMSYSCTL_MSCR_DCACTIVE_Msk | MEMSYSCTL_MSCR_FORCEWT_Msk))
                                       |  (orig_mscr & MEMSYSCTL_MSCR_FORCEWT_Msk);

    /* Disable PMU/DWT - we know this is enabled at boot by system code using
     * PMU timers, so we could never permit PDDEBUG OFF otherwise.
     * When/if this is resolved, we should consider removing this, so
     * as not to interfere with deliberate debugging.
     */
    uint32_t orig_demcr = DCB->DEMCR;
    DCB->DEMCR = orig_demcr &~ DCB_DEMCR_TRCENA_Msk;

    /* Assume automatic EWIC sequencing - NVIC masks transferred and EWIC
     * enabled by M55.
     */

    /* Trigger the EWIC sleep */
    pm_core_enter_wic_sleep(false);

    /* Restore enables */
    MEMSYSCTL->MSCR |= orig_mscr & (MEMSYSCTL_MSCR_ICACTIVE_Msk | MEMSYSCTL_MSCR_DCACTIVE_Msk);
    SCB->CCR = orig_ccr;
    DCB->DEMCR = orig_demcr;
    ICB->CPPWR = orig_cppwr;

    /* Make sure enables are synchronised */
    __DSB();
    __ISB();

    /* Restore FP/MVE state */
    if (fp_saved) {
        restore_fp_state(&fp_state);
    }
}

/**
  @fn    PM_RESET_REASON pm_core_get_reset_reason(void)
  @brief Get reset reason
  @return reset reason
 */
PM_RESET_REASON pm_core_get_reset_reason(void)
{
    return (PM_RESET_REASON) *RESET_REASON_REG;
}
