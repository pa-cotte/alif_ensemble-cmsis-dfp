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
 * @file     pm.h
 * @author   Raj Ranjan
 * @email    raj.ranjan@alifsemi.com
 * @version  V1.0.0
 * @date     20-Feb-2023
 * @brief    Power Management Services API
 * @bug      None.
 * @Note     None
 ******************************************************************************/
#ifndef PM_H_
#define PM_H_

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "system_utils.h"


#ifdef  __cplusplus
extern "C"
{
#endif


/**
  @brief PM Return Status
 */
typedef enum _PM_STATUS
{
    PM_STATUS_OK,              /*!<  SUCCESS                 */
    PM_STATUS_UNSUPPORTED,     /*!<  Unsupported Sleep state */
    PM_STATUS_ERROR,           /*!<  ERROR                   */

    PM_STATUS_MAX = 0x7FFFFFFFUL
}PM_STATUS;

/**
  @brief enum of wake reasons-
 */
typedef enum _PM_RESET_REASON
{
    PM_RESET_REASON_POR_OR_SOC_OR_HOST_RESET =  0,   /*!< Indicates the last
                                                         reset of the External
                                                         System was caused by
                                                         the POR (Reset because
                                                         of power on/off or SOC)
                                                         or Host reset i.e.
                                                         Secure Enclave       */
    PM_RESET_REASON_NRST_RESET               =  1,   /*!< Indicates that the
                                                         last reset of the
                                                         External System was
                                                         caused by nSRST      */
    PM_RESET_REASON_EXTERNAL_SYS_RESET       =  4,   /*!< Indicates the last
                                                         reset of the External
                                                         System was caused by a
                                                         request to reset this
                                                         External System      */

    PM_RESET_REASON_MAX = 0x7FFFFFFFUL
} PM_RESET_REASON;

/**
  @brief Wake sources
 */
typedef enum _PM_EWIC_WAKEUP_SRC
{
#if (defined(M55_HE))
    /* CM55_HE Wake-up IRQs (EXTSYS1 )        */
    PM_EWIC_WAKEUP_SRC_DMA2_ABORT       = (32UL),

    PM_EWIC_WAKEUP_SRC_HES10_RX         = (33UL),
    PM_EWIC_WAKEUP_SRC_HES11_RX         = (35UL),
    PM_EWIC_WAKEUP_SRC_SEES10_RX        = (37UL),
    PM_EWIC_WAKEUP_SRC_SEES11_RX        = (39UL),
    PM_EWIC_WAKEUP_SRC_ES1ES10_RX       = (41UL),
    PM_EWIC_WAKEUP_SRC_ES1ES11_RX       = (43UL),
    PM_EWIC_WAKEUP_SRC_LPUART_RX        = (45UL),
    PM_EWIC_WAKEUP_SRC_LPI2C_RX         = (47UL),
    PM_EWIC_WAKEUP_SRC_LPCMP            = (56UL),
    PM_EWIC_WAKEUP_SRC_LPGPIO           = (57UL),
    PM_EWIC_WAKEUP_SRC_LPRTC            = (58UL),
    PM_EWIC_WAKEUP_SRC_AON_LP_TIMER0    = (60UL),
    PM_EWIC_WAKEUP_SRC_AON_LP_TIMER1    = (61UL),
    PM_EWIC_WAKEUP_SRC_AON_LP_TIMER2    = (62UL),
    PM_EWIC_WAKEUP_SRC_AON_LP_TIMER3    = (63UL),
#elif (defined(M55_HP))
    /* CM55_HP Wake-up IRQs (EXTSYS0 )         */
    PM_EWIC_WAKEUP_SRC_DMA1_ABORT       = (32UL),

    PM_EWIC_WAKEUP_SRC_HES00_RX         = (33UL),
    PM_EWIC_WAKEUP_SRC_HES01_RX         = (35UL),
    PM_EWIC_WAKEUP_SRC_SEES00_RX        = (37UL),
    PM_EWIC_WAKEUP_SRC_SEES01_RX        = (39UL),
    PM_EWIC_WAKEUP_SRC_ES1ES00_RX       = (41UL),
    PM_EWIC_WAKEUP_SRC_ES1ES01_RX       = (43UL),
    PM_EWIC_WAKEUP_SRC_LPCMP            = (56UL),
    PM_EWIC_WAKEUP_SRC_LPGPIO           = (57UL),
    PM_EWIC_WAKEUP_SRC_LPRTC            = (58UL),
    PM_EWIC_WAKEUP_SRC_AON_LP_TIMER0    = (60UL),
    PM_EWIC_WAKEUP_SRC_AON_LP_TIMER1    = (61UL),
    PM_EWIC_WAKEUP_SRC_AON_LP_TIMER2    = (62UL),
    PM_EWIC_WAKEUP_SRC_AON_LP_TIMER3    = (63UL),
#endif
    /* Max */
    PM_EWIC_WAKEUP_SRC_MAX              = 0x7FFFFFFF,
} PM_EWIC_WAKEUP_SRC;

/**
  @brief Power Management wakeup reason
*/
typedef struct _pm_wakeup_reason_t {
    PM_EWIC_WAKEUP_SRC    ewic_wakeup_src;
    PM_RESET_REASON       reset_reason   ;
} pm_wakeup_reason_t;


/**
 *******************************************************************************
 *                        Function documentation
 ******************************************************************************/

/**
  @fn          uint16_t pm_get_version(void)
  @brief       Get PM driver version.
  @return      uint16_t
*/
uint16_t pm_get_version(void);

/**
  @fn     uint16_t pm_core_enter_normal_sleep(void)
  @brief  Power management API which performs normal sleep operation
  @note   This function should be called with interrupts disabled.
  @note   This function is provided for consistency with the
          deeper sleeps, which require interrupts disabled.
          In interrupt-enabled context, in bare-metal use, direct
          use of __WFE() may be more appropriate.
  @return This function returns nothing
 */
static inline void pm_core_enter_normal_sleep(void)
{
    __WFI();
}

/**
  @fn     void pm_core_enter_deep_sleep(void)
  @brief  Power management API which performs deep sleep operation
  @note   On current silicon, this does nothing on its own beyond basic sleep;
          but it activates power saving if a WIC is enabled. So it is
          called by the calls below.
  @note   This function should be called with interrupts disabled
  @return This function returns nothing
 */
void pm_core_enter_deep_sleep(void);

/**
  @fn     void pm_core_enter_iwic_sleep(void)
  @brief  Power management API which performs IWIC sleep operation
          This enters the deepest possible CPU sleep state, without
          losing CPU state. All CPU clocks can be stopped, including
          SysTick. CPU and subsystem power will remain on.
  @note   Possible IWIC wake sources are events, NMI, debug events
          and interrupts 0-63 only, subject to standard interrupt
          enable controls.
  @note   This function should be called with interrupts disabled
  @return This function return nothing
 */
void pm_core_enter_iwic_sleep(void);

/**
  @fn     void pm_core_enter_subsys_off(void)
  @brief Power management API which performs subsystem off operation
  @note   This function should be called with interrupts disabled
          A cache clean operation is performed if necessary
  @note   This function will not return if the system goes off
          before a wake event. It will return if a wake event
          occurs before power off is possible.
  @note   Possible EWIC wake sources are a limited selection
          of interrupts 0-63 - see the HWRM for details.
          The CPU may also reboot if power is automatically
          applied to the subsystem for other reasons aside from
          EWIC wakeup.
          The pending information from EWIC is transferred
          into the NVIC on startup, so interrupt handlers
          can respond to the cause as soon as they're
          unmasked by drivers.
  @return This function returns nothing, or causes reboot.
 */
void pm_core_enter_subsys_off(void);

/**
  @fn     void pm_shut_down_dcache(void)
  @brief  Preparation for pm_core_enter_subsys_off

  In preparation for removing power, we need to ensure the data
  cache is clean.
  This is a potentially slow operation, so it may be desirable
  to do so with interrupts enabled before hand. If this call is
  made before pm_core_enter_subsys_off, it significantly reduces
  the work that core does with interrupts disabled.

  Example usage:

  ```
      atomic_bool service_requested; // set by an EWIC-capable IRQ handler

      while (!service_requested) {
          uint32_t cache_state = pm_shut_down_cache(void); // keep IRQs enabled while cleaning
          __disable_irq();
          if (!service_requested) { // re-check as IRQ could have occurred during clean
              pm_core_enter_subsys_off(); // if IRQ pends during shutdown, this will return
          }
          pm_restore_cache_enable(cache_state);
          __enable_irq(); // Any pending IRQ will be triggered after this
      }
      // Do service work
   ```

  @return This function returns a state indicator for pm_restore_dcache_enable
  */
uint32_t pm_shut_down_dcache(void);

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
void pm_restore_dcache_enable(uint32_t old_state);

/**
  @fn    PM_WAKEUP_REASON pm_core_get_reset_reason(void)
  @brief Get reset reason
  @return Reset reason return
 */
PM_RESET_REASON pm_core_get_reset_reason();

#ifdef  __cplusplus
}
#endif

#endif /* POWER_MANAGEMENT_H_ */
