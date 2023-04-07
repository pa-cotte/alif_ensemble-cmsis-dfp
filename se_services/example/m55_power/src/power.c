/* Copyright (C) 2022 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */
/**
 * @brief   Power source file
 */

#include <stdio.h>
#include <inttypes.h>
#include <stdbool.h>
#include "vbat_rtc.h"
#include "services_lib_interface.h"
#include "exectb_mcu.h"

#define SET_REGISTER_BITS_U32(Address, Value)  \
                  (*(volatile uint32_t *)(Address) |= (uint32_t)(Value))
#define CLEAR_REGISTER_BITS_U32(Address, Value)  \
                  (*(volatile uint32_t *)(Address) &= ~((uint32_t)(Value)))

 /*
  * 0x1000    delay approximately 112 msec in simulation
  * 0x200     delay approximately  4 msec in simulation
  */
#if PLATFORM_TYPE == FPGA
#define TIMER_LOAD_VALUE    0x50000
#else
#define TIMER_LOAD_VALUE    0x1000
#endif // #if PLATFORM_TYPE == SIMULATION_BOLT

/**
 * Function to initialize stop mode service
 * @return
 */
int POWER_stop_mode_init(void)
{
  SERVICES_print("M55 POWER_stop_mode_init: STARTS\n");

  // Enable M55-HE TCM and 4KB backup RAM retention
  uint32_t mask = 0xF3; //BIT0 | BIT1 | BIT4 | BIT5 | BIT6 | BIT7
  CLEAR_REGISTER_BITS_U32(0x1A609000 + 0xC, mask);

  // Init RTC_A
  //VBAT_rtc_a_initialize();
  //SET_REGISTER_BITS_U32(0x1A609000 + 0x10, 0x1);

  // Start RTC_A
  VBAT_rtc_start(0x42000000ul, TIMER_LOAD_VALUE);

  SERVICES_print("M55 SE  POWER_stop_mode_init: ENDS\n");

  return 0;
}

/**
 * Function to end stop mode service
 */
void POWER_stop_mode_end(void)
{
  VBAT_rtc_stop(0x42000000ul);
}

/**
 * @brief Function to put m55 in sleep mode
 */
void POWER_m55_off(void)
{
  /*
   * CPPWR (Coprocessor Power Control Register)
   *
   * BIT20 = 1
   * This will permit FPU and MVE units to enter a non-retentive power state
   * The default setting does not allow FPU to enter non-retentive power state
   *
   * BIT0 = 1
   * This can be used as a hint to power control logic that the coprocessor
   * might be powered down.
   */
  SCnSCB->CPPWR = (SCnSCB->CPPWR) | (0x1 << 20) | (0x1);

  CPUPWR->DPDLPSTATE =  (0x3 << CPUPWR_DPDLPSTATE_DLPSTATE_Pos);

  CPUPWR->CPDLPSTATE =  (0x3 << CPUPWR_CPDLPSTATE_CLPSTATE_Pos) |
                        (0x3 << CPUPWR_CPDLPSTATE_ELPSTATE_Pos) |
                        (0x3 << CPUPWR_CPDLPSTATE_RLPSTATE_Pos);

  // Clear ICACTIVE and DCACTIVE to prevent cache lookups (This causes P-channel activity, and PDRAMS will become OFF)
  L1->MSCR &= ~(SCB_MSCR_ICACTIVE_Msk | SCB_MSCR_DCACTIVE_Msk);
  __DSB();

  // Set SLEEPDEEP to allow WIC handshakes to occur when executing
  // WFI() or WFE() in systems that include the WIC
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

  // Wait For Interrupt
  // Disable Interrupts to avoid taking an interrupt before executing WFI.
  __disable_irq();
}
