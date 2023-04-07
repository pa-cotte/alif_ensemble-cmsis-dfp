/**
 * @file  services_test.c
 *
 * @brief Services library test harness
 * @ingroup services
 * @par
 * Copyright (C) 2022 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/******************************************************************************
 *  I N C L U D E   F I L E S
 *****************************************************************************/
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "services_lib_interface.h"
#include "cmsis_compiler.h"
#include "services_lib_protocol.h"
#include "services_lib_api.h"

extern int MHU_send_msg(uint32_t message);
extern int POWER_stop_mode_init(void);
extern void POWER_stop_mode_end(void);
extern void POWER_m55_off(void);

/* forward tests */
static uint32_t test_services_stop_mode(char *p_test_name,
                                        uint32_t services_handle);
static uint32_t test_services_corstone_standby_mode(char *p_test_name,
                                                    uint32_t services_handle);

/*******************************************************************************
 *  M A C R O   D E F I N E S
 ******************************************************************************/

// Enabling this will cause the chip to go into stop mode and wake up repeatedly
#if ((DEVICE_TYPE == FUSION && DEVICE_REVISION == REV_B0) \
    || (DEVICE_TYPE == SPARK))
#define STOP_MODE_ENABLE            1
#else
#define STOP_MODE_ENABLE            0
#endif

#define RANDOMIZER_FEATURE          0
#define TEST_PRINT_ENABLE           1   /* Enable printing from Test harness */
#define PRINT_VIA_CONSOLE           0   /* Print via Debugger console        */
#define PRINT_VIA_SE_UART           1   /* Print via SE UART terminal        */
#define NUMBER_OF_TESTS             2

#define PRINT_BUFFER_SIZE           256 /* Maximum print buffer               */

/*******************************************************************************
 *  T Y P E D E F S
 ******************************************************************************/
typedef struct
{
  uint32_t (*test_fn)(char *p_test_name, uint32_t services_handle);
  char *test_name;
} services_test_t;

/*******************************************************************************
 *  G L O B A L   V A R I A B L E S
 ******************************************************************************/

static services_test_t tests[] =
{
    { test_services_corstone_standby_mode,   "Corestone standby   " },
    { test_services_stop_mode,               "Stop mode           " }
};

/*******************************************************************************
 *  C O D E
 ******************************************************************************/

/**
 * @fn    void TEST_print(uint32_t services_handle, char * fmt, ...)
 * @param services_handle
 * @param buffer_size
 * @param fmt
 */
void TEST_print(uint32_t services_handle, char *fmt, ...)
{
#if TEST_PRINT_ENABLE != 0
  va_list args;
  static char buffer[PRINT_BUFFER_SIZE] = { 0 };
  size_t buffer_size;

  /*
   * @todo Handle long strings bigger than buffer size
   */
  va_start(args,fmt);
  buffer_size = vsnprintf(buffer, PRINT_BUFFER_SIZE, fmt, args);
  va_end(args);

  /**
   * Choice of Console printing or via the SE-UART
   */
#if PRINT_VIA_CONSOLE != 0
  if (buffer_size >= 0)
  {
    printf("%s", buffer);
  }
#endif
#if PRINT_VIA_SE_UART != 0
  SERVICES_uart_write(services_handle, strlen(buffer)+1, (uint8_t *)buffer);
#endif
  (void)buffer_size;
#endif // #if SERVICES_PRINT_ENABLE != 0
}

/**
 * SRAM 0/1 Retention services API test
 */
static uint32_t test_services_corstone_standby_mode(char *p_test_name,
                                                    uint32_t services_handle)
{
  uint32_t error_code = SERVICES_REQ_SUCCESS;
  uint32_t service_error_code;

  host_cpu_clus_pwr_req_t host_cpu_clus_pwr_req;
  bsys_pwr_req_t bsys_pwr_req;

  host_cpu_clus_pwr_req.word = 0;
  host_cpu_clus_pwr_req.bits.mem_ret_req = 1;
  host_cpu_clus_pwr_req.bits.pwr_req = 1;

  bsys_pwr_req.word = 0;
  bsys_pwr_req.bits.systop_pwr_req = SYSTOP_PWR_REQ_LOGIC_ON_MEM_ON;
  bsys_pwr_req.bits.dbgtop_pwr_req = DBGTOP_PWR_REQ_ON;
  bsys_pwr_req.bits.refclk_req = REFCLK_REQ_ON;
  bsys_pwr_req.bits.wakeup_en = WAKEUP_EN_SE_ON;

  error_code = SERVICES_corstone_standby_mode(services_handle,
                                              host_cpu_clus_pwr_req,
                                              bsys_pwr_req,
                                              &service_error_code);
  TEST_print(services_handle,
             "** TEST %s error_code=%s service_resp=0x%08X\n",
             p_test_name,
             SERVICES_error_to_string(error_code),
             service_error_code);

  return error_code;
}

/**
 * Power services
 */
static uint32_t test_services_stop_mode(char *p_test_name,
                                        uint32_t services_handle)
{
  volatile unsigned int * wic_ctrl = (unsigned int*)0x1A604010u; // EXTSYS1_CTRL_REG
  *wic_ctrl = 0x00000100; //set deep is wic and use EWIC + clearing cold_wakeup register bit

  uint32_t error_code = SERVICES_REQ_SUCCESS;

  // Configure EWIC - RTC_A, RTC_SE
  error_code = SERVICES_power_ewic_config(services_handle,
                                          EWIC_RTC_SE | EWIC_RTC_A,
                                          LOWEST_POWER_PROFILE);
  if (error_code != SERVICES_REQ_SUCCESS)
  {
    TEST_print(services_handle,
               "** TEST %s Service error_code=%s\n",
               p_test_name,
               SERVICES_error_to_string(error_code));
    TEST_print(services_handle, "EWIC config req failed\n");
    return error_code;
  }

  // Configure VBAT wake up source
  error_code = SERVICES_power_wakeup_config(services_handle,
                                            VBAT_WAKEUP_RTC_SE
                                            | VBAT_WAKEUP_RTC_A,
                                            LOWEST_POWER_PROFILE);
  if (error_code != SERVICES_REQ_SUCCESS)
  {
    TEST_print(services_handle,
               "** TEST %s Service error_code=%s\n",
               p_test_name,
               SERVICES_error_to_string(error_code));
    TEST_print(services_handle, "VBAT wake up config req failed\n");
    return error_code;
  }

  // Configure memory retention
  error_code = SERVICES_power_mem_retention_config(services_handle,
                                                   0x3FF, // bit9:0
                                                   LOWEST_POWER_PROFILE);

  if (error_code != SERVICES_REQ_SUCCESS)
  {
    TEST_print(services_handle,
               "** TEST %s Service error_code=%s\n",
               p_test_name,
               SERVICES_error_to_string(error_code));
    TEST_print(services_handle, "Memory retention config req failed\n");
    return error_code;
  }

  // Save m55-he vtor for wake up
  error_code = SERVICES_power_m55_he_vtor_save(services_handle,
                                               0x0,
                                               0x0,
                                               LOWEST_POWER_PROFILE);

  if (error_code != SERVICES_REQ_SUCCESS)
  {
    TEST_print(services_handle,
               "** TEST %s Service error_code=%s\n",
               p_test_name,
               SERVICES_error_to_string(error_code));
    TEST_print(services_handle, "m55-he vtor save req failed\n");
    return error_code;
  }

  // Save m55-hp vtor for wake up
  error_code = SERVICES_power_m55_hp_vtor_save(services_handle,
                                               0x0,
                                               0x0,
                                               LOWEST_POWER_PROFILE);

  if (error_code != SERVICES_REQ_SUCCESS)
  {
    TEST_print(services_handle,
               "** TEST %s Service error_code=%s\n",
               p_test_name,
               SERVICES_error_to_string(error_code));
    TEST_print(services_handle, "m55-hp vtor save req failed\n");
    return error_code;
  }

#if STOP_MODE_ENABLE == 1
  // Stop mode request
  POWER_stop_mode_init();
  error_code = SERVICES_power_stop_mode_req(services_handle,
                                            LOWEST_POWER_PROFILE,
                                            0);
  if (error_code == SERVICES_REQ_SUCCESS)
  {
    POWER_m55_off();

    // Wait for interrupt
    __DSB();
    __WFI();
    __ISB();
  }
  else
  {
    TEST_print(services_handle,
               "** TEST %s Service error_code=%s\n",
               p_test_name,
               SERVICES_error_to_string(error_code));
    TEST_print(services_handle, "Stop mode req failed\n");

    POWER_stop_mode_end();
  }
#endif // #if STOP_MODE_ENABLE == 1
  return error_code;
}

/**
 * @fn    void SERVICES_test(uint32_t services_handle)
 * @brief test harness for supported SERVICE APIS
 */
void SERVICES_test(uint32_t services_handle)
{
  uint32_t service_error_code;
  int retry_count;

  /* keep sending heartbeat services requests until one succeeds */
  retry_count = SERVICES_synchronize_with_se(services_handle);

  /* Disable tracing output for services */
  SERVICES_system_set_services_debug(services_handle, false,
                                     &service_error_code);
  /* show services version */
  TEST_print(services_handle, "SERVICES_synchronize_with_se() returned %d\n", retry_count);
  TEST_print(services_handle, "m55_power: SERVICES version %s %s %s\n",
             SERVICES_version(), __DATE__, __TIME__);

  for (int number_of_tests = 0; number_of_tests < NUMBER_OF_TESTS;
       number_of_tests++)
  {
    services_test_t *p_test;
    uint32_t error_code;
#if RANDOMIZER_FEATURE == 1
    uint32_t random_test;
    srand(time());
    random_test = rand()%(NUMBER_OF_TESTS);
    p_test = (services_test_t *)&tests[random_test];
#else
    p_test = (services_test_t *)&tests[number_of_tests];
#endif
    error_code = p_test->test_fn(p_test->test_name,services_handle);
    (void)(error_code);
  }
}
