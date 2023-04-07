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
 * @file  services_test.c
 * @brief Services library test harness
 * @ingroup services
 * @par
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
#include "services_lib_protocol.h"
#include "services_lib_api.h"

/* forward tests */
static uint32_t test_services_heartbeat(char *p_test_name, uint32_t services_handle);
static uint32_t test_services_pinmux(char *p_test_name, uint32_t services_handle);
static uint32_t test_services_padcontrol(char *p_test_name, uint32_t services_handle);
static uint32_t test_services_crypto_trng64(char *p_test_name, uint32_t services_handle);
static uint32_t test_services_crypto_trng32(char *p_test_name, uint32_t services_handle);
static uint32_t test_services_crypto_lcs(char *p_test_name, uint32_t services_handle);
static uint32_t test_services_getotp(char *p_test_name, uint32_t services_handle);
static uint32_t test_services_gettoc(char *p_test_name, uint32_t services_handle);
static uint32_t test_services_gettoc_via_name_m55_he(char *p_test_name, uint32_t services_handle);
static uint32_t test_services_gettoc_via_name_m55_hp(char *p_test_name, uint32_t services_handle);
static uint32_t test_services_gettoc_via_cpuid_he(char *p_test_name, uint32_t services_handle);
static uint32_t test_services_gettoc_via_cpuid_hp(char *p_test_name, uint32_t services_handle);
static uint32_t test_services_gettoc_via_cpuid_a32(char *p_test_name, uint32_t services_handle);
static uint32_t test_services_gettoc_version(char *p_test_name, uint32_t services_handle);
static uint32_t test_services_gettoc_data(char *p_test_name, uint32_t services_handle);
static uint32_t test_services_get_se_revision(char *p_test_name, uint32_t services_handle);
static uint32_t test_services_get_socid(char *p_test_name, uint32_t services_handle);
static uint32_t test_services_read_otp (char *p_test_name, uint32_t services_handle);
static uint32_t test_services_boot_toc_a32(char *p_test_name, uint32_t services_handle);
static uint32_t test_services_boot_release_extsys0(char *p_test_name, uint32_t services_handle);
static uint32_t test_services_mbedtls_aes(char *p_test_name, uint32_t services_handle);
static uint32_t test_services_bounds(char *p_test_name, uint32_t services_handle);
static uint32_t test_services_ewic_config(char *p_test_name, uint32_t services_handle);
static uint32_t test_services_vbat_wakeup_config(char *p_test_name, uint32_t services_handle);
static uint32_t test_services_mem_retention_config(char *p_test_name, uint32_t services_handle);
static uint32_t test_services_ospi_write_key(char *p_test_name, uint32_t services_handle);
static uint32_t test_services_set_es0_frequency(char *p_test_name, uint32_t services_handle);
static uint32_t test_services_set_divider(char *p_test_name, uint32_t services_handle);

/*******************************************************************************
 *  M A C R O   D E F I N E S
 ******************************************************************************/
#define RANDOMIZER_FEATURE          0
#define TEST_PRINT_ENABLE           1   /* Enable printing from Test harness  */
#define PRINT_VIA_CONSOLE           0   /* Print via Debugger console         */
#define PRINT_VIA_SE_UART           1   /* Print via SE UART terminal         */
#define NUMBER_OF_TEST_RUNS         1   /* Number of times to test            */
#define NUMBER_OF_TESTS             30  /* Number of tests to test            */

#define PRINT_BUFFER_SIZE           256 /* Maximum print buffer               */

#if CPU == CPU_M55_HE
#define CPU_STRING "M55_HE"
#elif CPU == CPU_M55_HP
#define CPU_STRING "M55_HP"
#elif CPU == A32
#define CPU_STRING "A32"
#else
#define CPU_STRING "<unknown>"
#endif

/**
 * @brief TOC flags bit values
 */
#define TOC_IMAGE_COMPRESSED          0x10u
#define TOC_IMAGE_LOAD                0x20u
#define TOC_IMAGE_BOOT                0x40u
#define TOC_IMAGE_ENCRYPT             0x80u
#define TOC_IMAGE_DEFERRED            0x100u

/**
 * @brief Flag positions with the flag string
 */
#define FLAG_STRING_COMPRESSED 0
#define FLAG_STRING_LOAD_IMAGE 1
#define FLAG_STRING_VERIFY     2
#define FLAG_STRING_CPU_BOOTED 3
#define FLAG_STRING_ENCRYPTED  4
#define FLAG_STRING_DEFERRED   5
#define FLAG_STRING_END        6
#define FLAG_STRING_SIZE       10

/*******************************************************************************
 *  T Y P E D E F S
 ******************************************************************************/

/**
 * @struct services_test_t
 * @brief  test entry description
 */
typedef struct
{
  uint32_t (*test_fn)(char *p_test_name, uint32_t services_handle);
  char *test_name;
} services_test_t;

/*******************************************************************************
 *  G L O B A L   V A R I A B L E S
 ******************************************************************************/

static services_test_t tests[NUMBER_OF_TESTS] =
{
    { test_services_heartbeat,               "heartbeat       "            },
    { test_services_pinmux,                  "pinmux          "            },
    { test_services_padcontrol,              "padcontrol      "            },
    { test_services_read_otp,                "read otp        "            },
    { test_services_crypto_trng64,           "crypto TRNG 64  "            },
    { test_services_crypto_trng32,           "crypto TRNG 32  "            },
    { test_services_crypto_trng64,           "crypto TRNG 64  "            },
    { test_services_crypto_trng32,           "crypto TRNG 32  "            },
    { test_services_crypto_lcs,              "crypto get LCS  "            },
    { test_services_get_se_revision,         "get SE revision "            },
    { test_services_get_socid,               "get soc id      "            },
    { test_services_getotp,                  "OTP get data    "            },
    { test_services_gettoc,                  "TOC get data    "            },
    { test_services_gettoc_via_name_m55_he,  "TOC via name  HE"            },
    { test_services_gettoc_via_name_m55_hp,  "TOC via name  HP"            },
    { test_services_gettoc_via_cpuid_he,     "TOC via cpuid HE"            },
    { test_services_gettoc_via_cpuid_hp,     "TOC via cpuid HP"            },
    { test_services_gettoc_via_cpuid_a32,    "TOC via cpuidA32"            },
    { test_services_gettoc_version,          "TOC version     "            },
    { test_services_gettoc_data,             "TOC get data    "            },
    { test_services_boot_toc_a32,            "Boot TOC A32    "            },
    { test_services_boot_release_extsys0,    "Release EXTSYS0 "            },
    { test_services_mbedtls_aes,             "MbedTLS AES     "            },
    { test_services_bounds,                  "Bounds Tests    "            },
    { test_services_mem_retention_config,    "SE    Memory Retention config"},
    { test_services_ewic_config,             "EWIC config                 "},
    { test_services_vbat_wakeup_config,      "VBAT Wake Up config         "},
    { test_services_ospi_write_key,          "OSPI write key  "            },
    { test_services_set_es0_frequency,       "Set M55-HP frequency"        },
    { test_services_set_divider,             "Set a clock divider"         }
};

static SERVICES_toc_data_t toc_info; /*!< Global to test harness */
static SERVICES_otp_data_t otp_info  /*!< Global to test harness */;

/*******************************************************************************
 *  C O D E
 ******************************************************************************/

/**
 * @brief CPU ID to return string
 * @param cpu_id
 * @return
 */
static char *CPUID_to_string(uint32_t cpu_id)
{
  static char err_string[10] = { 0 };
  char *p_str = NULL;

  switch (cpu_id)
   {
       case FUSION_A32_0:
         p_str = "A32_0";
         break;
       case FUSION_A32_1:
         p_str = "A32_1";
         break;
       case FUSION_M55_HP:
         p_str = "M55_HP";
         break;
       case FUSION_M55_HE:
         p_str = "M55_HE";
         break;
       case 15:
         p_str = "CM0+  ";
         break;
       default:
         p_str = "??????";
         break;
  }
  strncpy(err_string, p_str, sizeof(err_string));

  return (char *)&err_string[0];
}

/**
 * @brief flags_to_string - convert TOC 'Flags' to a string
 * @param p_status
 * @param flag_string
 * @return
 */
char *flags_to_string(uint32_t flags, char flag_string[])
{
  bool compressed = (flags & TOC_IMAGE_COMPRESSED) != 0;
  flag_string[FLAG_STRING_COMPRESSED] = compressed == true ? 'C' : 'u';

  bool load = (flags & TOC_IMAGE_LOAD) != 0;
  flag_string[FLAG_STRING_LOAD_IMAGE] = load ? 'L' : ' ';

  bool booted = (flags & TOC_IMAGE_BOOT) != 0;
  flag_string[FLAG_STRING_CPU_BOOTED] = booted ? 'B' : ' ';

  bool encrypted = (flags & TOC_IMAGE_ENCRYPT) != 0;
  flag_string[FLAG_STRING_ENCRYPTED] = encrypted ? 'E' : ' ';

  bool deferred = (flags & TOC_IMAGE_DEFERRED) != 0x0;
  flag_string[FLAG_STRING_DEFERRED] = deferred ? 'D' : ' ';

#if 0
  bool verify = (flags & TOC_IMAGE_DEFERRED) != 0x0;
  flag_string[FLAG_STRING_VERIFY] = p_status->verified == true ? 'V' : 's';
#endif

  flag_string[FLAG_STRING_END] = '\0';
  return (char *)&flag_string[0]; /*!< return string back to printing */
}

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
  * APPLICATION Services
  */

/**
 * @fn    static uint32_t test_services_pinmux(uint32_t services_handle)
 * @param p_test_name
 * @param services_handle
 * @return
 */
static uint32_t test_services_pinmux(char*p_test_name,
                                     uint32_t services_handle)
{
  uint32_t error_code = SERVICES_REQ_SUCCESS;
  uint32_t service_error_code;

#if ((DEVICE_TYPE == FUSION && DEVICE_REVISION == REV_B0) \
    || (DEVICE_TYPE == SPARK))
  error_code = SERVICES_pinmux(services_handle,
                               1, 44, 0,
                               &service_error_code);
#else
   /* configures blinky */
  error_code = SERVICES_pinmux(services_handle,
                               1, 14, 0,
                               &service_error_code);
#endif

  TEST_print(services_handle,
             "** TEST %s error_code=%s service_resp=0x%08X %s\n",
             p_test_name,
             SERVICES_error_to_string(error_code), 
             service_error_code, 
             service_error_code == PINMUX_SUCCESS ? "" : "\tINVALID PIN");

  return error_code;
}

/**
 * @fn    static uint32_t test_services_padcontrol(uint32_t services_handle)
 * @param p_test_name
 * @param services_handle
 * @return
 */
static uint32_t test_services_padcontrol(char*p_test_name,
                                         uint32_t services_handle)
{
  uint32_t error_code = SERVICES_REQ_SUCCESS;
  uint32_t service_error_code;

#if DEVICE_REVISION == REV_B0
  error_code = SERVICES_padcontrol(services_handle,
                                   1, 44, 0x0,
                                   &service_error_code);
#else
   /* configures blinky */
  error_code = SERVICES_padcontrol(services_handle,
                                   1, 14, 0x0,
                                   &service_error_code);
#endif

  TEST_print(services_handle,
             "** TEST %s error_code=%s service_resp=0x%08X %s\n",
             p_test_name,
             SERVICES_error_to_string(error_code),
             service_error_code, 
             service_error_code == PINMUX_SUCCESS ? "" : "\tINVALID PIN");

  return error_code;
}

/**
 * @fn    static uint32_t test_services_crypto_trng64(uint32_t services_handle)
 * @param services_handle
 * @return
 */
static uint32_t test_services_crypto_trng64(char *p_test_name,
                                            uint32_t services_handle)
{
  uint32_t error_code = SERVICES_REQ_SUCCESS;
  int32_t service_error_code;
  uint64_t rnd_value;

  error_code = SERVICES_cryptocell_get_rnd(services_handle,
                                           sizeof(uint64_t),/* random number/vector length in bytes*/
                                           &rnd_value,
                                           &service_error_code);
  TEST_print(services_handle,
              "** TEST %s error_code=%s 64-bit Random value = 0x%jx service_resp=%d\n",
              p_test_name,
              SERVICES_error_to_string(error_code),
              rnd_value,
              service_error_code);

  return (error_code);
}

/**
 * @fn      static uint32_t test_services_crypto_trng64(uint32_t services_handle)
 * @param services_handle
 * @return
 */
static uint32_t test_services_crypto_trng32(char *p_test_name,
                                            uint32_t services_handle)
{
  uint32_t error_code = SERVICES_REQ_SUCCESS;
  int32_t service_error_code;
  uint32_t rnd_value;

  error_code = SERVICES_cryptocell_get_rnd(services_handle,
                                           sizeof(uint32_t),/* random number/vector length in bytes*/
                                           &rnd_value,
                                           &service_error_code);
  TEST_print(services_handle,
             "** TEST %s error_code=%s 32-bit Random value = 0x%08x service_resp=%d\n",
             p_test_name,
             SERVICES_error_to_string(error_code),
             rnd_value,
             service_error_code);

  return error_code;
}

/**
 * Crypto Services
 */
/**
 * @fn    static uint32_t test_services_crypto_lcs(uint32_t services_handle)
 * @param services_handle
 * @return
 */
static uint32_t test_services_crypto_lcs(char *p_test_name,
                                         uint32_t services_handle)
{
  uint32_t error_code = SERVICES_REQ_SUCCESS;
  int32_t service_error_code;
  uint32_t lcs_state = 0xdeadbeef;

  error_code = SERVICES_cryptocell_get_lcs(services_handle,
                                           &lcs_state,
                                           &service_error_code);
  TEST_print(services_handle,
             "** TEST %s error_code=%s service_resp=0x%08X\n",
             p_test_name,
             SERVICES_error_to_string(error_code),
             lcs_state,
             service_error_code);

  return (error_code);
}

/**
 *  MAINTENANCE Services
 */
/**
 * @fn    static uint32_t test_services_heartbeat(uint32_t services_handle)
 * @brief test heartbeat
 * @param services_handle
 * @return
 */
static uint32_t test_services_heartbeat(char *p_test_name,
                                        uint32_t services_handle)
{
  uint32_t error_code = SERVICES_REQ_SUCCESS;

  error_code = SERVICES_heartbeat(services_handle);
  TEST_print(services_handle,
             "** TEST %s error_code=%s\n",
             p_test_name,
             SERVICES_error_to_string(error_code));

  return (error_code);
}

/**
  * System management Services
  */
static uint32_t test_services_gettoc(char *p_test_name,
                                     uint32_t services_handle)
{
  uint32_t error_code = SERVICES_REQ_SUCCESS;
  uint32_t service_error_code;
  uint32_t number_of_tocs = 0;

  error_code = SERVICES_system_get_toc_number(services_handle,
                                              &number_of_tocs,
                                              &service_error_code);
  TEST_print(services_handle,
             "** TEST %s error_code=%s Application TOC number = %d " \
             "service_resp=0x%08X\n",
             p_test_name,
             SERVICES_error_to_string(error_code),
             number_of_tocs,
             service_error_code);

  return (error_code);
}

static uint32_t test_services_getotp(char *p_test_name,
                                     uint32_t services_handle)
{
  uint32_t error_code = SERVICES_REQ_SUCCESS;
  uint32_t service_error_code;

  error_code = SERVICES_system_get_otp_data(services_handle,
                                            &otp_info,
                                            &service_error_code);
  TEST_print(services_handle,
             "** TEST %s error_code=%s service_resp=0x%08X\n",
             p_test_name,
             SERVICES_error_to_string(error_code),
             service_error_code);

  return error_code;
}

/**
 * @brief Read OTP
 *
 * @param p_test_name
 * @param services_handle
 * @return
 */
static uint32_t test_services_read_otp(char *p_test_name,
                                       uint32_t services_handle)
{
  uint32_t error_code = SERVICES_REQ_SUCCESS;
  uint32_t service_error_code;
  uint32_t otp_value;
  int otp_row;

  /**
   * start with an illegal offset you should not be trying
   */
  error_code = SERVICES_system_read_otp(services_handle,
                                        0x00,
                                        &otp_value,
                                        &service_error_code);
  TEST_print(services_handle,
             "** TEST %s error_code=%s service_resp=%s\n",
             p_test_name,
             SERVICES_error_to_string(error_code),
             SERVICES_error_to_string(service_error_code));

   /**
    * Programmable Device Serial Number. Sequential Device serial number
    * OTP Word Address = 0x0005C, Size = 8 bytes
    */
   for (otp_row=OTP_MANUFACTURE_INFO_SERIAL_NUMBER_START;
        otp_row<=OTP_MANUFACTURE_INFO_SERIAL_NUMBER_END; otp_row++)
   {
     error_code = SERVICES_system_read_otp(services_handle,
                                           otp_row,
                                           &otp_value,
                                           &service_error_code);
     TEST_print(services_handle,
                "    ** OTP offset 0x%0x OTP Value = 0x%08X\n",
                otp_row,
                otp_value);
   }
   return error_code;
}

/**
 * @brief TOC Test - get all TOC data
 *
 * @param p_test_name
 * @param services_handle
 * @return
 */
static uint32_t test_services_gettoc_data(char *p_test_name,
                                          uint32_t services_handle)
{
  uint32_t error_code = SERVICES_REQ_SUCCESS;
  uint32_t service_error_code;
  uint32_t each_toc;

  error_code = SERVICES_system_get_toc_data(services_handle,
                                            &toc_info,
                                            &service_error_code);
  TEST_print(services_handle,
             "** TEST %s error_code=%s TOC number = %d service_resp=0x%08X\n",
             p_test_name,
             SERVICES_error_to_string(error_code),
             toc_info.number_of_toc_entries,
             service_error_code);

  TEST_print(services_handle,
             "+-------------------------------------------------------------------------------+\n");
  TEST_print(services_handle,
             "|   Name   |    CPU   |Load Address|Boot Address|Image Size|Version|    Flags   |\n");
  TEST_print(services_handle,
             "+-------------------------------------------------------------------------------+\n");

  for (each_toc = 0; each_toc < toc_info.number_of_toc_entries ; each_toc++)
  {
    char flags_string[FLAG_STRING_SIZE] = {0}; /* Flags as string   */

    TEST_print(services_handle,
               "| %8s |  %6s  | 0x%08X | 0x%08X | %8d | %d.%d.%d | %10s |\n",
               toc_info.toc_entry[each_toc].image_identifier,
               CPUID_to_string(toc_info.toc_entry[each_toc].cpu),
               toc_info.toc_entry[each_toc].load_address,
               toc_info.toc_entry[each_toc].boot_address,
               toc_info.toc_entry[each_toc].image_size,
               ((toc_info.toc_entry[each_toc].version >> 24) & 0x0F),
               ((toc_info.toc_entry[each_toc].version >> 16) & 0x0F),
               ((toc_info.toc_entry[each_toc].version >>  8) & 0x0F),
               flags_to_string(toc_info.toc_entry[each_toc].flags,
                               flags_string));
  }
  TEST_print(services_handle,
             "+-------------------------------------------------------------------------------+\n");

  return error_code;
}

/**
 * @brief * @brief TOC test - get via name M55_HE
 *
 * @param p_test_name
 * @param services_handle
 * @return
 */
static uint32_t test_services_gettoc_via_name_m55_he(char *p_test_name,
                                                     uint32_t services_handle)
{
#if CPU != A32
  uint32_t error_code = SERVICES_REQ_SUCCESS;
  uint32_t service_error_code;

  error_code = SERVICES_system_get_toc_via_name(services_handle,
                                                (uint8_t *)"M55-HE",
                                                &service_error_code);
  TEST_print(services_handle,
             "** TEST %s error_code=%s service_resp=0x%08X\n",
             p_test_name,
             SERVICES_error_to_string(error_code),
             service_error_code);

  return error_code;
#else
  (void)(p_test_name);
  (void)(services_handle);
  return SERVICES_RESP_UNKNOWN_COMMAND;
#endif // #if CPU != A32
}

/**
 * @brief TOC test - get via name M55_HP
 *
 * @param p_test_name
 * @param services_handle
 * @return
 */
static uint32_t test_services_gettoc_via_name_m55_hp(char *p_test_name,
                                                     uint32_t services_handle)
{
#if CPU != A32
  uint32_t error_code = SERVICES_REQ_SUCCESS;
  uint32_t service_error_code;

  error_code = SERVICES_system_get_toc_via_name(services_handle,
                                                (uint8_t *)"M55-HP",
                                                &service_error_code);
  TEST_print(services_handle,
             "** TEST %s error_code=%s service_resp=0x%08X\n",
             p_test_name,
             SERVICES_error_to_string(error_code),
             service_error_code);

  return error_code;
#else
  (void)(p_test_name);
  (void)(services_handle);
  return SERVICES_RESP_UNKNOWN_COMMAND;
#endif // #if CPU != A32
}

/**
 * @brief TOC Test - get version
 *
 * @param p_test_name
 * @param services_handle
 * @return
 */
static uint32_t test_services_gettoc_version(char *p_test_name,
                                             uint32_t services_handle)
{
  uint32_t error_code = SERVICES_REQ_SUCCESS;
  uint32_t version = 0;                  /*<! Returned version */
  uint32_t service_error_code;

  error_code = SERVICES_system_get_toc_version(services_handle,
                                               &version,
                                               &service_error_code);
  TEST_print(services_handle,
             "** TEST %s error_code=%s service_resp=0x%08X version=%X\n",
             p_test_name,
             SERVICES_error_to_string(error_code),
             service_error_code,
             version);

  return error_code;
}

/**
 * @brief SYSTEM Test - get TOC data via CPU ID HE
 * @param p_test_name
 * @param services_handle
 * @return
 */
static uint32_t test_services_gettoc_via_cpuid_he(char *p_test_name,
                                                  uint32_t services_handle)
{
  uint32_t error_code = SERVICES_REQ_SUCCESS;
  uint32_t service_error_code;

  error_code = SERVICES_system_get_toc_via_cpuid(services_handle,
                                                 FUSION_M55_HE,
                                                 &toc_info,
                                                 &service_error_code);

  TEST_print(services_handle,
             "** TEST %s error_code=%s Found %d TOCs for CPU\n",
             p_test_name,
             SERVICES_error_to_string(error_code),
             toc_info.number_of_toc_entries);
  for (uint32_t each_toc = 0; each_toc < toc_info.number_of_toc_entries; each_toc++)
  {
    SERVICES_toc_info_t *toc_entry_p;

    toc_entry_p = (SERVICES_toc_info_t *)&toc_info.toc_entry[each_toc];

    TEST_print(services_handle,
               "    -> Name %8s flags %X Version %X %X %X\n",
               toc_entry_p->image_identifier,
               toc_entry_p->flags,
               ((toc_entry_p->version >> 24) & 0x0F),
               ((toc_entry_p->version >> 16) & 0x0F),
               ((toc_entry_p->version >>  8) & 0x0F));
  }

  return error_code;
}

/**
 * @brief SYSTEM Test - get TOC data via CPU ID HP
 * @param p_test_name
 * @param services_handle
 * @return
 */
static uint32_t test_services_gettoc_via_cpuid_hp(char *p_test_name,
                                                  uint32_t services_handle)
{
  uint32_t error_code = SERVICES_REQ_SUCCESS;
  uint32_t service_error_code;

  error_code = SERVICES_system_get_toc_via_cpuid(services_handle,
                                                 FUSION_M55_HP,
                                                 &toc_info,
                                                 &service_error_code);
  TEST_print(services_handle,
             "** TEST %s error_code=%s Found %d TOCs for CPU\n",
             p_test_name,
             SERVICES_error_to_string(error_code),
             toc_info.number_of_toc_entries);
  for (uint32_t each_toc = 0; each_toc < toc_info.number_of_toc_entries; each_toc++)
  {
    SERVICES_toc_info_t *toc_entry_p;

    toc_entry_p = (SERVICES_toc_info_t *)&toc_info.toc_entry[each_toc];

    TEST_print(services_handle,
               "    -> Name %8s flags %X Version %X %X %X\n",
               toc_entry_p->image_identifier,
               toc_entry_p->flags,
               ((toc_entry_p->version >> 24) & 0x0F),
               ((toc_entry_p->version >> 16) & 0x0F),
               ((toc_entry_p->version >>  8) & 0x0F));
  }

  return error_code;
}

/**
 * @brief SYSTEM Test - get TOC data via CPU ID A32
 * @param p_test_name
 * @param services_handle
 * @return
 */
static uint32_t test_services_gettoc_via_cpuid_a32(char *p_test_name,
                                                  uint32_t services_handle)
{
  uint32_t error_code = SERVICES_REQ_SUCCESS;
  uint32_t service_error_code;

  error_code = SERVICES_system_get_toc_via_cpuid(services_handle,
                                                 FUSION_A32_0,
                                                 &toc_info,
                                                 &service_error_code);
  TEST_print(services_handle,
             "** TEST %s error_code=%s Found %d TOCs for CPU\n",
             p_test_name,
             SERVICES_error_to_string(error_code),
             toc_info.number_of_toc_entries);
  for (uint32_t each_toc = 0; each_toc < toc_info.number_of_toc_entries; each_toc++)
  {
    SERVICES_toc_info_t *toc_entry_p;

    toc_entry_p = (SERVICES_toc_info_t *)&toc_info.toc_entry[each_toc];

    TEST_print(services_handle,
               "    -> Name %8s flags %X Version %X %X %X\n",
               toc_entry_p->image_identifier,
               toc_entry_p->flags,
               ((toc_entry_p->version >> 24) & 0x0F),
               ((toc_entry_p->version >> 16) & 0x0F),
               ((toc_entry_p->version >>  8) & 0x0F));
  }

  return error_code;
}

/**
 * @brief Boot Services - get SE revision
 * @param p_test_name
 * @param services_handle
 * @return
 */
static uint32_t test_services_get_se_revision(char *p_test_name,
                                              uint32_t services_handle)
{
  uint32_t error_code = SERVICES_REQ_SUCCESS;
  uint32_t service_error_code = 0;
  uint8_t se_revision[80] = {0};

  error_code = SERVICES_get_se_revision(services_handle,
                                        (uint8_t*)&se_revision[0],
                                        &error_code);
  TEST_print(services_handle,
             "** TEST %s error_code=%s service_resp=0x%08X\n",
             p_test_name,
             SERVICES_error_to_string(error_code),
             service_error_code);
  TEST_print(services_handle,
             "     ** %s\n",
             se_revision);

  return error_code;
}

static uint32_t test_services_get_socid(char *p_test_name,
                                        uint32_t services_handle)
{
  uint32_t error_code = SERVICES_REQ_SUCCESS;
  uint32_t service_error_code;
  uint32_t device_part_number;

  error_code = SERVICES_system_get_device_part_number(services_handle,
                                                      &device_part_number,
                                                      &service_error_code);
  TEST_print(services_handle,
             "** TEST %s error_code=%s Device number 0x%X service_resp=0x%08X\n",
             p_test_name,
             SERVICES_error_to_string(error_code),
             device_part_number,
             service_error_code);

  return error_code;
}

static uint32_t test_services_ewic_config(char *p_test_name,
                                          uint32_t services_handle)
{
  uint32_t error_code = SERVICES_REQ_SUCCESS;

  error_code = SERVICES_power_ewic_config(services_handle,
                                          EWIC_RTC_SE | EWIC_RTC_A,
                                          LOWEST_POWER_PROFILE);

  TEST_print(services_handle,
             "** TEST %s error_code=%s Device number 0x%X service_resp=0x%08X\n",
             p_test_name,
             SERVICES_error_to_string(error_code));

  return error_code;

}

static uint32_t test_services_vbat_wakeup_config(char *p_test_name,
                                                 uint32_t services_handle)
{
  uint32_t error_code = SERVICES_REQ_SUCCESS;

  error_code = SERVICES_power_wakeup_config(services_handle,
                                            VBAT_WAKEUP_RTC_SE
                                            | VBAT_WAKEUP_RTC_A,
                                            LOWEST_POWER_PROFILE);

  TEST_print(services_handle,
             "** TEST %s error_code=%s Device number 0x%X service_resp=0x%08X\n",
             p_test_name,
             SERVICES_error_to_string(error_code));

  return error_code;
}

static uint32_t test_services_mem_retention_config(char *p_test_name,
                                                      uint32_t services_handle)
{
  uint32_t error_code = SERVICES_REQ_SUCCESS;

  error_code = SERVICES_power_mem_retention_config(services_handle,
                                                   POWER_MEM_RET_SE_SRAM,
                                                   LOWEST_POWER_PROFILE);

  TEST_print(services_handle,
             "** TEST %s error_code=%s Device number 0x%X service_resp=0x%08X\n",
             p_test_name,
             SERVICES_error_to_string(error_code));

  return error_code;
}

/**
 * @brief BOOT TOC A32
 *
 * @param p_test_name
 * @param services_handle
 * @return
 */
static uint32_t test_services_boot_toc_a32(char *p_test_name,
                                           uint32_t services_handle)
{
  uint32_t error_code = SERVICES_REQ_SUCCESS;
  uint32_t service_error_code;
  uint8_t entry_id[8];

  memset(entry_id, 0x0, sizeof(entry_id));
  strcpy((char *)entry_id, "A32_3");

  error_code = SERVICES_boot_process_toc_entry(services_handle,
                                               entry_id,
                                               &service_error_code);

  TEST_print(services_handle,
             "** TEST %s error_code=%s service_resp=0x%08X\n",
             p_test_name,
             SERVICES_error_to_string(error_code),
             service_error_code);

  return error_code;
}

/**
 * @brief BOOT TOC ExtSys0
 *
 * @param p_test_name
 * @param services_handle
 * @return
 */
static uint32_t test_services_boot_release_extsys0(char *p_test_name,
                                                   uint32_t services_handle)
{
  uint32_t error_code = SERVICES_REQ_SUCCESS;
  uint32_t service_error_code;

  error_code = SERVICES_boot_release_cpu(services_handle,
                                         FUSION_EXTERNAL_SYS0,
                                         &service_error_code);

  TEST_print(services_handle,
             "** TEST %s error_code=%s service_resp=0x%08X\n",
             p_test_name,
             SERVICES_error_to_string(error_code),
             service_error_code);

  return error_code;
}

/**
 * @brief MBEDTLS AES
 *
 * @param p_test_name
 * @param services_handle
 * @return
 */
static uint32_t test_services_mbedtls_aes(char *p_test_name,
                                  uint32_t services_handle)
{
  /*
   * In C++, const really means const and can be used to declare static array sizes
   * In C, it is not a constant variable expression
   */
#define KEY_SIZE 256
#define AES_IV_SIZE 16
#define AES_BLOCK_SIZE 16

  /* https://csrc.nist.gov/CSRC/media/Projects/Cryptographic-Standards-and-Guidelines/documents/examples/AES_OFB.pdf */
  static const uint8_t IV[] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
                                0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F };
  static const uint8_t PLAIN[] = { 0x6B, 0xC1, 0xBE, 0xE2, 0x2E, 0x40, 0x9F,
                                   0x96, 0xE9, 0x3D, 0x7E, 0x11, 0x73, 0x93,
                                   0x17, 0x2A };
  static const uint8_t KEY[] = { 0x60, 0x3D, 0xEB, 0x10, 0x15, 0xCA, 0x71, 0xBE,
                                 0x2B, 0x73, 0xAE, 0xF0, 0x85, 0x7D, 0x77, 0x81,
                                 0x1F, 0x35, 0x2C, 0x07, 0x3B, 0x61, 0x08, 0xD7,
                                 0x2D, 0x98, 0x10, 0xA3, 0x09, 0x14, 0xDF, 0xF4 };
  static const uint8_t CYPHER[] = { 0xDC, 0x7E, 0x84, 0xBF, 0xDA, 0x79, 0x16,
                                    0x4B, 0x7E, 0xCD, 0x84, 0x86, 0x98, 0x5D,
                                    0x38, 0x60 };

  uint8_t key[KEY_SIZE / 8];
  uint8_t iv[AES_IV_SIZE];
  uint8_t buf[AES_BLOCK_SIZE];
  
  uint32_t aes_ctx[24];

  // const char* TEST_NAME = "AES-OFB-256";

  uint32_t error_code = SERVICES_REQ_SUCCESS;
  uint32_t service_error_code;

  /*currently unused*/
  (void)(p_test_name);

  // Initialize aes engine
  SERVICES_cryptocell_mbedtls_aes_init(services_handle, &service_error_code,
                                       (uint32_t)aes_ctx);

  memcpy(key, KEY, sizeof(key));
  memcpy(buf, PLAIN, sizeof(buf));
  memcpy(iv, IV, sizeof(iv));

  TEST_print(services_handle,
             "** TEST SERVICES_cryptocell_mbedtls_aes_init    error_code=%s service_resp=0x%08X\n",
             SERVICES_error_to_string(error_code),
             service_error_code);

  // encrypt
  // set key into context
  SERVICES_cryptocell_mbedtls_aes_set_key(services_handle, 
                                          &service_error_code, 
                                          (uint32_t)aes_ctx, 
                                          (uint32_t)key, 
                                          KEY_SIZE,
                                          MBEDTLS_OP_ENCRYPT);
  TEST_print(services_handle,
             "** TEST SERVICES_cryptocell_mbedtls_aes_set_key error_code=%s service_resp=0x%08X\n",
             SERVICES_error_to_string(error_code),
             service_error_code);

  // perform cryptographic operation
  //mbedtls_aes_crypt_ofb(&ctx, sizeof(buf), 0, iv, buf, buf);
  SERVICES_cryptocell_mbedtls_aes_crypt(services_handle, 
                                        &service_error_code, 
                                        (uint32_t)aes_ctx,
                                        MBEDTLS_AES_CRYPT_OFB, 
                                        0, 
                                        sizeof(buf), 
                                        (uint32_t)iv, 
                                        (uint32_t)buf, 
                                        (uint32_t)buf);

  memcpy(buf, CYPHER, sizeof(buf));
  memcpy(iv, IV, sizeof(iv));
  TEST_print(services_handle,
             "** TEST SERVICES_cryptocell_mbedtls_aes_crypt   error_code=%s service_resp=0x%08X\n",
             SERVICES_error_to_string(error_code),
             service_error_code);
  // decrypt
  // set key into context
  //mbedtls_aes_setkey_dec(&ctx, key, KEY_SIZE);
  SERVICES_cryptocell_mbedtls_aes_set_key(services_handle, 
                                          &service_error_code, 
                                          (uint32_t)aes_ctx, 
                                          (uint32_t)key, 
                                          KEY_SIZE,
                                          MBEDTLS_OP_DECRYPT);
  TEST_print(services_handle,
             "** TEST SERVICES_cryptocell_mbedtls_aes_set_key error_code=%s service_resp=0x%08X\n",
             SERVICES_error_to_string(error_code),
             service_error_code);

  // perform cryptographic operation
  //mbedtls_aes_crypt_ofb(&ctx, sizeof(buf), 0, iv, buf, buf);
  SERVICES_cryptocell_mbedtls_aes_crypt(services_handle, 
                                        &service_error_code, 
                                        (uint32_t)aes_ctx,
                                        MBEDTLS_AES_CRYPT_OFB, 
                                        0, 
                                        sizeof(buf), 
                                        (uint32_t)iv, 
                                        (uint32_t)buf, 
                                        (uint32_t)buf);
  TEST_print(services_handle,
             "** TEST SERVICES_cryptocell_mbedtls_aes_crypt   error_code=%s service_resp=0x%08X\n",
             SERVICES_error_to_string(error_code),
             service_error_code);

  return error_code;
}

/**
 * @brief Bound check tests - UART
 *
 * @param p_test_name
 * @param services_handle
 * @return
 */
static uint32_t test_services_bounds(char *p_test_name,
                                     uint32_t services_handle)
{
  uint32_t error_code = SERVICES_REQ_SUCCESS;
  uint8_t buffer[PRINT_BUFFER_SIZE] = { 0 };

  error_code = SERVICES_uart_write(services_handle,
                                   0,
                                   (uint8_t*)&buffer[0]);
  TEST_print(services_handle,
             "** TEST %s error_code=%s service_resp=0x%08X\n",
             p_test_name,
             SERVICES_error_to_string(error_code));

  error_code = SERVICES_uart_write(services_handle,
                                   PRINT_BUFFER_SIZE+1,
                                   (uint8_t*)&buffer[0]);
  TEST_print(services_handle,
             "** TEST %s error_code=%s service_resp=0x%08X\n",
             p_test_name,
             SERVICES_error_to_string(error_code));

  error_code = SERVICES_uart_write(services_handle,
                                     20,
                                     (uint8_t*)NULL);

  TEST_print(services_handle,
               "** TEST %s error_code=%s service_resp=0x%08X\n",
               p_test_name,
               SERVICES_error_to_string(error_code));

  return error_code;
}

/**
 * @brief OSPI Key write test
 *
 * @param p_test_name
 * @param services_handle
 * @return
 */
static uint32_t test_services_ospi_write_key(char *p_test_name,
                                             uint32_t services_handle)
{
  uint32_t error_code = SERVICES_REQ_SUCCESS;
  uint32_t service_error_code = 0;
  uint8_t key[16] = {0x0, 0x1, 0x2, 0x3, 0x4,
                     0x5, 0x6, 0x7, 0x8, 0x9,
                     0xA, 0xB, 0xC, 0xD, 0xE,
                     0xF};
  uint32_t commands[4] = {
      OSPI_WRITE_OTP_KEY_OSPI0, OSPI_WRITE_OTP_KEY_OSPI1,
      OSPI_WRITE_EXTERNAL_KEY_OSPI0, OSPI_WRITE_EXTERNAL_KEY_OSPI1
  };

  for (int i = 0; i < 4; i++)
  {
    error_code = SERVICES_application_ospi_write_key(services_handle,
                                                     commands[i],
                                                     key, 
                                                     &error_code);
    TEST_print(services_handle,
               "** TEST %s command=0x%X error_code=%s service_resp=0x%08X\n",
               p_test_name,
               commands[i],
               SERVICES_error_to_string(error_code),
               service_error_code);
  }

  return error_code;
}

static uint32_t test_services_set_es0_frequency(char *p_test_name, uint32_t services_handle)
{
  uint32_t error_code = SERVICES_REQ_SUCCESS;
  uint32_t service_error_code;

  clock_frequency_t frequency = CLOCK_FREQUENCY_400MHZ;
  error_code = SERVICES_clocks_set_ES0_frequency(services_handle,
                                                 frequency,
                                                 &service_error_code);

  TEST_print(services_handle,
             "** TEST %s error_code=%s service_resp=0x%08X\n",
             p_test_name,
             SERVICES_error_to_string(error_code),
             service_error_code);

  return error_code;
}

/**
 * @fn    uint32_t test_services_set_divider(char *p_test_name, uint32_t services_handle)
 * @brief test harness for supported SERVICE APIS
 */
static uint32_t test_services_set_divider(char *p_test_name, uint32_t services_handle)
{
  uint32_t error_code = SERVICES_REQ_SUCCESS;
  uint32_t service_error_code;

  error_code = SERVICES_clocks_set_divider(services_handle,
                                           DIVIDER_CPUPLL,
                                           0x0, // divide by 0
                                           &service_error_code);

  TEST_print(services_handle,
             "** TEST %s error_code=%s service_resp=0x%08X\n",
             p_test_name,
             SERVICES_error_to_string(error_code),
             service_error_code);

  return error_code;

}
/**
 * @fn    void SERVICES_test(uint32_t services_handle)
 * @brief test harness for supported SERVICE APIS
 */
void SERVICES_test_guts(uint32_t services_handle)
{
  uint32_t service_error_code = SERVICES_REQ_SUCCESS;

  /* Disable tracing output for services */
  SERVICES_system_set_services_debug(services_handle, false,
                                     &service_error_code);

  /* show services version */
  TEST_print(services_handle, "[%s] SERVICES version %s\n",
             CPU_STRING, SERVICES_version());

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
    (void)error_code;
  }
}

/**
 * @fn    void SERVICES_test(uint32_t services_handle)
 * @brief test harness for supported SERVICE APIS
 */
void SERVICES_test(uint32_t services_handle)
{
  int retry_count;

  /* keep sending heartbeat services requests until one succeeds */
  retry_count = SERVICES_synchronize_with_se(services_handle);
  TEST_print(services_handle, "SERVICES_synchronize_with_se() returned %d\n", retry_count);

  SERVICES_test_guts(services_handle);
}
