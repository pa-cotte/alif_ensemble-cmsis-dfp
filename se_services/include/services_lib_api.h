/**
 * @file services_lib_api.h
 *
 * @brief Services library public API header file
 * @defgroup host_services Host Services
 * @par
 *
 * Copyright (C) 2022 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */
#ifndef __SERVICES_LIB_API_H__
#define __SERVICES_LIB_API_H__

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 *  I N C L U D E   F I L E S
 *****************************************************************************/
#include <stddef.h>
#include <stdbool.h>
#include "services_lib_protocol.h"

/*******************************************************************************
 *  M A C R O   D E F I N E S
 ******************************************************************************/

/**
 * Error codes
 */
#define SERVICES_REQ_SUCCESS                       0x00
#define SERVICES_INVALID_ADDRESS                   0x10
#define SERVICES_REQ_NOT_ACKNOWLEDGE               0xFF
#define SERVICES_REQ_TIMEOUT                       0xFD
#define SERVICES_RESP_UNKNOWN_COMMAND              0xFC

/**
 * OTP Offsets
 */
#define OTP_MANUFACTURE_INFO_DATA_START            0x51
#define OTP_MANUFACTURE_INFO_DATA_END              0x58
#define OTP_MANUFACTURE_INFO_SERIAL_NUMBER_START   0x59
#define OTP_MANUFACTURE_INFO_SERIAL_NUMBER_END     0x5A

#define OTP_MANUFACTURE_INFO_OPTIONAL_LENGTH_BYTES 32
#define OTP_MANUFACTURE_INFO_SERIAL_NUMBER_LENGTH_BYTES  8

#define OTP_OSPI_KEY_OSPI0                         0x60
#define OTP_OSPI_KEY_OSPI1                         0x64
#define OSPI_KEY_LENGTH_BYTES                      16

#define SERVICES_MAX_PACKET_BUFFER_SIZE            2048

/**
 * MBED TLS
 */
#define MBEDTLS_OP_DECRYPT                         0
#define MBEDTLS_OP_ENCRYPT                         1

#define MBEDTLS_AES_CRYPT_ECB                      0
#define MBEDTLS_AES_CRYPT_CBC                      1
#define MBEDTLS_AES_CRYPT_CTR                      2
#define MBEDTLS_AES_CRYPT_OFB                      3

#define MBEDTLS_HASH_SHA1                          0
#define MBEDTLS_HASH_SHA224                        1
#define MBEDTLS_HASH_SHA256                        2

#define MBEDTLS_CCM_KEY                            0
#define MBEDTLS_GCM_KEY                            1

#define MBEDTLS_CCM_ENCRYPT_AND_TAG                0
#define MBEDTLS_CCM_AUTH_DECRYPT                   1
#define MBEDTLS_CCM_STAR_ENCRYPT_AND_TAG           2
#define MBEDTLS_CCM_STAR_AUTH_DECRYPT              3
#define MBEDTLS_GCM_ENCRYPT_AND_TAG                4
#define MBEDTLS_GCM_DECRYPT_AND_TAG                5
#define MBEDTLS_GCM_AUTH_DECRYPT                   6

#define MBEDTLS_CHACHAPOLY_ENCRYPT_AND_TAG         0
#define MBEDTLS_CHACHAPOLY_AUTH_DECRYPT            1

#define SERVICES_NUMBER_OF_TOC_ENTRIES             15

/**
 * OSPI write key commands
 */
#define OSPI_WRITE_OTP_KEY_OSPI0                   0
#define OSPI_WRITE_OTP_KEY_OSPI1                   1
#define OSPI_WRITE_EXTERNAL_KEY_OSPI0              2
#define OSPI_WRITE_EXTERNAL_KEY_OSPI1              3

/**
 * TOC related
 */
#define TOC_NAME_LENGTH                            8

/**
 * Pin muxing/pad control error codes - follow the pattern from the PLL services
 */
#define PINMUX_SUCCESS                             0x0
#define PINMUX_ERROR_INVALID_PARAMETER             0x200

/**
 * Memory retention bit encoding for mem_retention_enable
 */
#define POWER_MEM_RET_FIREWALL_RAM             0x01UL // bit0
#define POWER_MEM_RET_SE_SRAM                  0x02UL // bit1
#define POWER_MEM_RET_BACKUP_RAM_4KB           0x04UL // bit2
// M55-HE TCM RET1: ITCM 0-128kb; DTCM 0-128kb
#define POWER_MEM_RET_ES1_TCM_RET1             0x08UL // bit3
// M55-HE TCM RET1: ITCM 128-256kb; DTCM 128-256kb
#define POWER_MEM_RET_ES1_TCM_RET2             0x10UL // bit4
// XTENSA TCM RET1: ITCM 128-512kb
#define POWER_MEM_RET_XTENSA_TCM_RET1          0x20UL // bit5
// XTENSA TCM RET1: ITCM 64-128kb
#define POWER_MEM_RET_XTENSA_TCM_RET2          0x40UL // bit6
// XTENSA TCM RET1: ITCM 0-64kb
#define POWER_MEM_RET_XTENSA_TCM_RET3          0x80UL // bit7
// M55-M TCM RET1: ITCM 1MB; DTCM 384kb
#define POWER_MEM_RET_M55_M_TCM_RET1           0x100UL // bit8
#define POWER_MEM_RET_MODEM_BACKUP_RAM_16KB    0x200UL // bit9

/**
 * Global standby configuration macros
 */

/*
 * Host CPU Cluster Power Request HOST_CPU_CLUS_PWR_REQ
 */
// MEM_RET_REQ
#define MEM_RET_REQ_LAST_LEVEL_CACHE_RET_OFF     0x0
#define MEM_RET_REQ_LAST_LEVEL_CACHE_RET_ON      0x1
// PWR_REQ
#define PWR_REQ_CLUSTOP_LOW_POWER_ON             0x0
#define PWR_REQ_CLUSTOP_FUNC_RET_ON              0x1

/*
 * Base System Power Request BSYS_PWR_REQ
 */
// SYSTOP_PWR_REQ
#define SYSTOP_PWR_REQ_LOGIC_OFF_MEM_OFF         0x0
#define SYSTOP_PWR_REQ_LOGIC_OFF_MEM_RET         0x1
#define SYSTOP_PWR_REQ_LOGIC_ON_MEM_ON_OR_RET    0x2
#define SYSTOP_PWR_REQ_LOGIC_ON_MEM_ON           0x4
// DBGTOP_PWR_REQ
#define DBGTOP_PWR_REQ_OFF                       0x0
#define DBGTOP_PWR_REQ_ON                        0x1
// REFCLK_REQ
#define REFCLK_REQ_OFF                           0x0
#define REFCLK_REQ_ON                            0x1
// WAKEUP_EN
#define WAKEUP_EN_SE_OFF                         0x0
#define WAKEUP_EN_SE_ON                          0x1

/**
 * @brief Power / retention error codes
 */
#define ERROR_POWER_SRAM_RETENTION_INVALID      0x100

/*******************************************************************************
 *  T Y P E D E F S
 ******************************************************************************/

typedef int32_t (*wait_ms_t)(uint32_t wait_time_ms);
typedef int (*print_msg_t)(const char * fmt, ...);

/**
 *  @enum SERVICES_cpuid_t
 */
typedef enum
{
  FUSION_A32_0   = 0,                /**< A32_0 CPU         */
  FUSION_A32_1   = 1,                /**< A32_1 CPU         */
  FUSION_M55_HP  = 2,                /**< M55 HP CPU        */
  FUSION_M55_HE  = 3,                /**< M55 HE CPU        */
  FUSION_EXTERNAL_SYS0 = 4           /**< CPU in Ext SYS0   */
} SERVICES_cpuid_t;

/**
 *  @struct SERVICES_toc_info_t
 */
typedef struct
{
  uint8_t   image_identifier[TOC_NAME_LENGTH]; /**< TOC name         */
  uint32_t  version;                  /**< TOC Version      */
  uint32_t  cpu;                      /**< TOC Cpu ID       */
  uint32_t  store_address;            /**< TOC MRAM address */
  uint32_t  load_address;             /**< TOC load         */
  uint32_t  boot_address;             /**< TOC boot address */
  uint32_t  image_size;               /**< TOC image size   */
  uint32_t  flags;                    /**< TOC flag state   */
} SERVICES_toc_info_t;

/**
 * @struct SERVICES_otp_data_t
 * @todo   not all OTP fields are exposed, TBD
 *         ALIF serial number and manufacturing data is TBD
 */
typedef struct
{
  uint32_t  otp_alif_manufacturing_data[4];         /**< OTP ALIF Fab, Lot number.. */
  uint32_t  otp_alif_manufacturing_serial_number[2];/**< OTP ALIF Serial number     */
  uint8_t   otp_alif_manufacturing_part_number[16]; /**< OTP part number            */
  uint32_t  otp_alif_hbk_0[3];                      /**< OTP HBK0                   */
  uint32_t  otp_alif_hbk_1[3];                      /**< OTP HBK1                   */
  uint32_t  otp_alif_firmware_version_dcu[10];      /**< OTP FW, flags, DCU lockmask*/
} SERVICES_otp_data_t;

/**
 * @struct SERVICES_toc_data_t
 */
typedef struct
{
  uint32_t number_of_toc_entries;     /**< Number of real TOC objects */
  SERVICES_toc_info_t toc_entry[SERVICES_NUMBER_OF_TOC_ENTRIES]; /* TOC details */
} SERVICES_toc_data_t;

/**
 * EWIC
 */
typedef enum
{
  EWIC_RTC_SE               = 0x1,          // bit0
  EWIC_MODEM                = 0x2,          // bit1
  EWIC_MODEM_TO_SE_IRQ      = 0xC,          // bit3:2
  EWIC_MODEM_PPU_IRQ        = 0x10,         // bit4
  EWIC_MODEM_WARM_RESET_REQ = 0x20,         // bit5
  EWIC_RTC_A                = 0x40,         // bit6
  EWIC_VBAT_TIMER           = 0x780,        // bit10:7
  EWIC_VBAT_GPIO            = 0x7F800,      // bit18:11
  EWIC_VBAT_LP_CMP_IRQ      = 0x00080000UL, // bit19
  EWIC_ES1_LP_I2C_IRQ       = 0x00100000UL, // bit20
  EWIC_ES1_LP_UART_IRQ      = 0x00200000UL, // bit21
  EWIC_BROWN_OUT            = 0x00400000UL, // bit22
} SERVICES_ewic_cfg_t;

/**
 * WAKE UP
 */
typedef enum
{
  VBAT_WAKEUP_MDM           = 0x1,          // bit0
  VBAT_WAKEUP_RTC_SE        = 0x10,         // bit4
  VBAT_WAKEUP_RTC_A         = 0x20,         // bit5
  VBAT_WAKEUP_LPCMP         = 0x40,         // bit6
  VBAT_WAKEUP_BROWN_OUT     = 0x80,         // bit7
  VBAT_WAKEUP_LPTIMER       = 0XF00,        // bit11:8
  VBAT_WAKEUP_LPGPIO        = 0XFF0000,     // bit23:16
} SERVICES_wakeup_cfg_t;



/**
 * @struct Power profiles
 */
typedef enum
{
  LOWEST_POWER_PROFILE = 0,      /**< LOWEST_POWER_PROFILE */
  HIGH_PERFORMANCE_POWER_PROFILE,/**< HIGH_PERFORMANCE_POWER_PROFILE */
  USER_SPECIFIED_PROFILE,        /**< USER_SPECIFIED_PROFILE */
  DEFAULT_POWER_PROFILE,         /**< DEFAULT_POWER_PROFILE */
  NUMBER_OF_POWER_PROFILES       /**< NUMBER_OF_POWER_PROFILES */
} services_power_profile_t;

// Clocks Services definitions
typedef enum
{
  CLOCK_FREQUENCY_800MHZ,        /* Application CPU values */
  CLOCK_FREQUENCY_400MHZ,
  CLOCK_FREQUENCY_300MHZ,
  CLOCK_FREQUENCY_200MHZ,
  CLOCK_FREQUENCY_160MHZ,
  CLOCK_FREQUENCY_120MHZ,
  CLOCK_FREQUENCY_80MHZ,
  CLOCK_FREQUENCY_60MHZ,
  CLOCK_FREQUENCY_100MHZ,       /* Peripheral Clock values */
  CLOCK_FREQUENCY_50MHZ,
  CLOCK_FREQUENCY_20MHZ,
  CLOCK_FREQUENCY_10MHZ,
  CLOCK_FREQUENCY_76_8_RC_MHZ,  /* RC and XO clocks */
  CLOCK_FREQUENCY_38_4_RC_MHZ,
  CLOCK_FREQUENCY_76_8_XO_MHZ,
  CLOCK_FREQUENCY_38_4_XO_MHZ,
  CLOCK_FREQUENCY_DISABLED,
} clock_frequency_t;

// Oscillator clock selectors
typedef enum
{
  OSCILLATOR_SOURCE_RC,    // use RC as oscillator clock
  OSCILLATOR_SOURCE_XTAL   // use XTAL  as oscillator clock
} oscillator_source_t;

typedef enum
{
  OSCILLATOR_TARGET_SYS_CLOCKS,    // various system clocks
  OSCILLATOR_TARGET_PERIPH_CLOCKS, // clock for peripherrals
  OSCILLATOR_TARGET_S32K_CLOCK     // 32K low frequency clock
} oscillator_target_t;

// PLL clock selectors
typedef enum
{
  PLL_SOURCE_PLL,  // use the PLL clocks
  PLL_SOURCE_OSC   // use the OCS clocks (can be RC or XTAL)
} pll_source_t;

typedef enum
{
  PLL_TARGET_SYSREFCLK,
  PLL_TARGET_SYSCLK,
  PLL_TARGET_ES0,
  PLL_TARGET_ES1
} pll_target_t;

typedef enum
{
  CLKEN_SYSPLL,
  CLKEN_CPUPLL,
  CLKEN_ES0,
  CLKEN_ES1,
  CLKEN_HFXO_OUT,
  CLKEN_CLK_160M,
  CLKEN_CLK_100M,
  CLKEN_USB,
  CLKEN_HFOSC
} clock_enable_t;

typedef enum
{
  A32_CLOCK_GATE = 0,
  A32_REFCLK = 1,
  A32_SYSPLL = 2,
  A32_CPUPLL = 4
} a32_source_t;

typedef enum
{
  ACLK_CLOCK_GATE = 0,
  ACLK_REFCLK = 1,
  ACLK_SYSPLL = 2
} aclk_source_t;

typedef enum
{
  DIVIDER_CPUPLL,
  DIVIDER_SYSPLL,
  DIVIDER_ACLK,
  DIVIDER_HCLK,
  DIVIDER_PCLK
} clock_divider_t;

/*******************************************************************************
 *  G L O B A L   D E F I N E S
 ******************************************************************************/

/*******************************************************************************
 *  F U N C T I O N   P R O T O T Y P E S
 ******************************************************************************/

// Services infrastructure APIs
uint32_t SERVICES_register_channel(uint32_t mhu_id,
                                   uint32_t channel_number);
void SERVICES_unregister_channel(uint32_t mhu_id,
                                 uint32_t channel_number);

uintptr_t SERVICES_prepare_packet_buffer(uint32_t size);

typedef void (*SERVICES_sender_callback) (uint32_t sender_id, uint32_t data);

uint32_t SERVICES_send_msg(uint32_t services_handle, uint32_t services_data);
uint32_t SERVICES_send_request(uint32_t services_handle,
                               uint16_t service_id,
                               SERVICES_sender_callback callback);

const char *SERVICES_version(void);
char *SERVICES_error_to_string(uint32_t error_code);


// Services functional APIs
uint32_t SERVICES_heartbeat(uint32_t services_handle);
uint32_t SERVICES_heartbeat_async(uint32_t services_handle, 
                                  SERVICES_sender_callback callback);
uint32_t SERVICES_uart_write(uint32_t services_handle, size_t size,
                             const uint8_t *uart_data);
uint32_t SERVICES_pinmux(uint32_t services_handle, 
                         uint8_t port_number, 
                         uint8_t pin_number, 
                         uint8_t config_data, 
                         uint32_t * error_code);
uint32_t SERVICES_padcontrol(uint32_t services_handle, 
                             uint8_t port_number, 
                             uint8_t pin_number,
                             uint8_t configuration_value, 
                             uint32_t * error_code);
uint32_t SERVICES_application_ospi_write_key(uint32_t services_handle,
                                             uint32_t command,
                                             uint8_t * key,
                                             uint32_t * error_code);
uint32_t SERVICES_cryptocell_get_rnd(uint32_t services_handle, 
                                     uint16_t rnd_len,
                                     void *rnd_value,
                                     int32_t * error_code);

uint32_t SERVICES_cryptocell_get_lcs(uint32_t services_handle, 
                                     uint32_t *lcs_state,
                                     int32_t * error_code);

// MbedTLS macros and APIs
uint32_t SERVICES_cryptocell_mbedtls_hardware_poll(uint32_t services_handle,
                                                   uint32_t * error_code,
                                                   uint32_t data,
                                                   uint32_t output,
                                                   uint32_t len,
                                                   uint32_t olen);

uint32_t SERVICES_cryptocell_mbedtls_aes_init(uint32_t services_handle, 
                                              uint32_t * error_code, 
                                              uint32_t ctx);
uint32_t SERVICES_cryptocell_mbedtls_aes_set_key(uint32_t services_handle, 
                                                 uint32_t * error_code, 
                                                 uint32_t ctx, 
                                                 uint32_t key, 
                                                 uint32_t keybits,
                                                 uint32_t dir);
uint32_t SERVICES_cryptocell_mbedtls_aes_crypt(uint32_t services_handle, 
                                               uint32_t * error_code, 
                                               uint32_t ctx, 
                                               uint32_t crypt_type, 
                                               uint32_t mode, 
                                               uint32_t length, 
                                               uint32_t iv, 
                                               uint32_t input, 
                                               uint32_t output);

uint32_t SERVICES_cryptocell_mbedtls_sha_starts(uint32_t services_handle, 
                                                uint32_t * error_code, 
                                                uint32_t ctx, 
                                                uint32_t sha_type);
uint32_t SERVICES_cryptocell_mbedtls_sha_process(uint32_t services_handle, 
                                                uint32_t * error_code, 
                                                uint32_t ctx, 
                                                uint32_t sha_type, 
                                                uint32_t data);
uint32_t SERVICES_cryptocell_mbedtls_sha_update(uint32_t services_handle, 
                                                uint32_t * error_code, 
                                                uint32_t ctx, 
                                                uint32_t sha_type, 
                                                uint32_t data, 
                                                uint32_t data_length);
uint32_t SERVICES_cryptocell_mbedtls_sha_finish(uint32_t services_handle, 
                                                uint32_t * error_code, 
                                                uint32_t ctx, 
                                                uint32_t sha_type, 
                                                uint32_t data);

uint32_t SERVICES_cryptocell_mbedtls_ccm_gcm_set_key(uint32_t services_handle,
    uint32_t * error_code,
    uint32_t context_addr,
    uint32_t key_type,
    uint32_t cipher,
    uint32_t key_addr,
    uint32_t key_bits);
uint32_t SERVICES_cryptocell_mbedtls_ccm_gcm_crypt(uint32_t services_handle,
    uint32_t * error_code,
    uint32_t context_addr,
    uint32_t crypt_type,
    uint32_t length,
    uint32_t iv_addr,
    uint32_t iv_length,
    uint32_t add_addr,
    uint32_t add_length,
    uint32_t input_addr,
    uint32_t output_addr,
    uint32_t tag_addr,
    uint32_t tag_length);
uint32_t SERVICES_cryptocell_mbedtls_chacha20_crypt(uint32_t services_handle,
    uint32_t * error_code,
    uint32_t key_addr,
    uint32_t nonce_addr,
    uint32_t counter,
    uint32_t data_len,
    uint32_t input_addr,
    uint32_t output_addr);
uint32_t SERVICES_cryptocell_mbedtls_chachapoly_crypt(uint32_t services_handle,
    uint32_t * error_code,
    uint32_t context_addr,
    uint32_t crypt_type,
    uint32_t length,
    uint32_t nonce_addr,
    uint32_t aad_addr,
    uint32_t aad_len,
    uint32_t tag_addr,
    uint32_t input_addr,
    uint32_t output_addr);
uint32_t SERVICES_cryptocell_mbedtls_poly1305_crypt(uint32_t services_handle,
    uint32_t * error_code,
    uint32_t key_addr,
    uint32_t input_addr,
    uint32_t ilen,
    uint32_t mac_addr);
uint32_t SERVICES_cryptocell_mbedtls_cmac_init_setkey(uint32_t services_handle,
    uint32_t * error_code,
    uint32_t context_addr,
    uint32_t key_addr,
    uint32_t key_bits);
uint32_t SERVICES_cryptocell_mbedtls_cmac_update(uint32_t services_handle,
    uint32_t * error_code,
    uint32_t context_addr,
    uint32_t input_addr,
    uint32_t input_length);
uint32_t SERVICES_cryptocell_mbedtls_cmac_finish(uint32_t services_handle,
    uint32_t * error_code,
    uint32_t context_addr,
    uint32_t output_addr);
uint32_t SERVICES_cryptocell_mbedtls_cmac_reset(uint32_t services_handle,
    uint32_t * error_code,
    uint32_t context_addr);

uint32_t SERVICES_system_get_toc_version(uint32_t services_handle, 
                                         uint32_t * toc_version,
                                         uint32_t * error_code);
uint32_t SERVICES_system_get_toc_number(uint32_t services_handle, 
                                        uint32_t *toc_number,
                                        uint32_t * error_code);
uint32_t SERVICES_system_get_toc_via_name(uint32_t services_handle, 
                                          const uint8_t *cpu_name,
                                          uint32_t * error_code);
uint32_t SERVICES_system_get_toc_via_cpuid(uint32_t services_handle,
                                           SERVICES_cpuid_t cpuid,
                                           SERVICES_toc_data_t * toc_info,
                                           uint32_t * error_code);
uint32_t SERVICES_system_get_toc_data     (uint32_t services_handle,
                                           SERVICES_toc_data_t *toc_data,
                                           uint32_t * error_code);
uint32_t SERVICES_system_get_device_part_number(uint32_t services_handle, 
                                                uint32_t *device_part_number,
                                                uint32_t * error_code);
uint32_t SERVICES_system_set_services_debug (uint32_t services_handle,
                                             bool debug_enable,
                                             uint32_t *error_code);
uint32_t SERVICES_get_se_revision(uint32_t services_handle,
                                  uint8_t *revision_data, uint32_t *error_code);
uint32_t SERVICES_system_get_otp_data (uint32_t services_handle,
                                       SERVICES_otp_data_t *toc_info,
                                       uint32_t * error_code);
uint32_t SERVICES_system_read_otp     (uint32_t services_handle,
                                       uint32_t otp_offset,
                                       uint32_t *otp_value_word,
                                       uint32_t *error_code);

uint32_t SERVICES_boot_process_toc_entry(uint32_t services_handle, 
                                         const uint8_t * image_id,
                                         uint32_t * error_code);
uint32_t SERVICES_boot_cpu(uint32_t services_handle, 
                           uint32_t cpu_id,
                           uint32_t address,
                           uint32_t * error_code);
uint32_t SERVICES_boot_release_cpu(uint32_t services_handle,
                                   uint32_t cpu_id,
                                   uint32_t * error_code);
uint32_t SERVICES_boot_reset_cpu(uint32_t services_handle,
                                 uint32_t cpu_id,
                                 uint32_t * error_code);
uint32_t SERVICES_boot_reset_soc(uint32_t services_handle);

uint32_t SERVICES_power_stop_mode_req(uint32_t services_handle,
                                      services_power_profile_t power_profile,
                                      bool override);
uint32_t SERVICES_power_ewic_config(uint32_t services_handle,
                                    uint32_t ewic_source,
                                    services_power_profile_t power_profile);
uint32_t SERVICES_power_wakeup_config(uint32_t services_handle,
                                      uint32_t vbat_wakeup_source,
                                      services_power_profile_t power_profile);
uint32_t
SERVICES_power_mem_retention_config(uint32_t services_handle,
                                    uint32_t mem_retention,
                                    services_power_profile_t power_profile);
uint32_t
SERVICES_corstone_standby_mode(uint32_t services_handle,
                               host_cpu_clus_pwr_req_t host_cpu_clus_pwr_req,
                               bsys_pwr_req_t bsys_pwr_req,
                               uint32_t *error_code);
uint32_t
SERVICES_power_m55_he_vtor_save(uint32_t services_handle,
                                uint32_t ns_vtor_addr,
                                uint32_t se_vtor_addr,
                                services_power_profile_t power_profile);
uint32_t
SERVICES_power_m55_hp_vtor_save(uint32_t services_handle,
                                uint32_t ns_vtor_addr,
                                uint32_t se_vtor_addr,
                                services_power_profile_t power_profile);


// Clocks services
uint32_t SERVICES_clocks_select_osc_source(uint32_t services_handle,
                                           oscillator_source_t source,
                                           oscillator_target_t target,
                                           uint32_t * error_code);

uint32_t SERVICES_clocks_select_pll_source(uint32_t services_handle,
                                           pll_source_t source,
                                           pll_target_t target,
                                           uint32_t * error_code);

uint32_t SERVICES_clocks_enable_clock(uint32_t services_handle,
                                      clock_enable_t clock,
                                      bool enable,
                                      uint32_t * error_code);

uint32_t SERVICES_clocks_set_ES0_frequency(uint32_t services_handle,
                                           clock_frequency_t frequency,
                                           uint32_t * error_code);

uint32_t SERVICES_clocks_set_ES1_frequency(uint32_t services_handle,
                                           clock_frequency_t frequency,
                                           uint32_t * error_code);

uint32_t SERVICES_clocks_select_a32_source(uint32_t services_handle,
                                           a32_source_t source,
                                           uint32_t * error_code);

uint32_t SERVICES_clocks_select_aclk_source(uint32_t services_handle,
                                            aclk_source_t source,
                                            uint32_t * error_code);

uint32_t SERVICES_clocks_set_divider(uint32_t services_handle,
                                     clock_divider_t divider,
                                     uint32_t value,
                                     uint32_t * error_code);

#ifdef __cplusplus
}
#endif
#endif /* __SERVICES_LIB_API_H__ */
