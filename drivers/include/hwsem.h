/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

#ifndef HWSEM_H_
#define HWSEM_H_

#ifdef  __cplusplus
extern "C"
{
#endif

#include <stdint.h>

typedef struct _HWSEM_INSTANCE_Type {
    volatile uint32_t       HWSEM_REQ_REG;              /*!< (@ 0x00000000) Request register                                             */
    volatile uint32_t       HWSEM_REL_REG;              /*!< (@ 0x00000004) Release register                                             */
    volatile uint32_t       HWSEM_RST_REG;              /*!< (@ 0x00000008) Reset register                                               */
    const volatile uint32_t RESERVED;
} HWSEM_INSTANCE_Type;

typedef struct {
    HWSEM_INSTANCE_Type     module[16];
} HWSEM_Type;


/**
  \fn          static inline uint32_t hwsem_request(HWSEM_Type *hwsem, uint8_t instance, uint32_t master_id)
  \brief       Request ownership of a hwsem instance.
  \param[in]   hwsem     Pointer to the HWSEM register map
  \param[in]   instance  The hwsem instance
  \param[in]   master_id The master id to be used to request the ownership
  \return      Current master id. Will be same as the parameter master_id if the request was successful.
*/
static inline uint32_t hwsem_request(HWSEM_Type *hwsem, uint8_t instance, uint32_t master_id)
{
    HWSEM_INSTANCE_Type *hwsem_instance = &hwsem->module[instance];

    /* Write the master_id to the request register */
    hwsem_instance->HWSEM_REQ_REG = master_id;

    /* Read back and return the value */
    return hwsem_instance->HWSEM_REQ_REG;
}

/**
  \fn          static inline uint32_t hwsem_getcount(HWSEM_Type *hwsem, uint8_t instance)
  \brief       Get the count for a hwsem instance.
  \param[in]   hwsem     Pointer to the HWSEM register map
  \param[in]   instance  The hwsem instance
  \return      HWSEM count
*/
static inline uint32_t hwsem_getcount(HWSEM_Type *hwsem, uint8_t instance)
{
    HWSEM_INSTANCE_Type *hwsem_instance = &hwsem->module[instance];
    return hwsem_instance->HWSEM_REL_REG;
}

/**
  \fn          static inline void hwsem_release(HWSEM_Type *hwsem, uint8_t instance, uint32_t master_id)
  \brief       Release ownership of a hwsem instance.
  \param[in]   hwsem     Pointer to the HWSEM register map
  \param[in]   instance  The hwsem instance
  \param[in]   master_id The master id to be used to release ownership
  \return      none
*/
static inline void hwsem_release(HWSEM_Type *hwsem, uint8_t instance, uint32_t master_id)
{
    HWSEM_INSTANCE_Type *hwsem_instance = &hwsem->module[instance];
    hwsem_instance->HWSEM_REL_REG = master_id;
}

/**
  \fn          static void hwsem_reset(HWSEM_Type *hwsem, uint8_t instance)
  \brief       Reset a hwsem instance.
  \param[in]   hwsem     Pointer to the HWSEM register map
  \param[in]   instance  The hwsem instance
  \return      none
*/
static void hwsem_reset(HWSEM_Type *hwsem, uint8_t instance)
{
    HWSEM_INSTANCE_Type *hwsem_instance = &hwsem->module[instance];
    hwsem_instance->HWSEM_RST_REG = 0x1U;
}

#ifdef __cplusplus
}
#endif
#endif /* HWSEM_H_ */
