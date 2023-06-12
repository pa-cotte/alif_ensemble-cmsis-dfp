/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

#include "crc.h"

/**
 @fn           crc_swap(uint32_t input_value)
 @brief        Swap the input data
 @param[in]    input_value   : CRC 32 bit algorithm input data
 @return       result of swapped input data
 */
static uint32_t crc_swap(uint32_t input_value)
{
    uint32_t res = (((input_value & 0xFF) << 24)      |
                   ((input_value & 0xFF00) << 8)      |
                   ((input_value & 0xFF0000) >> 8)    |
                   ((input_value & 0xFF000000) >>24));

    return res;
}

/**
 @fn           crc_bit_reflect(uint32_t input)
 @brief        Reflect the CRC 32 bit output
 @param[in]    input    : 32 bit CRC output
 @return       result of reflected CRC 32 bit output
 */
uint32_t crc_bit_reflect(uint32_t input)
{
    uint32_t res = 0;
    uint32_t i, bit;

    for(i = 0; i < 32; i++)
    {
        bit = (input >> i) & 1;
        bit = bit << (32 - (i + 1));
        res |= bit;
    }

    return res;
}

/**
@fn         uint32_t CRC_calculate_Unaligned(uint32_t key, uint8_t *input,
                                             uint8_t length, uint32_t poly)
@brief      To calculate the CRC result for unaligned data
             1. It will take the aligned data for CRC result from the hardware.
             2. It will the Unaligned input data and its length.
             3. If the algorithm is 32 bit CRC then it will take the standard
                32 bit CRC Polynomial
             4. If the algorithm is 32 bit Custom CRC polynomial , it will take
                the polynomial entered from the user.
@param[in]  key   : Output of aligned data for CRC from the hardware
@param[in]  input : unaligned input data
@param[in]  length: length of unaligned data
@param[in]  poly  : Standard polynomial or the user entered polynomial depending upon the CRC algorithm
@return     Calculated CRC output for unaligned data
*/
uint32_t crc_calculate_unaligned(uint32_t key, const uint8_t *input,
                                 const uint8_t length, uint32_t poly)
{
    uint32_t crc, check_bit, polynomial;
    uint8_t data;
    int32_t i, j;

    crc = key;

    /* Store the reflected polynomial */
    polynomial = crc_bit_reflect(poly);

    for(i = 0; i < length; i++)
    {
        data = input[i];
        for(j = 0; j < 8; j++)
        {
            check_bit = (crc ^ data) & 1;
            crc >>= 1;

            if(check_bit)
            {
                crc = crc ^ polynomial;
            }
            data >>= 1;
        }
    }

    return ~crc;
}

/**
 @fn           crc_calculate_8bit(CRC_Type *crc, void *data_in,
                                  uint32_t len, uint32_t *data_out)
 @brief        Calculate the CRC output  for 8 bit CRC algorithm
 @param[in]    crc      : Pointer to the CRC register map
 @param[in]    data_in  : pointer which holds the address of CRC 8 bit input
 @param[in]    len      : Length of the input data
 @param[in]    data_out : 8 bit CRC output
 @return       None
 */
void crc_calculate_8bit(CRC_Type *crc, const void *data_in,
                        uint32_t len, uint32_t *data_out)
{
    for (uint32_t count = 0; count < len; count ++)
    {
        /* User input 8 bit data is storing into the DATA_IN_8 bit register */
        crc->CRC_DATA_IN_8_0 = ((uint8_t *)data_in)[count];
    }

    /* data_out pointer to store the CRC output */
    *data_out = (crc->CRC_OUT);
}

/**
 @fn           crc_calculate_16bit(CRC_Type *crc, void *data_in,
                                   uint32_t len, uint32_t *data_out)
 @brief        Calculate the CRC output  for 16 bit CRC algorithm
 @param[in]    crc      : Pointer to the CRC register map
 @param[in]    data_in  : pointer which holds the address of CRC 16 bit input
 @param[in]    len      : Length of the input data
 @param[in]    data_out : 16 bit CRC output
 @return       None
 */
 void crc_calculate_16bit(CRC_Type *crc, const void *data_in,
                          uint32_t len, uint32_t *data_out)
{
    for (uint32_t count = 0; count < len; count ++)
    {
        /* User input 8 bit data is storing into the DATA_IN_8 bit register */
        crc->CRC_DATA_IN_8_0 = ((uint8_t *)data_in)[count];
    }

    /* data_out pointer to store the CRC output */
    *data_out = (crc->CRC_OUT);
}

 /**
  @fn           crc_calculate_32bit(CRC_Type *crc, void *data_in,
                                    uint32_t len, uint32_t *data_out)
  @brief        Calculate the CRC output  for 32 bit CRC algorithm
  @param[in]    crc      : Pointer to the CRC register map
  @param[in]    data_in  : pointer which holds the address of CRC 32 bit input
  @param[in]    len      : Length of the input data
  @param[in]    data_out : 32 bit CRC output
  @return       None
  */
void crc_calculate_32bit(CRC_Type *crc, const void *data_in,
                         uint32_t len, uint32_t *data_out)
{
    uint32_t *data32;
    uint32_t value,reverse;
    uint32_t aligned_length;

    aligned_length   = len - (len % 4);
    data32           = (uint32_t *)data_in;

    for (uint32_t count = 0; count < aligned_length / 4; count++)
    {
        value = *(data32++);

        /* Swap the input data */
        value = crc_swap(value);

        /* User input 32 bit data is storing into the DATA_IN_32 bit register */
        crc->CRC_DATA_IN_32_0 = value;
    }

    /* Store the CRC aligned output */
    reverse = (crc->CRC_OUT);

    *data_out = ~reverse;
}
