/* Copyright (C) 2023 Alif Semiconductor - All Rights Reserved.
 * Use, distribution and modification of this code is permitted under the
 * terms stated in the Alif Semiconductor Software License Agreement
 *
 * You should have received a copy of the Alif Semiconductor Software
 * License Agreement with this file. If not, please write to:
 * contact@alifsemi.com, or visit: https://alifsemi.com/license
 *
 */

/**************************************************************************//**
 * @file     Camera_Sensor.h
 * @author   Tanay Rami and Chandra Bhushan Singh
 * @email    tanay@alifsemi.com and chandrabhushan.singh@alifsemi.com
 * @version  V1.1.0
 *             -Removed enums for clock source, interface, polarity, hsync mode,
 *             data mode, and data mask.
 *             -Included low level header file 'cpi.h'
 *             -Replaced data types in CAMERA_SENSOR_INFO structure with CPI low
 *             level file data types.
 * @date     19-April-2023
 * @brief    Camera Sensor Device definitions.
 ******************************************************************************/

#ifndef CAMERA_SENSOR_H_
#define CAMERA_SENSOR_H_

#ifdef  __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include "Camera_Common.h"
#include "cpi.h"

/** \brief CAMERA Sensor Device Information */
typedef struct _CAMERA_SENSOR_INFO {
  uint8_t output_format;                                          /**< Camera Sensor Image Output Format Mono/RAW-Bayer/RGB/YUV */
  uint8_t additional_width;                                       /**< if Camera Sensor requires additional width  (ex: in case of border pixel for demosaic.) */
  uint8_t additional_height;                                      /**< if Camera Sensor requires additional height (ex: in case of border pixel for demosaic.) */
} CAMERA_SENSOR_INFO;

/** \brief CAMERA Sensor Device Configurations */
typedef struct _CAMERA_SENSOR_CONFIG {
  CPI_INTERFACE            interface;                             /**< Camera Sensor Interface             */
  CPI_WAIT_VSYNC           vsync_wait;                            /**< Camera Sensor VSYNC Wait            */
  CPI_CAPTURE_DATA_ENABLE  vsync_mode;                            /**< Camera Sensor VSYNC Mode            */
  CPI_SIG_POLARITY         pixelclk_pol;                          /**< Camera Sensor Pixel Clock Polarity  */
  CPI_SIG_POLARITY         hsync_pol;                             /**< Camera Sensor HSYNC Polarity        */
  CPI_SIG_POLARITY         vsync_pol;                             /**< Camera Sensor VSYNC Polarity        */
  CPI_DATA_MODE            data_mode;                             /**< Camera Sensor Data Mode             */
  CPI_DATA_FIELD           data_field;                            /**< Select MSB/LSB                      */
  CPI_CODE10ON8_CODING     code10on8;                             /**< code10on8 enable/disable            */
  CPI_DATA_MASK            data_mask;                             /**< Camera Sensor Data Mask             */
} CAMERA_SENSOR_CONFIG;

/** \brief CAMERA Sensor Device Operations */
typedef struct _CAMERA_SENSOR_OPERATIONS {
  int32_t (*Init)    (ARM_CAMERA_RESOLUTION camera_resolution);   /**< Initialize Camera Sensor device     */
  int32_t (*Uninit)  (void);                                      /**< De-initialize Camera Sensor device  */
  int32_t (*Start)   (void);                                      /**< Start Camera Sensor device          */
  int32_t (*Stop)    (void);                                      /**< Stop Camera Sensor device           */
  int32_t (*Control) (uint32_t control, uint32_t arg);            /**< Control Camera Sensor device        */
} CAMERA_SENSOR_OPERATIONS;

/** \brief CAMERA Sensor Device */
typedef struct _CAMERA_SENSOR_DEVICE {
  CAMERA_SENSOR_INFO       *Info;                                 /**< Camera Sensor device Information    */
  CAMERA_SENSOR_CONFIG     *Config;                               /**< Camera Sensor device Configurations */
  CAMERA_SENSOR_OPERATIONS *Ops;                                  /**< Camera Sensor device Operations     */
} CAMERA_SENSOR_DEVICE;

#ifdef  __cplusplus
}
#endif

#endif /* CAMERA_SENSOR_H_ */

/************************ (C) COPYRIGHT ALIF SEMICONDUCTOR *****END OF FILE****/
