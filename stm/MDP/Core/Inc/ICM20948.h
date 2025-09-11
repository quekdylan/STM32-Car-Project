/**
  ******************************************************************************
  * @file           : ICM20948.h
  * @brief          : ICM20948 Gyroscope and Accelerometer Driver Header
  * @author         : Your Name
  * @date           : 2025
  ******************************************************************************
  */

#ifndef ICM20948_H
#define ICM20948_H

#include "ICM20948_ADDR.h"
#include "ICM20948_OPTIONS.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// SENSITIVITY SCALE FACTORS
// ============================================================================

// Gyroscope sensitivity scale factors (LSB per degree per second)
#define GRYO_SENSITIVITY_SCALE_FACTOR_250DPS  131.0f
#define GRYO_SENSITIVITY_SCALE_FACTOR_500DPS  65.5f
#define GRYO_SENSITIVITY_SCALE_FACTOR_1000DPS 32.8f
#define GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS 16.4f

// Accelerometer sensitivity scale factors (LSB per g)
#define ACCEL_SENSITIVITY_SCALE_FACTOR_2G  16384.0f
#define ACCEL_SENSITIVITY_SCALE_FACTOR_4G  8192.0f
#define ACCEL_SENSITIVITY_SCALE_FACTOR_8G  4096.0f
#define ACCEL_SENSITIVITY_SCALE_FACTOR_16G 2048.0f

// ============================================================================
// FUNCTION PROTOTYPES
// ============================================================================

/**
 * @brief Check if ICM20948 is available at I2C address 1 (0x68)
 * @param hi2c I2C handle
 * @retval 1 if device found, 0 if not found
 */
uint8_t ICM20948_isI2cAddress1(I2C_HandleTypeDef *hi2c);

/**
 * @brief Check if ICM20948 is available at I2C address 2 (0x69)
 * @param hi2c I2C handle
 * @retval 1 if device found, 0 if not found
 */
uint8_t ICM20948_isI2cAddress2(I2C_HandleTypeDef *hi2c);

/**
 * @brief Initialize ICM20948 sensor
 * @param hi2c I2C handle
 * @param selectI2cAddress I2C address selector (0 or 1)
 * @param selectGyroSensitivity Gyroscope sensitivity setting
 * @retval None
 */
void ICM20948_init(I2C_HandleTypeDef *hi2c, uint8_t selectI2cAddress, uint8_t selectGyroSensitivity);

/**
 * @brief Read all gyroscope axes
 * @param hi2c I2C handle
 * @param selectI2cAddress I2C address selector (0 or 1)
 * @param selectGyroSensitivity Gyroscope sensitivity setting
 * @param readings Array to store X, Y, Z readings (in degrees per second)
 * @retval None
 */
void ICM20948_readGyroscope_allAxises(I2C_HandleTypeDef *hi2c, uint8_t selectI2cAddress, uint8_t selectGyroSensitivity, int16_t readings[3]);

/**
 * @brief Read only Z-axis gyroscope data
 * @param hi2c I2C handle
 * @param selectI2cAddress I2C address selector (0 or 1)
 * @param selectGyroSensitivity Gyroscope sensitivity setting
 * @param gyroZ Pointer to store Z-axis reading (in degrees per second)
 * @retval None
 */
void ICM20948_readGyroscope_Z(I2C_HandleTypeDef *hi2c, uint8_t selectI2cAddress, uint8_t selectGyroSensitivity, int16_t *gyroZ);

/**
 * @brief Read all accelerometer axes
 * @param hi2c I2C handle
 * @param selectI2cAddress I2C address selector (0 or 1)
 * @param selectAccelSensitivity Accelerometer sensitivity setting
 * @param readings Array to store X, Y, Z readings (in g)
 * @retval None
 */
void ICM20948_readAccelerometer_allAxises(I2C_HandleTypeDef *hi2c, uint8_t selectI2cAddress, uint8_t selectAccelSensitivity, int16_t readings[3]);

/**
 * @brief Read temperature sensor
 * @param hi2c I2C handle
 * @param selectI2cAddress I2C address selector (0 or 1)
 * @param reading Pointer to store temperature reading (raw value)
 * @retval None
 */
void ICM20948_readTemperature(I2C_HandleTypeDef *hi2c, uint8_t selectI2cAddress, int16_t *reading);

#ifdef __cplusplus
}
#endif

#endif /* ICM20948_H */
