#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "stm32f4xx_hal.h"

// Initialize IMU: detects 0x68/0x69, configures sensor, calibrates gyro bias.
// Writes detected address to out_addrSel if non-NULL.
void imu_init(I2C_HandleTypeDef *hi2c, uint8_t *out_addrSel);

// Call at 100 Hz: reads gyro Z and integrates yaw (degrees, [-180,180]).
void imu_update_yaw_100Hz(void);

// Get current yaw in degrees.
float imu_get_yaw(void);
// OLD:
// void Gyro_Read_Z(I2C_HandleTypeDef *hi2c, uint8_t *readGyroZData, float *gyroZ);

// NEW:
void Gyro_Read_Z(I2C_HandleTypeDef *hi2c, float *gyroZ_dps);

// Zero current yaw to 0 degrees (helper)
void imu_zero_yaw(void);

#endif /* INC_IMU_H_ */
