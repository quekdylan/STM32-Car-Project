#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "stm32f4xx_hal.h"

// Initialize IMU: detects 0x68/0x69 and configures the sensor. Gyro bias
// calibration is deferred until imu_calibrate_bias_blocking() is called.
// Writes detected address to out_addrSel if non-NULL.
void imu_init(I2C_HandleTypeDef *hi2c, uint8_t *out_addrSel);

// Blocking gyro bias calibration (~2 s of samples). Call when the robot is
// stationary; yaw is zeroed after calibration so future deltas are relative to
// the calibrated heading.
void imu_calibrate_bias_blocking(int sample_count);

// Call from a 100 Hz scheduler; internally decimates gyro updates to 50 Hz.
void imu_update_yaw_100Hz(void);

// Get current yaw in degrees.
float imu_get_yaw(void);
// OLD:
// void Gyro_Read_Z(I2C_HandleTypeDef *hi2c, uint8_t *readGyroZData, float *gyroZ);

// NEW:
void Gyro_Read_Z(I2C_HandleTypeDef *hi2c, float *gyroZ_dps);

// Zero current yaw to 0 degrees (helper)
void imu_zero_yaw(void);

// Reset magnetometer calibration (min/max) accumulators.
void imu_mag_reset_calibration(void);

// Read IMU internal temperature in Celsius; returns 0 on success.
int imu_read_temperature_c(float *temp_c);

// Read magnetometer heading (degrees 0-359) using raw mag X/Y; returns 0 on success, non-zero on failure.
int imu_read_mag_heading_deg(float *heading_deg);

// Get current gyro bias in deg/s
float imu_get_gyro_bias(void);

// Manually set gyro bias (useful for known drift measurements)
void imu_set_gyro_bias(float bias_dps);

// Gyro scale factor functions
float imu_get_scale_factor(void);
void imu_set_scale_factor(float scale);

#endif /* INC_IMU_H_ */
