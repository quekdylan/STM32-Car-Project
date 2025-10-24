/**
  ******************************************************************************
  * @file           : ICM20948_OPTIONS.h
  * @brief          : ICM20948 Configuration Options and Bit Definitions
  * @author         : Your Name
  * @date           : 2025
  ******************************************************************************
  */

#ifndef ICM20948_OPTIONS_H
#define ICM20948_OPTIONS_H

// ============================================================================
// BIT POSITIONS
// ============================================================================

#define BIT_0 0
#define BIT_1 1
#define BIT_2 2
#define BIT_3 3
#define BIT_4 4
#define BIT_5 5
#define BIT_6 6
#define BIT_7 7

// ============================================================================
// GYROSCOPE CONFIGURATION OPTIONS
// ============================================================================

// Gyroscope Full Scale Range
#define GYRO_FULL_SCALE_250DPS  0x00
#define GYRO_FULL_SCALE_500DPS  0x02
#define GYRO_FULL_SCALE_1000DPS 0x04
#define GYRO_FULL_SCALE_2000DPS 0x06

// Gyroscope DLPF (Digital Low Pass Filter) Configuration
#define GYRO_DLPFCFG_BIT 3
#define GYRO_FCHOICE_BIT 0

// Gyroscope DLPF Enable
#define EN_GRYO_DLPF 0x00
#define DIS_GRYO_DLPF 0x01

// ============================================================================
// ACCELEROMETER CONFIGURATION OPTIONS
// ============================================================================

// Accelerometer Full Scale Range
#define ACCEL_FULL_SCALE_2G  0x00
#define ACCEL_FULL_SCALE_4G  0x02
#define ACCEL_FULL_SCALE_8G  0x04
#define ACCEL_FULL_SCALE_16G 0x06

// Accelerometer DLPF Configuration
#define ACCEL_DLPFCFG_BIT 3
#define ACCEL_FCHOICE_BIT 0

// Accelerometer DLPF Enable
#define EN_ACCEL_DLPF 0x00
#define DIS_ACCEL_DLPF 0x01

// ============================================================================
// POWER MANAGEMENT OPTIONS
// ============================================================================

// Clock Source Selection
#define CLK_INTERNAL_20MHZ_OSC 0x00
#define CLK_AUTO_SELECT        0x01
#define CLK_STOP               0x07

// Sleep Mode
#define SLEEP_DISABLE 0x00
#define SLEEP_ENABLE  0x40

// Cycle Mode
#define CYCLE_DISABLE 0x00
#define CYCLE_ENABLE  0x20

// ============================================================================
// INTERRUPT CONFIGURATION
// ============================================================================

// Interrupt Pin Configuration
#define INT_PIN_ACTIVE_HIGH 0x00
#define INT_PIN_ACTIVE_LOW  0x80
#define INT_PIN_OPEN_DRAIN  0x40
#define INT_PIN_PUSH_PULL   0x00

// Interrupt Latch
#define INT_LATCH_DISABLE 0x00
#define INT_LATCH_ENABLE  0x20

// Interrupt Clear
#define INT_CLEAR_ANY_READ 0x00
#define INT_CLEAR_STATUS   0x10

// ============================================================================
// FIFO CONFIGURATION
// ============================================================================

// FIFO Mode
#define FIFO_MODE_NORMAL 0x00
#define FIFO_MODE_STREAM 0x40

// FIFO Reset
#define FIFO_RESET_DISABLE 0x00
#define FIFO_RESET_ENABLE  0x01

// ============================================================================
// I2C MASTER CONFIGURATION
// ============================================================================

// I2C Master Clock
#define I2C_MST_CLK_348KHZ 0x07
#define I2C_MST_CLK_333KHZ 0x0F
#define I2C_MST_CLK_320KHZ 0x17
#define I2C_MST_CLK_308KHZ 0x1F
#define I2C_MST_CLK_296KHZ 0x27
#define I2C_MST_CLK_286KHZ 0x2F
#define I2C_MST_CLK_276KHZ 0x37
#define I2C_MST_CLK_267KHZ 0x3F
#define I2C_MST_CLK_258KHZ 0x47
#define I2C_MST_CLK_500KHZ 0x0B
#define I2C_MST_CLK_471KHZ 0x13
#define I2C_MST_CLK_444KHZ 0x1B
#define I2C_MST_CLK_421KHZ 0x23
#define I2C_MST_CLK_400KHZ 0x2B
#define I2C_MST_CLK_381KHZ 0x33
#define I2C_MST_CLK_364KHZ 0x3B
// #define I2C_MST_CLK_348KHZ 0x43  // Duplicate definition removed

// I2C Master Enable
#define I2C_MST_ENABLE  0x20
#define I2C_MST_DISABLE 0x00

// ============================================================================
// SAMPLE RATE DIVIDERS
// ============================================================================

// Gyroscope Sample Rate Divider (1-255)
// Sample Rate = 1.125 kHz / (1 + GYRO_SMPLRT_DIV)
#define GYRO_SMPLRT_DIV_MIN 0x00
#define GYRO_SMPLRT_DIV_MAX 0xFF

// Accelerometer Sample Rate Divider (1-4095)
// Sample Rate = 1.125 kHz / (1 + ACCEL_SMPLRT_DIV)
#define ACCEL_SMPLRT_DIV_MIN 0x000
#define ACCEL_SMPLRT_DIV_MAX 0xFFF

#endif /* ICM20948_OPTIONS_H */
