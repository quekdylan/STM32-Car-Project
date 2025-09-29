/**
  ******************************************************************************
  * @file           : ICM20948.c
  * @brief          : ICM20948 Gyroscope and Accelerometer Driver
  * @author         : Your Name
  * @date           : 2025
  ******************************************************************************
  */

#include "ICM20948.h"

// ============================================================================
// PRIVATE DEFINES
// ============================================================================

#define X 0
#define Y 1
#define Z 2

#define X_HIGH_BYTE 0
#define X_LOW_BYTE 1
#define Y_HIGH_BYTE 2
#define Y_LOW_BYTE 3
#define Z_HIGH_BYTE 4
#define Z_LOW_BYTE 5

#define T_HIGH_BYTE 0
#define T_LOW_BYTE 1

// ICM20948 Control Constants
#define ICM20948_RESET 0x80
#define ICM20948_DISABLE_SENSORS 0x00
#define ICM20948_ENABLE_SENSORS 0x3F
#define ICM20948_AUTO_SELECT_CLOCK 0x01

// ============================================================================
// PRIVATE VARIABLES
// ============================================================================

static uint8_t readGyroDataZ[2];
static uint8_t mag_enabled = 0;

// ============================================================================
// PRIVATE FUNCTION PROTOTYPES
// ============================================================================

static HAL_StatusTypeDef _ICM20948_SelectUserBank(I2C_HandleTypeDef *hi2c, uint8_t selectI2cAddress, int userBankNum);
static HAL_StatusTypeDef _ICM20948_WriteByte(I2C_HandleTypeDef *hi2c, uint8_t selectI2cAddress, uint8_t registerAddress, uint8_t writeData);
static HAL_StatusTypeDef _ICM20948_ReadByte(I2C_HandleTypeDef *hi2c, uint8_t selectI2cAddress, uint8_t registerAddress, uint8_t *readData);
static HAL_StatusTypeDef _ICM20948_BurstRead(I2C_HandleTypeDef *hi2c, uint8_t selectI2cAddress, uint8_t startAddress, uint16_t amountOfRegistersToRead, uint8_t *readData);

// ============================================================================
// PRIVATE FUNCTIONS
// ============================================================================

/**
 * @brief Select user bank for ICM20948
 * @param hi2c I2C handle
 * @param selectI2cAddress I2C address selector (0 or 1)
 * @param userBankNum User bank number (0-3)
 * @retval HAL status
 */
static HAL_StatusTypeDef _ICM20948_SelectUserBank(I2C_HandleTypeDef *hi2c, uint8_t selectI2cAddress, int userBankNum) {
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t writeData = userBankNum << 4; // Shift to bits 4-7
    uint8_t deviceI2CAddress = (selectI2cAddress == 0) ? ICM20948__I2C_SLAVE_ADDRESS_1 : ICM20948__I2C_SLAVE_ADDRESS_2;

    status = HAL_I2C_Mem_Write(
        hi2c,
        deviceI2CAddress << 1,
        ICM20948__USER_BANK_ALL__REG_BANK_SEL__REGISTER,
        I2C_MEMADD_SIZE_8BIT,
        &writeData,
        I2C_MEMADD_SIZE_8BIT,
        10);

    return status;
}

/**
 * @brief Write a single byte to ICM20948 register
 * @param hi2c I2C handle
 * @param selectI2cAddress I2C address selector (0 or 1)
 * @param registerAddress Register address to write to
 * @param writeData Data to write
 * @retval HAL status
 */
static HAL_StatusTypeDef _ICM20948_WriteByte(I2C_HandleTypeDef *hi2c, uint8_t selectI2cAddress, uint8_t registerAddress, uint8_t writeData) {
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t deviceI2CAddress = (selectI2cAddress == 0) ? ICM20948__I2C_SLAVE_ADDRESS_1 : ICM20948__I2C_SLAVE_ADDRESS_2;

    status = HAL_I2C_Mem_Write(
        hi2c,
        deviceI2CAddress << 1,
        registerAddress,
        I2C_MEMADD_SIZE_8BIT,
        &writeData,
        I2C_MEMADD_SIZE_8BIT,
        10);

    return status;
}

/**
 * @brief Read a single byte from ICM20948 register
 * @param hi2c I2C handle
 * @param selectI2cAddress I2C address selector (0 or 1)
 * @param registerAddress Register address to read from
 * @param readData Pointer to store read data
 * @retval HAL status
 */
static HAL_StatusTypeDef _ICM20948_ReadByte(I2C_HandleTypeDef *hi2c, uint8_t selectI2cAddress, uint8_t registerAddress, uint8_t *readData) {
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t deviceI2CAddress = (selectI2cAddress == 0) ? ICM20948__I2C_SLAVE_ADDRESS_1 : ICM20948__I2C_SLAVE_ADDRESS_2;

    status = HAL_I2C_Mem_Read(
        hi2c,
        deviceI2CAddress << 1,
        registerAddress,
        I2C_MEMADD_SIZE_8BIT,
        readData,
        I2C_MEMADD_SIZE_8BIT,
        10);

    return status;
}

/**
 * @brief Burst read multiple bytes from ICM20948
 * @param hi2c I2C handle
 * @param selectI2cAddress I2C address selector (0 or 1)
 * @param startAddress Starting register address
 * @param amountOfRegistersToRead Number of registers to read
 * @param readData Buffer to store read data
 * @retval HAL status
 */
static HAL_StatusTypeDef _ICM20948_BurstRead(I2C_HandleTypeDef *hi2c, uint8_t selectI2cAddress, uint8_t startAddress, uint16_t amountOfRegistersToRead, uint8_t *readData) {
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t deviceI2CAddress = (selectI2cAddress == 0) ? ICM20948__I2C_SLAVE_ADDRESS_1 : ICM20948__I2C_SLAVE_ADDRESS_2;

    status = HAL_I2C_Mem_Read(
        hi2c,
        deviceI2CAddress << 1,
        startAddress,
        I2C_MEMADD_SIZE_8BIT,
        readData,
        amountOfRegistersToRead,
        10);

    return status;
}

// ============================================================================
// PUBLIC FUNCTIONS
// ============================================================================

/**
 * @brief Check if ICM20948 is available at I2C address 1 (0x68)
 * @param hi2c I2C handle
 * @retval 1 if device found, 0 if not found
 */
uint8_t ICM20948_isI2cAddress1(I2C_HandleTypeDef *hi2c) {
    HAL_StatusTypeDef addressStatus = HAL_I2C_IsDeviceReady(hi2c, ICM20948__I2C_SLAVE_ADDRESS_1 << 1, 2, 10);
    return (addressStatus == HAL_OK) ? 1 : 0;
}

/**
 * @brief Check if ICM20948 is available at I2C address 2 (0x69)
 * @param hi2c I2C handle
 * @retval 1 if device found, 0 if not found
 */
uint8_t ICM20948_isI2cAddress2(I2C_HandleTypeDef *hi2c) {
    HAL_StatusTypeDef addressStatus = HAL_I2C_IsDeviceReady(hi2c, ICM20948__I2C_SLAVE_ADDRESS_2 << 1, 2, 10);
    return (addressStatus == HAL_OK) ? 1 : 0;
}

/**
 * @brief Initialize ICM20948 sensor
 * @param hi2c I2C handle
 * @param selectI2cAddress I2C address selector (0 or 1)
 * @param selectGyroSensitivity Gyroscope sensitivity setting
 * @retval None
 */
void ICM20948_init(I2C_HandleTypeDef *hi2c, uint8_t selectI2cAddress, uint8_t selectGyroSensitivity) {
    HAL_StatusTypeDef status = HAL_OK;

    // Select User Bank 0
    status = _ICM20948_SelectUserBank(hi2c, selectI2cAddress, USER_BANK_0);

    // Reset the device
    status = _ICM20948_WriteByte(
        hi2c,
        selectI2cAddress,
        ICM20948__USER_BANK_0__PWR_MGMT_1__REGISTER,
        ICM20948_RESET);

    HAL_Delay(200); // Wait for reset to complete

    // Set clock source to auto-select
    status = _ICM20948_WriteByte(
        hi2c,
        selectI2cAddress,
        ICM20948__USER_BANK_0__PWR_MGMT_1__REGISTER,
        ICM20948_AUTO_SELECT_CLOCK);

    // Configure power management 2 (enable gyro and accel)
    status = _ICM20948_WriteByte(
        hi2c,
        selectI2cAddress,
        ICM20948__USER_BANK_0__PWR_MGMT_2__REGISTER,
        0x00); // Enable all sensors

    // Select User Bank 2 for gyro configuration
    status = _ICM20948_SelectUserBank(hi2c, selectI2cAddress, USER_BANK_2);

    // Configure gyroscope
    // Note: selectGyroSensitivity constants are already pre-shifted to bits [2:1].
    // Do NOT shift again here.
    status = _ICM20948_WriteByte(
        hi2c,
        selectI2cAddress,
        ICM20948__USER_BANK_2__GYRO_CONFIG_1__REGISTER,
        (3 << GYRO_DLPFCFG_BIT) | (selectGyroSensitivity) | (EN_GRYO_DLPF << GYRO_FCHOICE_BIT));

    // Configure accelerometer (enable and set sensitivity)
    status = _ICM20948_WriteByte(
        hi2c,
        selectI2cAddress,
        ICM20948__USER_BANK_2__ACCEL_CONFIG__REGISTER,
        3 << BIT_3 | ACCEL_FULL_SCALE_2G << BIT_1 | 0x01 << BIT_0);

    // Set sample rate divider
    status = _ICM20948_WriteByte(
        hi2c,
        selectI2cAddress,
        ICM20948__USER_BANK_2__GYRO_SMPLRT_DIV__REGISTER,
        0x16); // Sample rate ≈ 1.125 kHz / (1 + 0x16) ≈ 51 Hz

    // Return to User Bank 0
    status = _ICM20948_SelectUserBank(hi2c, selectI2cAddress, USER_BANK_0);

    // Disable internal I2C master to allow direct bypass access to AK09916
    status = _ICM20948_WriteByte(
        hi2c,
        selectI2cAddress,
        ICM20948__USER_BANK_0__USER_CTRL__REGISTER,
        0x00);

    // Configure interrupt pin
    status = _ICM20948_WriteByte(
        hi2c,
        selectI2cAddress,
        ICM20948__USER_BANK_0__INT_PIN_CFG__REGISTER,
        0x02);
}

/**
 * @brief Read all gyroscope axes
 * @param hi2c I2C handle
 * @param selectI2cAddress I2C address selector (0 or 1)
 * @param selectGyroSensitivity Gyroscope sensitivity setting
 * @param readings Array to store X, Y, Z readings
 * @retval None
 */
void ICM20948_readGyroscope_allAxises(I2C_HandleTypeDef *hi2c, uint8_t selectI2cAddress, uint8_t selectGyroSensitivity, int16_t readings[3]) {
    uint8_t readData[6];

    // Read 6 bytes starting from GYRO_XOUT_H
    _ICM20948_BurstRead(hi2c, selectI2cAddress, ICM20948__USER_BANK_0__GYRO_XOUT_H__REGISTER, 6, readData);

    // Combine high and low bytes
    readings[X] = (readData[X_HIGH_BYTE] << 8) | readData[X_LOW_BYTE];
    readings[Y] = (readData[Y_HIGH_BYTE] << 8) | readData[Y_LOW_BYTE];
    readings[Z] = (readData[Z_HIGH_BYTE] << 8) | readData[Z_LOW_BYTE];

    // Apply sensitivity scaling
    switch (selectGyroSensitivity) {
        case GYRO_FULL_SCALE_250DPS:
            readings[X] /= GRYO_SENSITIVITY_SCALE_FACTOR_250DPS;
            readings[Y] /= GRYO_SENSITIVITY_SCALE_FACTOR_250DPS;
            readings[Z] /= GRYO_SENSITIVITY_SCALE_FACTOR_250DPS;
            break;
        case GYRO_FULL_SCALE_500DPS:
            readings[X] /= GRYO_SENSITIVITY_SCALE_FACTOR_500DPS;
            readings[Y] /= GRYO_SENSITIVITY_SCALE_FACTOR_500DPS;
            readings[Z] /= GRYO_SENSITIVITY_SCALE_FACTOR_500DPS;
            break;
        case GYRO_FULL_SCALE_1000DPS:
            readings[X] /= GRYO_SENSITIVITY_SCALE_FACTOR_1000DPS;
            readings[Y] /= GRYO_SENSITIVITY_SCALE_FACTOR_1000DPS;
            readings[Z] /= GRYO_SENSITIVITY_SCALE_FACTOR_1000DPS;
            break;
        case GYRO_FULL_SCALE_2000DPS:
            readings[X] /= GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS;
            readings[Y] /= GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS;
            readings[Z] /= GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS;
            break;
    }
}

/**
 * @brief Read only Z-axis gyroscope data
 * @param hi2c I2C handle
 * @param selectI2cAddress I2C address selector (0 or 1)
 * @param selectGyroSensitivity Gyroscope sensitivity setting
 * @param gyroZ Pointer to store Z-axis reading
 * @retval None
 */
 void ICM20948_readGyroscope_Z(I2C_HandleTypeDef * hi2c,
    uint8_t const selectI2cAddress,
    uint8_t const selectGyroSensitivity,
    int16_t *gyroZ)
{
// Read 2 bytes starting from GYRO_ZOUT_H
_ICM20948_BurstRead(hi2c, selectI2cAddress,
ICM20948__USER_BANK_0__GYRO_ZOUT_H__REGISTER,
2, readGyroDataZ);

// Return RAW counts (signed 16-bit). Scale in float at the IMU layer.
*gyroZ = (int16_t)((readGyroDataZ[0] << 8) | readGyroDataZ[1]);
}

/**
 * @brief Read all accelerometer axes
 * @param hi2c I2C handle
 * @param selectI2cAddress I2C address selector (0 or 1)
 * @param selectAccelSensitivity Accelerometer sensitivity setting
 * @param readings Array to store X, Y, Z readings
 * @retval None
 */
void ICM20948_readAccelerometer_allAxises(I2C_HandleTypeDef *hi2c, uint8_t selectI2cAddress, uint8_t selectAccelSensitivity, int16_t readings[3]) {
    uint8_t readData[6];

    // Read 6 bytes starting from ACCEL_XOUT_H
    _ICM20948_BurstRead(hi2c, selectI2cAddress, ICM20948__USER_BANK_0__ACCEL_XOUT_H__REGISTER, 6, readData);

    // Combine high and low bytes
    readings[X] = (readData[X_HIGH_BYTE] << 8) | readData[X_LOW_BYTE];
    readings[Y] = (readData[Y_HIGH_BYTE] << 8) | readData[Y_LOW_BYTE];
    readings[Z] = (readData[Z_HIGH_BYTE] << 8) | readData[Z_LOW_BYTE];

    // Apply sensitivity scaling
    switch (selectAccelSensitivity) {
        case ACCEL_FULL_SCALE_2G:
            readings[X] /= ACCEL_SENSITIVITY_SCALE_FACTOR_2G;
            readings[Y] /= ACCEL_SENSITIVITY_SCALE_FACTOR_2G;
            readings[Z] /= ACCEL_SENSITIVITY_SCALE_FACTOR_2G;
            break;
        case ACCEL_FULL_SCALE_4G:
            readings[X] /= ACCEL_SENSITIVITY_SCALE_FACTOR_4G;
            readings[Y] /= ACCEL_SENSITIVITY_SCALE_FACTOR_4G;
            readings[Z] /= ACCEL_SENSITIVITY_SCALE_FACTOR_4G;
            break;
        case ACCEL_FULL_SCALE_8G:
            readings[X] /= ACCEL_SENSITIVITY_SCALE_FACTOR_8G;
            readings[Y] /= ACCEL_SENSITIVITY_SCALE_FACTOR_8G;
            readings[Z] /= ACCEL_SENSITIVITY_SCALE_FACTOR_8G;
            break;
        case ACCEL_FULL_SCALE_16G:
            readings[X] /= ACCEL_SENSITIVITY_SCALE_FACTOR_16G;
            readings[Y] /= ACCEL_SENSITIVITY_SCALE_FACTOR_16G;
            readings[Z] /= ACCEL_SENSITIVITY_SCALE_FACTOR_16G;
            break;
    }
}

/**
 * @brief Read temperature sensor
 * @param hi2c I2C handle
 * @param selectI2cAddress I2C address selector (0 or 1)
 * @param reading Pointer to store temperature reading
 * @retval None
 */
void ICM20948_readTemperature(I2C_HandleTypeDef *hi2c, uint8_t selectI2cAddress, int16_t *reading) {
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t readData[2];

    // Ensure we're in User Bank 0
    status = _ICM20948_SelectUserBank(hi2c, selectI2cAddress, USER_BANK_0);

    // Read 2 bytes starting from TEMP_OUT_H
    status = _ICM20948_BurstRead(hi2c, selectI2cAddress, ICM20948__USER_BANK_0__TEMP_OUT_H__REGISTER, 2, readData);

    // Combine high and low bytes
    *reading = (readData[T_HIGH_BYTE] << 8) | readData[T_LOW_BYTE];
}

// ================= Magnetometer (AK09916) =================
// AK09916 is accessed via ICM20948 I2C master pass-through.
// For simplicity we put it into continuous measurement mode 2 (100Hz) and read raw XYZ.

#define AK09916_I2C_ADDR          0x0C
#define AK09916_REG_WIA1          0x00
#define AK09916_REG_WIA2          0x01
#define AK09916_REG_STATUS1       0x10
#define AK09916_REG_HXL           0x11
#define AK09916_REG_STATUS2       0x17
#define AK09916_REG_CNTL2         0x31
#define AK09916_REG_CNTL3         0x32
#define AK09916_CMD_SOFT_RESET    0x01
#define AK09916_MODE_POWER_DOWN   0x00
#define AK09916_MODE_SINGLE       0x01
#define AK09916_CONT_MODE_2       0x08  /* Continuous measurement mode 2 (100 Hz) */

// Helper: enable pass-through (BYPASS) so MCU can talk directly to AK09916.
static int icm_enable_bypass_(I2C_HandleTypeDef *hi2c, uint8_t sel){
    // Select bank 0
    if (_ICM20948_SelectUserBank(hi2c, sel, USER_BANK_0) != HAL_OK) return -1;
    // Ensure internal I2C master is disabled so host can talk directly to AK09916
    if (_ICM20948_WriteByte(hi2c, sel, ICM20948__USER_BANK_0__USER_CTRL__REGISTER, 0x00) != HAL_OK) return -1;
    HAL_Delay(1);
    // INT_PIN_CFG: set BYPASS_EN bit (bit 1)
    uint8_t cfg = 0x02; // BYPASS_EN=1, others default
    if (_ICM20948_WriteByte(hi2c, sel, ICM20948__USER_BANK_0__INT_PIN_CFG__REGISTER, cfg) != HAL_OK) return -1;
    return 0;
}

int ICM20948_mag_enable(I2C_HandleTypeDef *hi2c, uint8_t selectI2cAddress){
    if (mag_enabled) return 0;
    if (icm_enable_bypass_(hi2c, selectI2cAddress) != 0) return -1;
    HAL_Delay(5);
    // Check WHO_AM_I (WIA1/WIA2) optionally (not enforced here)
    uint8_t wia1=0,wia2=0;
    HAL_I2C_Mem_Read(hi2c, AK09916_I2C_ADDR<<1, AK09916_REG_WIA1, I2C_MEMADD_SIZE_8BIT, &wia1, 1, 20);
    HAL_I2C_Mem_Read(hi2c, AK09916_I2C_ADDR<<1, AK09916_REG_WIA2, I2C_MEMADD_SIZE_8BIT, &wia2, 1, 20);
    // Soft reset to ensure a clean start
    uint8_t cmd = AK09916_CMD_SOFT_RESET;
    if (HAL_I2C_Mem_Write(hi2c, AK09916_I2C_ADDR<<1, AK09916_REG_CNTL3, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 20) != HAL_OK) return -1;
    HAL_Delay(1);

    // Leave device in power-down; we'll request single measurements on demand.
    cmd = AK09916_MODE_POWER_DOWN;
    if (HAL_I2C_Mem_Write(hi2c, AK09916_I2C_ADDR<<1, AK09916_REG_CNTL2, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 20) != HAL_OK) return -1;
    HAL_Delay(2);
    mag_enabled = 1;
    return 0;
}

int ICM20948_mag_read_raw(I2C_HandleTypeDef *hi2c, uint8_t selectI2cAddress, int16_t *mx, int16_t *my, int16_t *mz){
    (void)selectI2cAddress; // bypass mode direct addressing
    if (!mag_enabled) return -1;
    uint8_t cmd = AK09916_MODE_SINGLE;
    if (HAL_I2C_Mem_Write(hi2c, AK09916_I2C_ADDR<<1, AK09916_REG_CNTL2, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 20) != HAL_OK) return -1;

    // Wait for data ready (DRDY=1). Allow up to ~20 ms.
    uint32_t start = HAL_GetTick();
    uint8_t st1 = 0;
    do {
        if (HAL_I2C_Mem_Read(hi2c, AK09916_I2C_ADDR<<1, AK09916_REG_STATUS1, I2C_MEMADD_SIZE_8BIT, &st1, 1, 10) != HAL_OK) return -1;
        if (st1 & 0x01) break;
        HAL_Delay(1);
    } while ((HAL_GetTick() - start) < 20);
    if ((st1 & 0x01) == 0) return -2; // timeout waiting for new data

    uint8_t raw[6];
    if (HAL_I2C_Mem_Read(hi2c, AK09916_I2C_ADDR<<1, AK09916_REG_HXL, I2C_MEMADD_SIZE_8BIT, raw, 6, 20) != HAL_OK) return -1;
    // Reading ST2 clears the data-ready latch inside the AK09916 so the next sample can update.
    uint8_t st2=0;
    if (HAL_I2C_Mem_Read(hi2c, AK09916_I2C_ADDR<<1, AK09916_REG_STATUS2, I2C_MEMADD_SIZE_8BIT, &st2, 1, 10) != HAL_OK) return -1;
    // Little-endian: L then H
    int16_t x = (int16_t)(raw[1] << 8 | raw[0]);
    int16_t y = (int16_t)(raw[3] << 8 | raw[2]);
    int16_t z = (int16_t)(raw[5] << 8 | raw[4]);
    if (mx) *mx = x;
    if (my) *my = y;
    if (mz) *mz = z;
    return 0;
}
