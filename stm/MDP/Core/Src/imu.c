#include "imu.h"
#include "ICM20948.h"
#include <math.h>
#include <float.h>

// =======================
// Module state
// =======================
static I2C_HandleTypeDef *s_imu_i2c = NULL;
static uint8_t  s_addrSel     = 0;     // 0 => 0x68, 1 => 0x69
static float    s_yaw_deg     = 0.0f;  // integrated heading (deg, wrapped to [-180,180])
static float    s_gyro_bias_z = 0.0f;  // bias in deg/s (for Z axis)
static uint8_t  s_mag_ready   = 0;     // magnetometer enabled flag

typedef struct {
    float min_x;
    float max_x;
    float min_y;
    float max_y;
    uint8_t have_range;
} mag_calib_t;

static mag_calib_t s_mag_cal;

static void mag_cal_reset_(void){
    s_mag_cal.min_x = FLT_MAX;
    s_mag_cal.max_x = -FLT_MAX;
    s_mag_cal.min_y = FLT_MAX;
    s_mag_cal.max_y = -FLT_MAX;
    s_mag_cal.have_range = 0;
}

// 100 Hz update period
#define IMU_DT_S 0.01f

// Small deadband to squash tiny noise (in deg/s) when robot is still
#define GZ_DEADBAND_DPS  0.4f

// Wrap to [-180, 180]
static inline float wrap180(float a){
    if (a >  180.0f) a -= 360.0f;
    if (a < -180.0f) a += 360.0f;
    return a;
}

// -----------------------
// Bias calibration @ 2000 dps
// -----------------------
static void imu_calibrate_bias_(void){
    // ~1 second of samples at ~500 * 2ms = ~1000ms
    const int N = 500;
    float sum_dps = 0.0f;

    for (int i = 0; i < N; i++){
        int16_t gz_raw = 0;
        // Read raw Z at 2000 dps scale
        ICM20948_readGyroscope_Z(s_imu_i2c, s_addrSel, GYRO_FULL_SCALE_2000DPS, &gz_raw);

        // Convert raw -> deg/s (use constant for 2000 dps range)
        const float LSB_PER_DPS = (float)GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS; // ~16.4 LSB/(deg/s)
        float gz_dps = ((float)gz_raw) / LSB_PER_DPS;

        sum_dps += gz_dps;
        HAL_Delay(2);
    }

    s_gyro_bias_z = sum_dps / (float)N; // average bias (deg/s)
}

// =======================
// Public API
// =======================

void imu_init(I2C_HandleTypeDef *hi2c, uint8_t *out_addrSel){
    s_imu_i2c = hi2c;

    if (ICM20948_isI2cAddress1(hi2c))      s_addrSel = 0;
    else if (ICM20948_isI2cAddress2(hi2c)) s_addrSel = 1;
    else                                   Error_Handler();

    if (out_addrSel) *out_addrSel = s_addrSel;

    // Configure IMU with 2000 dps full-scale
    ICM20948_init(hi2c, s_addrSel, GYRO_FULL_SCALE_2000DPS);

    // Keep the robot still for ~1s during bias calibration
    imu_calibrate_bias_();

    // Start from zero heading
    s_yaw_deg = 0.0f;

    // Enable magnetometer (non-fatal if fails)
    if (ICM20948_mag_enable(hi2c, s_addrSel) == 0) {
        s_mag_ready = 1;
        mag_cal_reset_();
    } else {
        s_mag_ready = 0;
    }
}

float imu_get_yaw(void){
    return s_yaw_deg;
}

// Optional helper: zero current yaw (e.g., when you want "forward" to be current heading)
void imu_zero_yaw(void){
    s_yaw_deg = 0.0f;
}

void imu_mag_reset_calibration(void){
    mag_cal_reset_();
}

int imu_read_temperature_c(float *temp_c){
    if (!s_imu_i2c || !temp_c) return -1;
    int16_t raw = 0;
    ICM20948_readTemperature(s_imu_i2c, s_addrSel, &raw);
    // Datasheet: Temp (°C) = (raw / 333.87) + 21
    *temp_c = ((float)raw / 333.87f) + 21.0f;
    return 0;
}

int imu_read_mag_heading_deg(float *heading_deg){
    if (!s_mag_ready || !heading_deg) return -1;
    int16_t mx=0,my=0,mz=0;
    if (ICM20948_mag_read_raw(s_imu_i2c, s_addrSel, &mx, &my, &mz) != 0) return -2;
    // Simple heading from X/Y (adjust sign/orientation as needed). atan2(y, x)
    float hx = (float)mx;
    float hy = (float)my;

    if (hx < s_mag_cal.min_x) s_mag_cal.min_x = hx;
    if (hx > s_mag_cal.max_x) s_mag_cal.max_x = hx;
    if (hy < s_mag_cal.min_y) s_mag_cal.min_y = hy;
    if (hy > s_mag_cal.max_y) s_mag_cal.max_y = hy;

    float span_x = s_mag_cal.max_x - s_mag_cal.min_x;
    float span_y = s_mag_cal.max_y - s_mag_cal.min_y;

    if (span_x > 50.0f && span_y > 50.0f) {
        s_mag_cal.have_range = 1;
    }

    if (s_mag_cal.have_range) {
        float bias_x = 0.5f * (s_mag_cal.max_x + s_mag_cal.min_x);
        float bias_y = 0.5f * (s_mag_cal.max_y + s_mag_cal.min_y);
        hx -= bias_x;
        hy -= bias_y;

        float avg_span = 0.5f * (span_x + span_y);
        if (span_x > 1e-3f) hx *= avg_span / span_x;
        if (span_y > 1e-3f) hy *= avg_span / span_y;
    }

    float rad = atan2f(hy, hx); // -pi .. pi
    float deg = rad * (180.0f / 3.14159265358979323846f);
    if (deg < 0) deg += 360.0f;
    *heading_deg = deg;
    return 0;
}

// Read instantaneous Z gyro rate in deg/s (bias-removed, with small deadband)
void Gyro_Read_Z(I2C_HandleTypeDef *hi2c, float *gyroZ_dps){
    int16_t gz_raw = 0;

    // Read raw Z at 2000 dps scale
    ICM20948_readGyroscope_Z(hi2c, s_addrSel, GYRO_FULL_SCALE_2000DPS, &gz_raw);

    // Convert raw -> deg/s
    const float LSB_PER_DPS = (float)GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS; // ~16.4
    float gz_dps = ((float)gz_raw) / LSB_PER_DPS;

    // Remove bias
    gz_dps -= s_gyro_bias_z;

    // Deadband
    if (gz_dps > -GZ_DEADBAND_DPS && gz_dps < GZ_DEADBAND_DPS) {
        gz_dps = 0.0f;
    }

    *gyroZ_dps = gz_dps;
}

// Call this at 100 Hz (every 10 ms): integrates Z rate into yaw (deg), wraps to [-180,180]
void imu_update_yaw_100Hz(void){
    int16_t gz_raw = 0;

    // Read raw Z at 2000 dps scale
    ICM20948_readGyroscope_Z(s_imu_i2c, s_addrSel, GYRO_FULL_SCALE_2000DPS, &gz_raw);

    // Convert raw -> deg/s
    const float LSB_PER_DPS = (float)GRYO_SENSITIVITY_SCALE_FACTOR_2000DPS; // ~16.4
    float yaw_rate_dps = ((float)gz_raw) / LSB_PER_DPS;

    // Remove bias
    yaw_rate_dps -= s_gyro_bias_z;

    // Deadband
    if (yaw_rate_dps > -GZ_DEADBAND_DPS && yaw_rate_dps < GZ_DEADBAND_DPS) {
        yaw_rate_dps = 0.0f;
    }

    // Integrate: yaw[k+1] = yaw[k] + rate * dt
    s_yaw_deg = wrap180(s_yaw_deg + yaw_rate_dps * IMU_DT_S);
}
