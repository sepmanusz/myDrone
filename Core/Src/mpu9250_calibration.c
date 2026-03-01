/*
 * mpu9250_calibration.c
 * MPU9250 calibration routines: gyro, accel, magnetometer
 */

#include "mpu9250_calibration.h"
#include <math.h>
#include "uart_printf.h"
#include "main.h"

/* Global calibration data */
MPU9250_Calibration_t mpu9250_cal = {
    .gyro_offset_x = 0.0f,
    .gyro_offset_y = 0.0f,
    .gyro_offset_z = 0.0f,
    .accel_offset_x = 0.0f,
    .accel_offset_y = 0.0f,
    .accel_offset_z = 0.0f,
    .mag_offset_x = 0.0f,
    .mag_offset_y = 0.0f,
    .mag_offset_z = 0.0f,
    .mag_scale_x = 1.0f,
    .mag_scale_y = 1.0f,
    .mag_scale_z = 1.0f,
    .gyro_calibrated = 0,
    .accel_calibrated = 0,
    .mag_calibrated = 0
};

/*
 * Wait for USER button press and release to proceed with calibration steps.
 * Placed at file scope so it can be reused by multiple calibration routines.
 */
static void wait_for_user_button(void)
{
    uart_printf("Press USER button to continue...\r\n");
    while (HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin) == GPIO_PIN_RESET) {
        HAL_Delay(10);
    }
    HAL_Delay(50);
    while (HAL_GPIO_ReadPin(USER_Btn_GPIO_Port, USER_Btn_Pin) == GPIO_PIN_SET) {
        HAL_Delay(10);
    }
    HAL_Delay(50);
}

/**
 * @brief Calibrate gyroscope by averaging stationary readings
 * Samples accelerations while device is stationary and averages them as offset
 */
HAL_StatusTypeDef MPU9250_CalibrateGyro(SPI_HandleTypeDef *hspi, uint16_t samples)
{
    if (samples == 0) samples = 100;
    
    uart_printf("Gyro calibration: Keep device STATIONARY for %u samples...\r\n", samples);
    HAL_Delay(2000);  /* Wait for user to position device */
    
    double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
    MPU9250_Data data;

    
    for (uint16_t i = 0; i < samples; i++) {
        if (MPU9250_ReadData(hspi, &data) != HAL_OK) {
            uart_printf("Gyro cal: read error at sample %u\r\n", i);
            return HAL_ERROR;
        }
        sum_x += data.gyroX;
        sum_y += data.gyroY;
        sum_z += data.gyroZ;
        HAL_Delay(10);
        
        if ((i + 1) % 20 == 0) {
            uart_printf("  Gyro cal: %u/%u samples\r\n", i + 1, samples);
        }
    }
    
    /* Store average as offset */
    mpu9250_cal.gyro_offset_x = (float)(sum_x / samples);
    mpu9250_cal.gyro_offset_y = (float)(sum_y / samples);
    mpu9250_cal.gyro_offset_z = (float)(sum_z / samples);
    mpu9250_cal.gyro_calibrated = 1;
    
    uart_printf("Gyro calibration complete:\r\n");
    uart_printf("  Offset X: %d (x1e-4 dps)\r\n", (int)(mpu9250_cal.gyro_offset_x * 10000));
    uart_printf("  Offset Y: %d (x1e-4 dps)\r\n", (int)(mpu9250_cal.gyro_offset_y * 10000));
    uart_printf("  Offset Z: %d (x1e-4 dps)\r\n", (int)(mpu9250_cal.gyro_offset_z * 10000));
    
    return HAL_OK;
}

/**
 * @brief Calibrate accelerometer in known orientations
 * Collects readings in +/-Z (1g, -1g) and averages to compute offsets
 */
HAL_StatusTypeDef MPU9250_CalibrateAccel(SPI_HandleTypeDef *hspi, uint16_t samples_per_axis)
{
    if (samples_per_axis == 0) samples_per_axis = 50;
    
    uart_printf("Accel calibration: Place device flat, +Z UP (gravity). Press button or wait 5s...\r\n");
    HAL_Delay(5000);
    
    double sum_x_up = 0.0, sum_y_up = 0.0, sum_z_up = 0.0;
    MPU9250_Data data;
    
    /* Collect +Z (gravity up) */
    for (uint16_t i = 0; i < samples_per_axis; i++) {
        if (MPU9250_ReadData(hspi, &data) != HAL_OK) {
            return HAL_ERROR;
        }
        sum_x_up += data.accelX;
        sum_y_up += data.accelY;
        sum_z_up += data.accelZ;
        HAL_Delay(10);
    }
    
    float accel_x_up = (float)(sum_x_up / samples_per_axis);
    float accel_y_up = (float)(sum_y_up / samples_per_axis);
    float accel_z_up = (float)(sum_z_up / samples_per_axis);
    
    /* Reset for -Z reading */
    uart_printf("Accel cal: Flip device upside down (-Z UP). Wait 5s...\r\n");
    HAL_Delay(5000);
    
    double sum_x_down = 0.0, sum_y_down = 0.0, sum_z_down = 0.0;
    
    for (uint16_t i = 0; i < samples_per_axis; i++) {
        if (MPU9250_ReadData(hspi, &data) != HAL_OK) {
            return HAL_ERROR;
        }
        sum_x_down += data.accelX;
        sum_y_down += data.accelY;
        sum_z_down += data.accelZ;
        HAL_Delay(10);
    }
    
    float accel_x_down = (float)(sum_x_down / samples_per_axis);
    float accel_y_down = (float)(sum_y_down / samples_per_axis);
    float accel_z_down = (float)(sum_z_down / samples_per_axis);
    
    /* Compute offsets: midpoint between +Z and -Z readings for all axes */
    /* Offset = (reading_up + reading_down) / 2  */
    mpu9250_cal.accel_offset_x = (accel_x_up + accel_x_down) / 2.0f;  /* Should be ~0 */
    mpu9250_cal.accel_offset_y = (accel_y_up + accel_y_down) / 2.0f;  /* Should be ~0 */
    mpu9250_cal.accel_offset_z = (accel_z_up + accel_z_down) / 2.0f - 1.0f;  /* Center around 1g */
    mpu9250_cal.accel_calibrated = 1;
    
    uart_printf("Accel calibration complete:\r\n");
    uart_printf("  Offset X: %d (x1e-4 g)\r\n", (int)(mpu9250_cal.accel_offset_x * 10000));
    uart_printf("  Offset Y: %d (x1e-4 g)\r\n", (int)(mpu9250_cal.accel_offset_y * 10000));
    uart_printf("  Offset Z: %d (x1e-4 g)\r\n", (int)(mpu9250_cal.accel_offset_z * 10000));
    
    return HAL_OK;
}

/**
 * @brief Calibrate magnetometer by collecting readings during rotation (figure-8 motion)
 * Finds hard-iron offsets by computing min/max in each axis
 */
HAL_StatusTypeDef MPU9250_CalibrateMag(SPI_HandleTypeDef *hspi, uint16_t duration_ms)
{
    if (duration_ms == 0) duration_ms = 60000;  /* Default 60 seconds for spherical rotation */
    
    uart_printf("Mag calibration: Rotate device in ALL directions for 60 seconds.\r\n");
    uart_printf("Move slowly in XY, YZ, XZ planes (all directions). Press button to start...\r\n");
    wait_for_user_button();
    
    MPU9250_Data data;
    float mag_min_x = 1e6, mag_max_x = -1e6;
    float mag_min_y = 1e6, mag_max_y = -1e6;
    float mag_min_z = 1e6, mag_max_z = -1e6;
    
    uint32_t start_time = HAL_GetTick();
    uint32_t sample_count = 0;
    
    /* Collect magnetometer data while rotating */
    while ((HAL_GetTick() - start_time) < duration_ms) {
        if (MPU9250_ReadData(hspi, &data) != HAL_OK) {
            return HAL_ERROR;
        }
        
        /* Track min/max for each axis */
        if (data.magX < mag_min_x) mag_min_x = data.magX;
        if (data.magX > mag_max_x) mag_max_x = data.magX;
        if (data.magY < mag_min_y) mag_min_y = data.magY;
        if (data.magY > mag_max_y) mag_max_y = data.magY;
        if (data.magZ < mag_min_z) mag_min_z = data.magZ;
        if (data.magZ > mag_max_z) mag_max_z = data.magZ;
        
        sample_count++;
        HAL_Delay(10);
        
        if (sample_count % 100 == 0) {
            uint32_t elapsed = (HAL_GetTick() - start_time);
            uart_printf("  Mag cal: %lu ms / %u ms\r\n", elapsed, duration_ms);
        }
    }
    
    /* Compute hard-iron offsets (center between min and max) */
    mpu9250_cal.mag_offset_x = (mag_max_x + mag_min_x) / 2.0f;
    mpu9250_cal.mag_offset_y = (mag_max_y + mag_min_y) / 2.0f;
    mpu9250_cal.mag_offset_z = (mag_max_z + mag_min_z) / 2.0f;
    
    /* Compute soft-iron scale factors */
    float avg_range = ((mag_max_x - mag_min_x) + (mag_max_y - mag_min_y) + (mag_max_z - mag_min_z)) / 3.0f;
    mpu9250_cal.mag_scale_x = avg_range / (mag_max_x - mag_min_x);
    mpu9250_cal.mag_scale_y = avg_range / (mag_max_y - mag_min_y);
    mpu9250_cal.mag_scale_z = avg_range / (mag_max_z - mag_min_z);
    
    mpu9250_cal.mag_calibrated = 1;
    
    uart_printf("Mag calibration complete (%lu samples):\r\n", sample_count);
    uart_printf("  Hard-iron offset X: %d (x0.01 uT)\r\n", (int)(mpu9250_cal.mag_offset_x * 100));
    uart_printf("  Hard-iron offset Y: %d (x0.01 uT)\r\n", (int)(mpu9250_cal.mag_offset_y * 100));
    uart_printf("  Hard-iron offset Z: %d (x0.01 uT)\r\n", (int)(mpu9250_cal.mag_offset_z * 100));
    uart_printf("  Soft-iron scale X: %u (x1e-4)\r\n", (unsigned)(mpu9250_cal.mag_scale_x * 10000));
    uart_printf("  Soft-iron scale Y: %u (x1e-4)\r\n", (unsigned)(mpu9250_cal.mag_scale_y * 10000));
    uart_printf("  Soft-iron scale Z: %u (x1e-4)\r\n", (unsigned)(mpu9250_cal.mag_scale_z * 10000));
    
    return HAL_OK;
}

/**
 * @brief Perform full calibration (gyro, accel, mag in sequence)
 */
HAL_StatusTypeDef MPU9250_CalibrateAll(SPI_HandleTypeDef *hspi)
{
    uart_printf("\n========== MPU9250 FULL CALIBRATION START ==========\r\n");
    
    if (MPU9250_CalibrateGyro(hspi, 200) != HAL_OK) {
        uart_printf("ERROR: Gyro calibration failed\r\n");
        return HAL_ERROR;
    }
    HAL_Delay(1000);
    
    if (MPU9250_CalibrateAccel(hspi, 100) != HAL_OK) {
        uart_printf("ERROR: Accel calibration failed\r\n");
        return HAL_ERROR;
    }
    HAL_Delay(1000);
    
    if (MPU9250_CalibrateMag(hspi, 60000) != HAL_OK) {  /* 60 second spherical rotation */
        uart_printf("ERROR: Mag calibration failed\r\n");
        return HAL_ERROR;
    }
    
    uart_printf("========== MPU9250 FULL CALIBRATION COMPLETE ==========\r\n\n");
    return HAL_OK;
}

/**
 * @brief Apply calibration offsets to sensor data
 */
void MPU9250_ApplyCalibration(MPU9250_Data *data)
{
    if (data == NULL) return;
    
    /* Apply gyro offset */
    if (mpu9250_cal.gyro_calibrated) {
        data->gyroX -= mpu9250_cal.gyro_offset_x;
        data->gyroY -= mpu9250_cal.gyro_offset_y;
        data->gyroZ -= mpu9250_cal.gyro_offset_z;
    }
    
    /* Apply accel offset */
    if (mpu9250_cal.accel_calibrated) {
        data->accelX -= mpu9250_cal.accel_offset_x;
        data->accelY -= mpu9250_cal.accel_offset_y;
        data->accelZ -= mpu9250_cal.accel_offset_z;
    }
    
    /* Apply mag hard-iron and soft-iron correction */
    if (mpu9250_cal.mag_calibrated) {
        float mx = data->magX - mpu9250_cal.mag_offset_x;
        float my = data->magY - mpu9250_cal.mag_offset_y;
        float mz = data->magZ - mpu9250_cal.mag_offset_z;
        
        data->magX = mx * mpu9250_cal.mag_scale_x;
        data->magY = my * mpu9250_cal.mag_scale_y;
        data->magZ = mz * mpu9250_cal.mag_scale_z;
    }
}

/**
 * @brief Reset calibration to defaults
 */
void MPU9250_CalibrationReset(void)
{
    mpu9250_cal.gyro_offset_x = 0.0f;
    mpu9250_cal.gyro_offset_y = 0.0f;
    mpu9250_cal.gyro_offset_z = 0.0f;
    mpu9250_cal.accel_offset_x = 0.0f;
    mpu9250_cal.accel_offset_y = 0.0f;
    mpu9250_cal.accel_offset_z = 0.0f;
    mpu9250_cal.mag_offset_x = 0.0f;
    mpu9250_cal.mag_offset_y = 0.0f;
    mpu9250_cal.mag_offset_z = 0.0f;
    mpu9250_cal.mag_scale_x = 1.0f;
    mpu9250_cal.mag_scale_y = 1.0f;
    mpu9250_cal.mag_scale_z = 1.0f;
    mpu9250_cal.gyro_calibrated = 0;
    mpu9250_cal.accel_calibrated = 0;
    mpu9250_cal.mag_calibrated = 0;
    
    uart_printf("Calibration reset to defaults\r\n");
}

/**
 * @brief Compute 2D heading from magnetometer only (horizontal plane)
 * Good when sensor is horizontal; fails if tilted
 */
float ComputeHeading2D(float mag_x, float mag_y, float declination)
{
    /* Heading from atan2: angle between mag vector and north (Y-axis) */
    float heading = atan2f(mag_x, mag_y) * 57.29577951308232f;  /* rad to deg */
    heading += declination;
    if (heading < 0.0f) heading += 360.0f;
    if (heading >= 360.0f) heading -= 360.0f;
    return heading;
}

/**
 * @brief Compute tilt-compensated 3D heading
 * Uses accel pitch/roll to project mag to horizontal plane
 * Better for tilted sensors; more complex calculation
 */
float ComputeHeading3D(float mag_x, float mag_y, float mag_z,
                       float accel_x, float accel_y, float accel_z,
                       float declination)
{
    /* Normalize accelerometer (gives gravity vector) */
    float accel_mag = sqrtf(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);
    if (accel_mag < 0.001f) {
        /* Fallback to 2D if accel is too weak */
        return ComputeHeading2D(mag_x, mag_y, declination);
    }
    
    float nx = accel_x / accel_mag;
    float ny = accel_y / accel_mag;
    float nz = accel_z / accel_mag;
    
    /* Compute pitch and roll from accelerometer */
    float pitch = asinf(-nx);  /* rad */
    float roll = atan2f(ny, nz);  /* rad */
    
    /* Rotate magnetic vector to horizontal plane using pitch and roll */
    float sin_pitch = sinf(pitch);
    float cos_pitch = cosf(pitch);
    float sin_roll = sinf(roll);
    float cos_roll = cosf(roll);
    
    /* Rotation matrix to horizontal plane:
     * mag_horizontal_x = mag_x * cos_pitch + mag_y * sin_roll * sin_pitch + mag_z * cos_roll * sin_pitch
     * mag_horizontal_y = mag_y * cos_roll - mag_z * sin_roll
     */
    float mag_h_x = mag_x * cos_pitch + mag_y * sin_roll * sin_pitch + mag_z * cos_roll * sin_pitch;
    float mag_h_y = mag_y * cos_roll - mag_z * sin_roll;
    
    /* Compute heading from horizontal magnetic components */
    float heading = atan2f(mag_h_x, mag_h_y) * 57.29577951308232f;
    heading += declination;
    if (heading < 0.0f) heading += 360.0f;
    if (heading >= 360.0f) heading -= 360.0f;
    return heading;
}
