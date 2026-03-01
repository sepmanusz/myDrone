/*
 * mpu9250_calibration.h
 * MPU9250 sensor calibration: gyro, accel, magnetometer offsets
 */

#ifndef MPU9250_CALIBRATION_H
#define MPU9250_CALIBRATION_H

#include "stm32f7xx_hal.h"
#include "mpu9250.h"

/* Calibration data structure */
typedef struct {
    /* Gyroscope offset (deg/s) */
    float gyro_offset_x;
    float gyro_offset_y;
    float gyro_offset_z;
    
    /* Accelerometer offset (g) */
    float accel_offset_x;
    float accel_offset_y;
    float accel_offset_z;
    
    /* Magnetometer hard-iron offset (uT) */
    float mag_offset_x;
    float mag_offset_y;
    float mag_offset_z;
    
    /* Magnetometer scale factors (soft-iron correction) */
    float mag_scale_x;
    float mag_scale_y;
    float mag_scale_z;
    
    /* Calibration status flags */
    uint8_t gyro_calibrated;
    uint8_t accel_calibrated;
    uint8_t mag_calibrated;
} MPU9250_Calibration_t;

/* Global calibration data */
extern MPU9250_Calibration_t mpu9250_cal;

/* Calibration functions */
HAL_StatusTypeDef MPU9250_CalibrateGyro(SPI_HandleTypeDef *hspi, uint16_t samples);
HAL_StatusTypeDef MPU9250_CalibrateAccel(SPI_HandleTypeDef *hspi, uint16_t samples_per_axis);
HAL_StatusTypeDef MPU9250_CalibrateMag(SPI_HandleTypeDef *hspi, uint16_t duration_ms);
HAL_StatusTypeDef MPU9250_CalibrateAll(SPI_HandleTypeDef *hspi);

/* Apply calibration offsets to raw data */
void MPU9250_ApplyCalibration(MPU9250_Data *data);

/* Reset calibration to defaults */
void MPU9250_CalibrationReset(void);

/* Heading calculation functions */
/**
 * @brief Compute 2D heading from magnetometer only (horizontal plane)
 * @param mag_x, mag_y: calibrated magnetometer readings (uT)
 * @param declination: magnetic declination in degrees (Budapest: -4.0°)
 * @return heading in degrees [0, 360)
 */
float ComputeHeading2D(float mag_x, float mag_y, float declination);

/**
 * @brief Compute tilt-compensated 3D heading
 * Uses accelerometer to get pitch/roll, then projects mag vector to horizontal plane
 * @param mag_x, mag_y, mag_z: calibrated magnetometer (uT)
 * @param accel_x, accel_y, accel_z: calibrated accelerometer (g)
 * @param declination: magnetic declination in degrees
 * @return heading in degrees [0, 360)
 */
float ComputeHeading3D(float mag_x, float mag_y, float mag_z,
                       float accel_x, float accel_y, float accel_z,
                       float declination);

#endif /* MPU9250_CALIBRATION_H */
