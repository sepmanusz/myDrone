#ifndef MPU9250_H
#define MPU9250_H

#include "main.h"

/* MPU9250 Regiszterek */
#define MPU9250_ADDR                0x68
#define MPU9250_WHOAMI              0x75
#define MPU9250_WHOAMI_VALUE        0x71

#define MPU9250_PWR_MGMT_1          0x6B
#define MPU9250_CONFIG              0x1A
#define MPU9250_GYRO_CONFIG         0x1B
#define MPU9250_ACCEL_CONFIG        0x1C
#define MPU9250_ACCEL_CONFIG2       0x1D

#define MPU9250_ACCEL_XOUT_H        0x3B
#define MPU9250_ACCEL_XOUT_L        0x3C
#define MPU9250_ACCEL_YOUT_H        0x3D
#define MPU9250_ACCEL_YOUT_L        0x3E
#define MPU9250_ACCEL_ZOUT_H        0x3F
#define MPU9250_ACCEL_ZOUT_L        0x40

#define MPU9250_TEMP_OUT_H          0x41
#define MPU9250_TEMP_OUT_L          0x42

#define MPU9250_GYRO_XOUT_H         0x43
#define MPU9250_GYRO_XOUT_L         0x44
#define MPU9250_GYRO_YOUT_H         0x45
#define MPU9250_GYRO_YOUT_L         0x46
#define MPU9250_GYRO_ZOUT_H         0x47
#define MPU9250_GYRO_ZOUT_L         0x48

/* MPU9250 and AK8963 registers for magnetometer handling */
#define MPU9250_INT_PIN_CFG        0x37
#define MPU9250_USER_CTRL          0x6A
#define MPU9250_I2C_MST_CTRL       0x24
#define MPU9250_I2C_SLV0_ADDR      0x25
#define MPU9250_I2C_SLV0_REG       0x26
#define MPU9250_I2C_SLV0_CTRL      0x27
#define MPU9250_I2C_SLV0_DO        0x63
#define MPU9250_EXT_SENS_DATA_00   0x49

#define AK8963_ADDR                0x0C
#define AK8963_WIA                 0x00
#define AK8963_ST1                 0x02
#define AK8963_HXL                 0x03
#define AK8963_ST2                 0x09
#define AK8963_CNTL1               0x0A
#define AK8963_CNTL2               0x0B
#define AK8963_ASAX                0x10

/* MPU9250 chip select port/pin */
#define MPU9250_CS_PORT GPIOB
#define MPU9250_CS_PIN GPIO_PIN_8

/* Típus definíciók */
typedef struct {
    float accelX;
    float accelY;
    float accelZ;
    float gyroX;
    float gyroY;
    float gyroZ;
    float temp;
    float magX;
    float magY;
    float magZ;
} MPU9250_Data;

/* IMU Euler angles (pitch, roll, yaw) */
typedef struct {
    float pitch;  /* Rotation around Y axis (forward/backward tilt) */
    float roll;   /* Rotation around X axis (left/right tilt) */
    float yaw;    /* Rotation around Z axis (heading) */
} IMU_Angles_t;

/* Függvények */
void MPU9250_Init(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef MPU9250_ReadData(SPI_HandleTypeDef *hspi, MPU9250_Data *data);
uint8_t MPU9250_ReadReg(SPI_HandleTypeDef *hspi, uint8_t reg);
void MPU9250_WriteReg(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t value);
HAL_StatusTypeDef MPU9250_Check(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef MPU9250_MagInit(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef MPU9250_ReadMag(SPI_HandleTypeDef *hspi, float *mx, float *my, float *mz);

/* IMU angle calculation functions */
void IMU_CalculateAngles(const MPU9250_Data *mpu_data, IMU_Angles_t *angles);
void IMU_UpdateAnglesWithGyro(IMU_Angles_t *angles, const MPU9250_Data *mpu_data, float dt);
void IMU_ComplementaryFilter(IMU_Angles_t *angles, const MPU9250_Data *mpu_data, float dt, float alpha);

#endif /* MPU9250_H */
