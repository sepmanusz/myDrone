#include "mpu9250.h"
#include <math.h>
#include <string.h>
#include "timer_measure.h"

/* Define M_PI if not available */
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif



/* magnetometer adjustment coefficients (factory fuse ROM) */
static float mag_adjust[3] = {1.0f, 1.0f, 1.0f};
static const float MAG_RES_16BIT = 0.15f; /* uT per LSB for 16-bit */

/* Helper: read consecutive registers from MPU via SPI into buf (len bytes)
   buf[0] will receive reg, so we send reg|0x80 then read len+1 bytes and copy */
static void MPU9250_ReadRegs(SPI_HandleTypeDef *hspi,
                                   uint8_t reg,
                                   uint8_t *buf,
                                   uint8_t len)
{
    uint8_t addr = reg | 0x80;   // read bit

    HAL_GPIO_WritePin(MPU9250_CS_PORT, MPU9250_CS_PIN, GPIO_PIN_RESET);

    HAL_SPI_Transmit(hspi, &addr, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(hspi, buf, len, HAL_MAX_DELAY);

    HAL_GPIO_WritePin(MPU9250_CS_PORT, MPU9250_CS_PIN, GPIO_PIN_SET);
}

/* Helper: write a single byte to MPU register via SPI */
/* Note: we use MPU9250_WriteReg() directly; no separate raw wrapper needed. */

/* Write to AK8963 (magnetometer) via MPU I2C_SLV0 write */
static void MPU9250_I2CWriteAK8963(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t value)
{
    /* Set slave0 address (write) */
    MPU9250_WriteReg(hspi, MPU9250_I2C_SLV0_ADDR, (uint8_t)AK8963_ADDR);
    /* Register inside AK8963 */
    MPU9250_WriteReg(hspi, MPU9250_I2C_SLV0_REG, reg);
    /* Data to write */
    MPU9250_WriteReg(hspi, MPU9250_I2C_SLV0_DO, value);
    /* Enable transfer of 1 byte */
    MPU9250_WriteReg(hspi, MPU9250_I2C_SLV0_CTRL, 0x81);
    HAL_Delay(10);
}

/* Read from AK8963 via MPU I2C_SLV0 into EXT_SENS_DATA_00.. */
static void MPU9250_I2CReadAK8963(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t *out, uint8_t len)
{
    /* Set slave0 address for read (set MSB) */
    MPU9250_WriteReg(hspi, MPU9250_I2C_SLV0_ADDR, (uint8_t)(0x80 | AK8963_ADDR));
    MPU9250_WriteReg(hspi, MPU9250_I2C_SLV0_REG, reg);
    /* Enable and set number of bytes to read (enable bit 7) */
    MPU9250_WriteReg(hspi, MPU9250_I2C_SLV0_CTRL, (uint8_t)(0x80 | len));
    HAL_Delay(10);
    /* Read from EXT_SENS_DATA_00 */
    MPU9250_ReadRegs(hspi, MPU9250_EXT_SENS_DATA_00, out, len);
}

/* Initialize AK8963: read factory adjustments and set continuous 16-bit mode */
HAL_StatusTypeDef MPU9250_MagInit(SPI_HandleTypeDef *hspi)
{
    uint8_t buf[8] = {0};

    /* Enable I2C master mode */
    MPU9250_WriteReg(hspi, MPU9250_USER_CTRL, 0x20); /* I2C_MST_EN */
    HAL_Delay(10);
    /* Set I2C master clock (400 kHz) */
    MPU9250_WriteReg(hspi, MPU9250_I2C_MST_CTRL, 0x0D);
    HAL_Delay(10);

    /* Power down magnetometer */
    MPU9250_I2CWriteAK8963(hspi, AK8963_CNTL1, 0x00);
    HAL_Delay(10);

    /* Enter fuse ROM access mode to read sensitivity adjustment values */
    MPU9250_I2CWriteAK8963(hspi, AK8963_CNTL1, 0x0F);
    HAL_Delay(10);

    /* Read ASAX, ASAY, ASAZ (3 bytes) */
    MPU9250_I2CReadAK8963(hspi, AK8963_ASAX, buf, 3);
    mag_adjust[0] = (((float)buf[0] - 128.0f) / 256.0f) + 1.0f;
    mag_adjust[1] = (((float)buf[1] - 128.0f) / 256.0f) + 1.0f;
    mag_adjust[2] = (((float)buf[2] - 128.0f) / 256.0f) + 1.0f;

    /* Power down then set to continuous measurement mode 2 (100Hz), 16-bit (0x16) */
    MPU9250_I2CWriteAK8963(hspi, AK8963_CNTL1, 0x00);
    HAL_Delay(10);
    MPU9250_I2CWriteAK8963(hspi, AK8963_CNTL1, 0x16);
    HAL_Delay(10);

     /* Configure MPU9250 I2C_SLV0 to read magnetometer registers continuously
         Read 7 bytes starting at AK8963_HXL into EXT_SENS_DATA_00. Enable transfer. */
     MPU9250_WriteReg(hspi, MPU9250_I2C_SLV0_ADDR, (uint8_t)(0x80 | AK8963_ADDR)); /* read */
     MPU9250_WriteReg(hspi, MPU9250_I2C_SLV0_REG, AK8963_HXL);
     MPU9250_WriteReg(hspi, MPU9250_I2C_SLV0_CTRL, (uint8_t)(0x80 | 7)); /* enable, read 7 bytes */
     HAL_Delay(10);

    return HAL_OK;
}

/* Read magnetometer values (uT) into mx,my,mz */
HAL_StatusTypeDef MPU9250_ReadMag(SPI_HandleTypeDef *hspi, float *mx, float *my, float *mz)
{
    uint8_t raw[7] = {0};
    /* Read 7 bytes from MPU EXT_SENS_DATA_00 which is populated from AK8963 in continuous mode */
    MPU9250_ReadRegs(hspi, MPU9250_EXT_SENS_DATA_00, raw, 7);

    // Overflow check
    if (raw[6] & 0x08) {
        return HAL_ERROR;
    }

    /* Combine little-endian pairs */
    int16_t hx = (int16_t)((raw[1] << 8) | raw[0]);
    int16_t hy = (int16_t)((raw[3] << 8) | raw[2]);
    int16_t hz = (int16_t)((raw[5] << 8) | raw[4]);

    /* Apply factory adjustments and resolution to get microtesla */
    *mx = (float)hx * MAG_RES_16BIT * mag_adjust[0];
    *my = (float)hy * MAG_RES_16BIT * mag_adjust[1];
    *mz = (float)hz * MAG_RES_16BIT * mag_adjust[2];

    return HAL_OK;
}

/* Érzékenységi skálázási faktorok */
#define ACCEL_SCALE_FACTOR 8192.0f   // ±4g
#define GYRO_SCALE_FACTOR 65.5f      // ±500 dps* ±250 dps */
#define TEMP_SCALE_FACTOR 340.0f
#define ROOM_TEMP_OFFSET 36.53f



/**
 * @brief MPU9250 egy regiszter olvasása
 */
uint8_t MPU9250_ReadReg(SPI_HandleTypeDef *hspi, uint8_t reg)
{
    uint8_t tx_data[2] = {reg | 0x80, 0x00};  /* Read bit = 1 */
    uint8_t rx_data[2] = {0, 0};
    
    HAL_GPIO_WritePin(MPU9250_CS_PORT, MPU9250_CS_PIN, GPIO_PIN_RESET);

    HAL_SPI_TransmitReceive(hspi, tx_data, rx_data, 2, HAL_MAX_DELAY);

    HAL_GPIO_WritePin(MPU9250_CS_PORT, MPU9250_CS_PIN, GPIO_PIN_SET);

    
    return rx_data[1];
}

/**
 * @brief MPU9250 egy regiszter írása
 */
void MPU9250_WriteReg(SPI_HandleTypeDef *hspi, uint8_t reg, uint8_t value)
{
    uint8_t tx_data[2] = {reg & 0x7F, value};  /* Write bit = 0 */
    
    HAL_GPIO_WritePin(MPU9250_CS_PORT, MPU9250_CS_PIN, GPIO_PIN_RESET);

    HAL_SPI_Transmit(hspi, tx_data, 2, HAL_MAX_DELAY);

    HAL_GPIO_WritePin(MPU9250_CS_PORT, MPU9250_CS_PIN, GPIO_PIN_SET);

}

/**
 * @brief MPU9250 inicializálása
 */
void MPU9250_Init(SPI_HandleTypeDef *hspi)
{
    HAL_Delay(100);

    /* Ébredés, Gyro X clock */
    MPU9250_WriteReg(hspi, MPU9250_PWR_MGMT_1, 0x01);
    HAL_Delay(100);

    /* Sample Rate Divider
       1kHz / (1 + 4) = 200 Hz */
    MPU9250_WriteReg(hspi, 0x19, 0x04);
    HAL_Delay(10);

    /* CONFIG – Gyro DLPF = 3 → 41 Hz */
    MPU9250_WriteReg(hspi, MPU9250_CONFIG, 0x03);
    HAL_Delay(10);

    /* GYRO_CONFIG
       ±500 dps → FS_SEL = 1 → 0x08 */
    MPU9250_WriteReg(hspi, MPU9250_GYRO_CONFIG, 0x08);
    HAL_Delay(10);

    /* ACCEL_CONFIG
       ±4g → AFS_SEL = 1 → 0x08 */
    MPU9250_WriteReg(hspi, MPU9250_ACCEL_CONFIG, 0x08);
    HAL_Delay(10);

    /* ACCEL_CONFIG2
       Accel DLPF = 41 Hz → 0x03 */
    MPU9250_WriteReg(hspi, MPU9250_ACCEL_CONFIG2, 0x03);
    HAL_Delay(10);

    HAL_Delay(100);
}

/**
 * @brief MPU9250 ellenőrzés (WHO_AM_I regiszter olvasása)
 */
HAL_StatusTypeDef MPU9250_Check(SPI_HandleTypeDef *hspi)
{
    uint8_t who_am_i = MPU9250_ReadReg(hspi, MPU9250_WHOAMI);
    
    if (who_am_i == MPU9250_WHOAMI_VALUE) {
        return HAL_OK;
    }
    return HAL_ERROR;
}

/**
 * @brief MPU9250 adatok olvasása - részletezett időmérésekkel
 */
HAL_StatusTypeDef MPU9250_ReadData(SPI_HandleTypeDef *hspi, MPU9250_Data *data)
{
    uint8_t buf[14];
    uint8_t rawmag[7] = {0};
    
    TimerMeasure_t timer_spi_transfer;
    TimerMeasure_t timer_data_copy;
    TimerMeasure_t timer_conversion;
    
    extern void uart_printf(const char *format, ...);
    
    /* SPI transfer mérése */
    Timer_Start(&timer_spi_transfer);
    /* Burst read accel(6)+temp(2)+gyro(6) -> 14 bytes starting at ACCEL_XOUT_H */
    MPU9250_ReadRegs(hspi, MPU9250_ACCEL_XOUT_H, buf, 14);
    Timer_Stop(&timer_spi_transfer);
    Timer_Start(&timer_data_copy);
    /* data already in buf */
    Timer_Stop(&timer_data_copy);
    
    /* Adatok konvertálása */
    Timer_Start(&timer_conversion);
    
    /* Gyorsulásmérő adatok */
    int16_t accel_x = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t accel_y = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t accel_z = (int16_t)((buf[4] << 8) | buf[5]);
    
    /* Hőmérséklet adat */
    int16_t temp_raw = (int16_t)((buf[6] << 8) | buf[7]);
    
    /* Giroszkóp adatok */
    int16_t gyro_x = (int16_t)((buf[8] << 8) | buf[9]);
    int16_t gyro_y = (int16_t)((buf[10] << 8) | buf[11]);
    int16_t gyro_z = (int16_t)((buf[12] << 8) | buf[13]);
    
    /* Konvertálás fizikai egységekre */
    data->accelX = accel_x / ACCEL_SCALE_FACTOR;
    data->accelY = accel_y / ACCEL_SCALE_FACTOR;
    data->accelZ = accel_z / ACCEL_SCALE_FACTOR;
    
    data->gyroX = gyro_x / GYRO_SCALE_FACTOR;
    data->gyroY = gyro_y / GYRO_SCALE_FACTOR;
    data->gyroZ = gyro_z / GYRO_SCALE_FACTOR;
    
    data->temp = (temp_raw / TEMP_SCALE_FACTOR) + ROOM_TEMP_OFFSET;
    
    Timer_Stop(&timer_conversion);
    
    /* Read magnetometer bytes from EXT_SENS_DATA_00 (configured in MagInit) */
    MPU9250_ReadRegs(hspi, MPU9250_EXT_SENS_DATA_00, rawmag, 7);
    /* Overflow check (ST2 bit 3) */
    if (rawmag[6] & 0x08) {
        /* overflow, mark magnetometer data as invalid */
        data->magX = 0.0f;
        data->magY = 0.0f;
        data->magZ = 0.0f;
    } else {
        int16_t hx = (int16_t)((rawmag[1] << 8) | rawmag[0]);
        int16_t hy = (int16_t)((rawmag[3] << 8) | rawmag[2]);
        int16_t hz = (int16_t)((rawmag[5] << 8) | rawmag[4]);
        data->magX = (float)hx * MAG_RES_16BIT * mag_adjust[0];
        data->magY = (float)hy * MAG_RES_16BIT * mag_adjust[1];
        data->magZ = (float)hz * MAG_RES_16BIT * mag_adjust[2];
    }
    
    /* Részletezett időmérések kiiratása
    Timer_PrintElapsed("  |_SPI_TRANSFER", &timer_spi_transfer);
    Timer_PrintElapsed("  |_DATA_COPY", &timer_data_copy);
    Timer_PrintElapsed("  |_CONVERSION", &timer_conversion);*/
    
    return HAL_OK;
}

/**
 * IMU_CalculateAngles: Calculate pitch, roll from accelerometer
 * 
 * This function computes the Euler angles using accelerometer data.
 * Pitch and roll are calculated from the acceleration vector.
 * Yaw is calculated from magnetometer (compass heading).
 * 
 * Note: This method assumes near-zero acceleration (level flight).
 * For better results with movement, use IMU_ComplementaryFilter().
 */
void IMU_CalculateAngles(const MPU9250_Data *mpu_data, IMU_Angles_t *angles)
{
    if (!mpu_data || !angles) return;
    
    /* Calculate pitch and roll from accelerometer
     * Roll  = atan2(accelY, accelZ)
     * Pitch = atan2(-accelX, sqrt(accelY^2 + accelZ^2)) */
    angles->roll  = atan2f(mpu_data->accelY, mpu_data->accelZ);
    angles->pitch = atan2f(-mpu_data->accelX, 
                           sqrtf(mpu_data->accelY * mpu_data->accelY + 
                                 mpu_data->accelZ * mpu_data->accelZ));
    
    /* Calculate yaw from magnetometer (compass heading)
     * Compensate for pitch and roll for better accuracy */
    float sin_roll = sinf(angles->roll);
    float cos_roll = cosf(angles->roll);
    float sin_pitch = sinf(angles->pitch);
    float cos_pitch = cosf(angles->pitch);
    
    float mx = mpu_data->magX;
    float my = mpu_data->magY;
    float mz = mpu_data->magZ;
    
    /* Tilt-compensated heading (2D with tilt compensation) */
    float mag_x_comp = mx * cos_pitch + 
                       mz * sin_pitch;
    float mag_y_comp = mx * sin_roll * sin_pitch + 
                       my * cos_roll - 
                       mz * sin_roll * cos_pitch;
    
    angles->yaw = atan2f(-mag_y_comp, mag_x_comp);
    
    /* Convert from radians to degrees */
    angles->pitch *= 180.0f / M_PI;
    angles->roll  *= 180.0f / M_PI;
    angles->yaw   *= 180.0f / M_PI;
    
    /* Normalize yaw to 0-360 degrees */
    if (angles->yaw < 0.0f) {
        angles->yaw += 360.0f;
    }
}

/**
 * IMU_UpdateAnglesWithGyro: Update angles using gyroscope integration
 * 
 * ⚠️  WARNING: This function only integrates gyroscope data without any correction.
 * Over time, it will DRIFT due to gyroscope bias and noise!
 * 
 * Use IMU_ComplementaryFilter() instead, which combines gyro + accelerometer
 * for much better long-term accuracy.
 * 
 * This function is kept for reference or special cases only.
 * 
 * dt: time delta in seconds (sampling period)
 */
void IMU_UpdateAnglesWithGyro(IMU_Angles_t *angles, const MPU9250_Data *mpu_data, float dt)
{
    if (!angles || !mpu_data || dt <= 0.0f) return;
    
    /* Integrate gyroscope data (in degrees/second)
     * Each gyro axis contributes to the corresponding angle */
    angles->roll  += mpu_data->gyroX * dt;
    angles->pitch += mpu_data->gyroY * dt;
    angles->yaw   += mpu_data->gyroZ * dt;
    
    /* Normalize angles to avoid overflow */
    if (angles->yaw > 180.0f) {
        angles->yaw -= 360.0f;
    } else if (angles->yaw < -180.0f) {
        angles->yaw += 360.0f;
    }
}

/**
 * IMU_ComplementaryFilter: Complementary filter for better angle estimation
 * 
 * Combines accelerometer (slow, accurate but drifts with movement)
 * with gyroscope (fast, but drifts over time).
 * 
 * alpha: filter coefficient (0.0 - 1.0)
 *   - 0.98: Trust 98% gyro, 2% accelerometer (good for dynamic motion)
 *   - 0.95: Trust 95% gyro, 5% accelerometer (balanced)
 *   - 0.90: Trust 90% gyro, 10% accelerometer (more stable when stationary)
 *
 * Typical use: alpha = 0.98, dt = 0.01 (100 Hz)
 */
void IMU_ComplementaryFilter(IMU_Angles_t *angles, const MPU9250_Data *mpu_data, 
                             float dt, float alpha)
{
    if (!angles || !mpu_data || dt <= 0.0f) return;
    
    /* Calculate instantaneous angles from accelerometer */
    IMU_Angles_t accel_angles;
    IMU_CalculateAngles(mpu_data, &accel_angles);
    
    /* Update angles from gyroscope */
    float gyro_pitch = angles->pitch + mpu_data->gyroY * dt;
    float gyro_roll  = angles->roll  + mpu_data->gyroX * dt;
    float gyro_yaw   = angles->yaw   + mpu_data->gyroZ * dt;
    
    /* Complementary filter: combine gyro (fast) with accel (accurate)
     * Filtered = alpha * (previous + gyro_rate * dt) + (1 - alpha) * accel_angle */
    angles->pitch = alpha * gyro_pitch + (1.0f - alpha) * accel_angles.pitch;
    angles->roll  = alpha * gyro_roll  + (1.0f - alpha) * accel_angles.roll;
    
    /* Yaw is only from gyroscope (magnetometer is too noisy for direct calculation
     * but could be used for long-term drift correction if needed) */
    angles->yaw = gyro_yaw;
    
    /* Normalize angles */
    if (angles->yaw > 180.0f) {
        angles->yaw -= 360.0f;
    } else if (angles->yaw < -180.0f) {
        angles->yaw += 360.0f;
    }
}
