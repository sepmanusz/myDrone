/*
 * mpu9250_calibration.c
 * MPU9250 calibration routines: gyro, accel, magnetometer
 */

#include "mpu9250_calibration.h"
#include <stdint.h>
#include <math.h>
#include <string.h>
#include "uart_printf.h"
#include "main.h"
#include "stm32f7xx_hal_flash_ex.h"  /* needed for FLASH_SECTOR_ definitions */
#include "stm32f7xx_hal_gpio.h"   /* ensure GPIOx symbols available */

/* Flash storage configuration for STM32F722ZET
   Flash: 512 KB, Sector size: 16 KB
   We use the last sector (Sector 11) at address 0x080E0000 for calibration data
   Size: ~132 bytes for calibration struct, plenty of room in 16KB sector */
#define CALIBRATION_FLASH_ADDR 0x08070000  /* Last sector of STM32F722ZET (sector 7, 64KB per sector) */
#define CALIBRATION_MAGIC 0xCAFEBABE       /* Magic number to verify valid data */

/* Structure to store in flash (with magic number for validation) */
typedef struct {
    uint32_t magic;                        /* Magic number for validity check */
    MPU9250_Calibration_t calibration;
} CalibrationFlashData_t;

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
    /* polling USER button removed to avoid GPIO dependency; simple delay instead */
    uart_printf("Press USER button to continue (waiting 5s)...\r\n");
    HAL_Delay(5000);
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
    if (samples_per_axis == 0) samples_per_axis = 200;
    
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
    /* For Z we simply center between +1g and -1g readings; no extra -1 correction
       because offsets are subtracted later. */
    mpu9250_cal.accel_offset_z = (accel_z_up + accel_z_down) / 2.0f;  /* Should be ~0 */
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
    if (duration_ms == 0) duration_ms = 6000;  /* Default 60 seconds for spherical rotation */
    
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
    
    if (MPU9250_CalibrateAccel(hspi, 250) != HAL_OK) {
        uart_printf("ERROR: Accel calibration failed\r\n");
        return HAL_ERROR;
    }
    HAL_Delay(1000);
    
    if (MPU9250_CalibrateMag(hspi, 6000) != HAL_OK) {  /* 60 second spherical rotation */
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
 * @brief Save calibration data to flash (STM32F722ZET last sector)
 */
HAL_StatusTypeDef MPU9250_SaveCalibration(void)
{
    /* Create flash data with magic number */
    /* No need to store in local struct; write directly */
    
    /* Unlock flash */
    HAL_FLASH_Unlock();
    
    /* Erase sector (Sector 11 on STM32F722ZET) */
    FLASH_EraseInitTypeDef erase_init;
    uint32_t sector_error = 0;
    
    erase_init.TypeErase = FLASH_TYPEERASE_SECTORS;
    /* On STM32F722 flash has 8 sectors (0..7); use sector 7 as last sector */
    erase_init.Sector = 7;  /* last sector */
    erase_init.NbSectors = 1;
    erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3;  /* 2.7V to 3.6V */
    
    if (HAL_FLASHEx_Erase(&erase_init, &sector_error) != HAL_OK) {
        uint32_t err = HAL_FLASH_GetError();
        uart_printf("ERROR: Flash erase failed (sector error %lu, HAL error 0x%08lX)\r\n", sector_error, err);
        HAL_FLASH_Lock();
        return HAL_ERROR;
    }
    
    /* Write magic number */
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, CALIBRATION_FLASH_ADDR, CALIBRATION_MAGIC) != HAL_OK) {
        uint32_t err = HAL_FLASH_GetError();
        uart_printf("ERROR: Flash write (magic) failed, HAL error 0x%08lX\r\n", err);
        HAL_FLASH_Lock();
        return HAL_ERROR;
    }
    
    /* Write calibration data (word by word, 4 bytes each) */
    uint32_t *src_ptr = (uint32_t *)&mpu9250_cal;
    uint32_t addr = CALIBRATION_FLASH_ADDR + 4;  /* Skip magic number */
    uint32_t size = sizeof(MPU9250_Calibration_t);
    uint32_t words = (size + 3) / 4;  /* Round up to words */
    
    for (uint32_t i = 0; i < words; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, src_ptr[i]) != HAL_OK) {
            uint32_t err = HAL_FLASH_GetError();
            uart_printf("ERROR: Flash write (data @%lu) failed, HAL error 0x%08lX\r\n", i, err);
            HAL_FLASH_Lock();
            return HAL_ERROR;
        }
        addr += 4;
    }
    
    /* Lock flash */
    HAL_FLASH_Lock();
    
    uart_printf("Calibration saved to flash (at 0x%08lX)\r\n", CALIBRATION_FLASH_ADDR);
    return HAL_OK;
}

/**
 * @brief Load calibration data from flash
 */
HAL_StatusTypeDef MPU9250_LoadCalibration(void)
{
    /* Read magic number */
    uint32_t magic = *(uint32_t *)CALIBRATION_FLASH_ADDR;
    
    if (magic != CALIBRATION_MAGIC) {
        uart_printf("WARNING: No valid calibration in flash (magic: 0x%08lX, expected: 0x%08lX)\r\n", magic, CALIBRATION_MAGIC);
        uart_printf("         Using default calibration. Please run calibration and save.\r\n");
        return HAL_ERROR;  /* No valid calibration stored */
    }
    
    /* Read calibration data from flash */
    uint32_t *dest_ptr = (uint32_t *)&mpu9250_cal;
    uint32_t addr = CALIBRATION_FLASH_ADDR + 4;  /* Skip magic number */
    uint32_t size = sizeof(MPU9250_Calibration_t);
    uint32_t words = (size + 3) / 4;  /* Round up to words */
    
    for (uint32_t i = 0; i < words; i++) {
        dest_ptr[i] = *(uint32_t *)addr;
        addr += 4;
    }
    
    uart_printf("Calibration loaded from flash:\r\n");
    uart_printf("  Gyro offset:   X=%.6f  Y=%.6f  Z=%.6f dps\r\n", 
                mpu9250_cal.gyro_offset_x, mpu9250_cal.gyro_offset_y, mpu9250_cal.gyro_offset_z);
    uart_printf("  Accel offset:  X=%.6f  Y=%.6f  Z=%.6f g\r\n",
                mpu9250_cal.accel_offset_x, mpu9250_cal.accel_offset_y, mpu9250_cal.accel_offset_z);
    uart_printf("  Mag offset:    X=%.6f  Y=%.6f  Z=%.6f uT\r\n",
                mpu9250_cal.mag_offset_x, mpu9250_cal.mag_offset_y, mpu9250_cal.mag_offset_z);
    
    return HAL_OK;
}

