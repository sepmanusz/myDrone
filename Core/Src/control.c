/*
 * control.c
 * Extracted control logic from main while loop
 */

#include "control.h"
#include <math.h>
#include "mpu9250_calibration.h"

void Control_Step(SPI_HandleTypeDef *hspi, MPU9250_Data *mpu_data, uint32_t *last_read_time,
                  TimerMeasure_t *timer_accel_read, TimerMeasure_t *timer_mpu_read,
                  TimerMeasure_t *timer_total, uint16_t tick)
{
#ifdef MODE_DEBUG
	tick = 1000;
#endif

    if (HAL_GetTick() - *last_read_time >= tick) {
        *last_read_time = HAL_GetTick();

        /* Teljes ciklus időmérése */
        Timer_Start(timer_total);

        /* MPU adatok olvasása és feldolgozása */
        Timer_Start(timer_mpu_read);
        if (MPU9250_ReadData(hspi, mpu_data) == HAL_OK) {
            Timer_Stop(timer_mpu_read);
            
            /* Apply calibration offsets */
            MPU9250_ApplyCalibration(mpu_data);

            /* Print all sensor data in integer format for UART compatibility */
            /* Gyroscope (x1e-2 dps) */
            int gyro_x = (int)(mpu_data->gyroX * 100);
            int gyro_y = (int)(mpu_data->gyroY * 100);
            int gyro_z = (int)(mpu_data->gyroZ * 100);
            
            /* Accelerometer (x1e-4 g) */
            int accel_x = (int)(mpu_data->accelX * 10000);
            int accel_y = (int)(mpu_data->accelY * 10000);
            int accel_z = (int)(mpu_data->accelZ * 10000);
            
            /* Magnetometer (x0.01 uT) */
            int mag_x = (int)(mpu_data->magX * 100);
            int mag_y = (int)(mpu_data->magY * 100);
            int mag_z = (int)(mpu_data->magZ * 100);
            
            /* Heading calculation - try 2D first to diagnose the issue */
            float heading_2d = ComputeHeading2D(mpu_data->magX, mpu_data->magY, -4.0f);  /* Budapest declination */
            float heading_3d = ComputeHeading3D(mpu_data->magX, mpu_data->magY, mpu_data->magZ,
                                               mpu_data->accelX, mpu_data->accelY, mpu_data->accelZ, -4.0f);
            int heading_2d_int = (int)(heading_2d * 10);  /* x0.1 deg */
            int heading_3d_int = (int)(heading_3d * 10);  /* x0.1 deg */
            
            /* Human-readable sensor data output (no float printf) */
            /* Gyro (deg/s) with 2 decimals */
            float gx_f = mpu_data->gyroX;
            float gy_f = mpu_data->gyroY;
            float gz_f = mpu_data->gyroZ;
            int gx_whole = (int)gx_f; int gx_frac = (int)(fabsf(gx_f - gx_whole) * 100.0f);
            int gy_whole = (int)gy_f; int gy_frac = (int)(fabsf(gy_f - gy_whole) * 100.0f);
            int gz_whole = (int)gz_f; int gz_frac = (int)(fabsf(gz_f - gz_whole) * 100.0f);

            /* Accel (m/s^2) with 2 decimals */
            float ax_ms2_f = mpu_data->accelX * 9.81f;
            float ay_ms2_f = mpu_data->accelY * 9.81f;
            float az_ms2_f = mpu_data->accelZ * 9.81f;
            int ax_whole = (int)ax_ms2_f; int ax_frac = (int)(fabsf(ax_ms2_f - ax_whole) * 100.0f);
            int ay_whole = (int)ay_ms2_f; int ay_frac = (int)(fabsf(ay_ms2_f - ay_whole) * 100.0f);
            int az_whole = (int)az_ms2_f; int az_frac = (int)(fabsf(az_ms2_f - az_whole) * 100.0f);

            /* Mag (uT) with 2 decimals */
            float mx_f = mpu_data->magX;
            float my_f = mpu_data->magY;
            float mz_f = mpu_data->magZ;
            int mx_whole = (int)mx_f; int mx_frac = (int)(fabsf(mx_f - mx_whole) * 100.0f);
            int my_whole = (int)my_f; int my_frac = (int)(fabsf(my_f - my_whole) * 100.0f);
            int mz_whole = (int)mz_f; int mz_frac = (int)(fabsf(mz_f - mz_whole) * 100.0f);

            /* Heading (deg) with 1 decimal */
            int hd2d_whole = (int)heading_2d; int hd2d_frac = (int)(fabsf(heading_2d - hd2d_whole) * 10.0f);
            int hd3d_whole = (int)heading_3d; int hd3d_frac = (int)(fabsf(heading_3d - hd3d_whole) * 10.0f);

            uart_printf(
            		" Gyro:  X=%d.%02d  Y=%d.%02d  Z=%d.%02d dps   |\n\r"
            		" Accel: X=%d.%02d  Y=%d.%02d  Z=%d.%02d m/s^2 |\n\r"
            		" Mag:   X=%d.%02d  Y=%d.%02d  Z=%d.%02d uT    |\n\r"
            		" H2D: %d.%1d deg | H3D: %d.%1d deg\r\n\n",
                       gx_whole, gx_frac, gy_whole, gy_frac, gz_whole, gz_frac,
                       ax_whole, ax_frac, ay_whole, ay_frac, az_whole, az_frac,
                       mx_whole, mx_frac, my_whole, my_frac, mz_whole, mz_frac,
                       hd2d_whole, hd2d_frac, hd3d_whole, hd3d_frac);

#ifdef MODE_DEBUG
            Timer_PrintElapsed("ACCEL_REGS", timer_accel_read);
            Timer_PrintElapsed("MPU_READ", timer_mpu_read);
#endif
        }

        Timer_Stop(timer_total);
#ifdef MODE_DEBUG
        Timer_PrintElapsed("TOTAL_CYCLE", timer_total);
        uart_printf("---\r\n");
#endif
    }
}
