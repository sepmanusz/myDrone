/*
 * control.c
 * Extracted control logic from main while loop
 */

#include "control.h"
#include <math.h>
#include "mpu9250_calibration.h"

/* Global IMU angles for continuous tracking */
static IMU_Angles_t current_angles = {0.0f, 0.0f, 0.0f};
static uint32_t last_angle_update_time = 0;

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
            
            /* ========== IMU ANGLE CALCULATION ========== */
            /* Calculate time delta for gyro integration */
            uint32_t current_time = HAL_GetTick();
            float dt = 0.01f;  /* Default 10ms = 100Hz */
            if (last_angle_update_time > 0) {
                dt = (float)(current_time - last_angle_update_time) / 1000.0f;
                if (dt > 0.1f) dt = 0.01f;  /* Clamp unreasonable deltas */
            }
            last_angle_update_time = current_time;
            
            /* Use complementary filter for better angle estimation
             * alpha = 0.98: Trust gyro 98%, accel 2% (good for dynamic motion)
             * For stationary or slow flight, use 0.90-0.95 */
            IMU_ComplementaryFilter(&current_angles, mpu_data, dt, 0.98f);
            
            /* Convert angles to integer format for UART printing */
            int pitch_deg = (int)current_angles.pitch;
            int pitch_frac = (int)(fabsf(current_angles.pitch - pitch_deg) * 10.0f);
            int roll_deg = (int)current_angles.roll;
            int roll_frac = (int)(fabsf(current_angles.roll - roll_deg) * 10.0f);
            int yaw_deg = (int)current_angles.yaw;
            int yaw_frac = (int)(fabsf(current_angles.yaw - yaw_deg) * 10.0f);
            
            /* Human-readable sensor data output (no float printf) */
            /* only send telemetry at a limited rate so the UART and PC side can keep up
               (100Hz sensor sampling is fine but printing every single cycle may overflow
               the serial buffer and python reader).  Setting TELEMETRY_INTERVAL_MS to 100
               gives ~10Hz output. */
#ifndef TELEMETRY_INTERVAL_MS
#define TELEMETRY_INTERVAL_MS 100
#endif
            static uint32_t last_telemetry_time = 0;
            uint32_t now = HAL_GetTick();
            if ((now - last_telemetry_time) >= TELEMETRY_INTERVAL_MS) {
                last_telemetry_time = now;

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

                uart_printf(
                     //   " Gyro:  X=%d.%02d  Y=%d.%02d  Z=%d.%02d dps   |\n\r"
                     //   " Accel: X=%d.%02d  Y=%d.%02d  Z=%d.%02d m/s^2 |\n\r"
                     //   " Mag:   X=%d.%02d  Y=%d.%02d  Z=%d.%02d uT    |\n\r"
                        	">pitch:%d.%1d roll:%d.%1d yaw:%d.%1d\r\n\n",
                       //    gx_whole, gx_frac, gy_whole, gy_frac, gz_whole, gz_frac,
                      //     ax_whole, ax_frac, ay_whole, ay_frac, az_whole, az_frac,
                       //    mx_whole, mx_frac, my_whole, my_frac, mz_whole, mz_frac,
                           pitch_deg, pitch_frac, roll_deg, roll_frac, yaw_deg, yaw_frac);

                uart_printf(">pitch:%d,roll:%d,yaw:%d\r\n",
                		(int)current_angles.pitch,
						(int)current_angles.roll,
						(int)current_angles.yaw);
            }
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

/**
 * Control_GetAngles: Get current IMU angles calculated by Control_Step
 * 
 * Use this function in your motor control logic to get the current
 * pitch, roll, and yaw angles for stabilization.
 */
void Control_GetAngles(IMU_Angles_t *angles)
{
    if (angles) {
        *angles = current_angles;
    }
}
