/*
 * control.c
 * Extracted control logic from main while loop
 */

#include "control.h"
#include "timer_measure.h"
#include "mpu9250_calibration.h"
#include "imu_filter.h"   /* need complementary filter prototype */
#include "telemetry_output.h"

#ifndef CONTROL_LOOP_INTERVAL_MS
#define CONTROL_LOOP_INTERVAL_MS 5U   /* 200 Hz */
#endif

/* Global IMU angles for continuous tracking */
static IMU_Angles_t current_angles = {0.0f, 0.0f, 0.0f};
static uint32_t last_angle_update_time = 0;

void Control_Step(SPI_HandleTypeDef *hspi, MPU9250_Data *mpu_data, uint32_t *last_read_time,
                  TimerMeasure_t *timer_accel_read, TimerMeasure_t *timer_mpu_read,
                  TimerMeasure_t *timer_total, uint16_t tick)
{
    (void)tick;

    if (HAL_GetTick() - *last_read_time >= CONTROL_LOOP_INTERVAL_MS) {
        *last_read_time = HAL_GetTick();

        /* Teljes ciklus időmérése */
        Timer_Start(timer_total);

        /* MPU adatok olvasása és feldolgozása */
        Timer_Start(timer_mpu_read);
        if (MPU9250_ReadData(hspi, mpu_data) == HAL_OK) {
            Timer_Stop(timer_mpu_read);
            
            /* Apply calibration offsets */
            MPU9250_ApplyCalibration(mpu_data);
            
            /* ========== IMU ANGLE CALCULATION ========== */
            /* Calculate time delta for gyro integration */
            uint32_t current_time = HAL_GetTick();
            float dt = 0.005f;  /* Default 5ms = 200Hz */
            if (last_angle_update_time > 0) {
                dt = (float)(current_time - last_angle_update_time) / 1000.0f;
                if (dt > 0.05f) dt = 0.005f;  /* Clamp unreasonable deltas */
            }
            last_angle_update_time = current_time;
            
            /* Use complementary filter for better angle estimation
             * alpha = 0.98: Trust gyro 98%, accel 2% (good for dynamic motion)
             * For stationary or slow flight, use 0.90-0.95 */
            IMU_ComplementaryFilter(&current_angles, mpu_data, dt, 0.98f);

                        Telemetry_TryPrint(mpu_data, &current_angles, (unsigned long)HAL_GetTick());
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
