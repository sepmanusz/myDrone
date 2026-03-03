#include <stdint.h>
#include "telemetry_output.h"

#include <math.h>
#include "uart_printf.h"

#ifndef TELEMETRY_INTERVAL_MS
#define TELEMETRY_INTERVAL_MS 100U    /* 10 Hz */
#endif

void Telemetry_TryPrint(const MPU9250_Data *mpu_data,
                        const IMU_Angles_t *angles,
                        unsigned long now_ms)
{
    if (!mpu_data || !angles) return;

    static unsigned long last_telemetry_time = 0;
    if ((now_ms - last_telemetry_time) < TELEMETRY_INTERVAL_MS) {
        return;
    }
    last_telemetry_time = now_ms;

    float gx_f = mpu_data->gyroX;
    float gy_f = mpu_data->gyroY;
    float gz_f = mpu_data->gyroZ;
    int gx_whole = (int)gx_f; int gx_frac = (int)(fabsf(gx_f - gx_whole) * 100.0f);
    int gy_whole = (int)gy_f; int gy_frac = (int)(fabsf(gy_f - gy_whole) * 100.0f);
    int gz_whole = (int)gz_f; int gz_frac = (int)(fabsf(gz_f - gz_whole) * 100.0f);

    float ax_ms2_f = mpu_data->accelX * 9.81f;
    float ay_ms2_f = mpu_data->accelY * 9.81f;
    float az_ms2_f = mpu_data->accelZ * 9.81f;
    int ax_whole = (int)ax_ms2_f; int ax_frac = (int)(fabsf(ax_ms2_f - ax_whole) * 100.0f);
    int ay_whole = (int)ay_ms2_f; int ay_frac = (int)(fabsf(ay_ms2_f - ay_whole) * 100.0f);
    int az_whole = (int)az_ms2_f; int az_frac = (int)(fabsf(az_ms2_f - az_whole) * 100.0f);

    float mx_f = mpu_data->magX;
    float my_f = mpu_data->magY;
    float mz_f = mpu_data->magZ;
    int mx_whole = (int)mx_f; int mx_frac = (int)(fabsf(mx_f - mx_whole) * 100.0f);
    int my_whole = (int)my_f; int my_frac = (int)(fabsf(my_f - my_whole) * 100.0f);
    int mz_whole = (int)mz_f; int mz_frac = (int)(fabsf(mz_f - mz_whole) * 100.0f);

    int pitch_deg = (int)angles->pitch;
    int pitch_frac = (int)(fabsf(angles->pitch - pitch_deg) * 10.0f);
    int roll_deg = (int)angles->roll;
    int roll_frac = (int)(fabsf(angles->roll - roll_deg) * 10.0f);
    int yaw_deg = (int)angles->yaw;
    int yaw_frac = (int)(fabsf(angles->yaw - yaw_deg) * 10.0f);

    uart_printf(
            " Gyro:  X=%d.%02d  Y=%d.%02d  Z=%d.%02d dps   |\n\r"
            " Accel: X=%d.%02d  Y=%d.%02d  Z=%d.%02d m/s^2 |\n\r"
            " Mag:   X=%d.%02d  Y=%d.%02d  Z=%d.%02d uT    |\n\r"
            "Orientation:\tPitch:%d.%1d Roll:%d.%1d Yaw:%d.%1d\r\n\n",
               gx_whole, gx_frac, gy_whole, gy_frac, gz_whole, gz_frac,
               ax_whole, ax_frac, ay_whole, ay_frac, az_whole, az_frac,
               mx_whole, mx_frac, my_whole, my_frac, mz_whole, mz_frac,
               pitch_deg, pitch_frac, roll_deg, roll_frac, yaw_deg, yaw_frac);
}
