#include "imu_filter.h"
#include <math.h>

/* provide M_PI if not defined by math.h */
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* helper: convert body-rate gyroscope readings into Euler-angle prediction
 * based on current roll/pitch/yaw.  The mathematics are the same as the
 * integration portion originally embedded in the complementary filter.
 */
void IMU_IntegrateGyro(const IMU_Angles_t *angles,
                       const MPU9250_Data *mpu_data,
                       float dt,
                       IMU_Angles_t *out)
{
    if (!angles || !mpu_data || !out || dt <= 0.0f) return;

    /* copy existing state as starting point */
    *out = *angles;

    float p = mpu_data->gyroX;
    float q = mpu_data->gyroY;
    float r = IMU_YAW_SIGN * mpu_data->gyroZ;

    float phi   = angles->roll  * M_PI/180.0f;
    float theta = angles->pitch * M_PI/180.0f;
    float sin_phi   = sinf(phi);
    float cos_phi   = cosf(phi);
    float sin_theta = sinf(theta);
    float cos_theta = cosf(theta);

    float tan_theta = (fabsf(cos_theta) > 1e-3f) ? sin_theta / cos_theta : 0.0f;
    float sec_theta = (fabsf(cos_theta) > 1e-3f) ? 1.0f / cos_theta : 0.0f;

    /* Euler rate equations (see comments in original code) */
    float phi_dot   = p + q * sin_phi * tan_theta + r * cos_phi * tan_theta;
    float theta_dot = q * cos_phi - r * sin_phi;
    float psi_dot   = q * sin_phi * sec_theta + r * cos_phi * sec_theta;

    out->roll  += phi_dot   * dt;
    out->pitch += theta_dot * dt;
    out->yaw   += psi_dot   * dt;
}

void IMU_ComplementaryFilter(IMU_Angles_t *angles,
                             const MPU9250_Data *mpu_data,
                             float dt,
                             float alpha)
{
    if (!angles || !mpu_data || dt <= 0.0f) return;

    /* Stationary gyro-bias adaptation to reduce long-term drift.
     * Learns bias only when motion is small and accel magnitude is near 1g. */
    static float gyro_bias_x = 0.0f;
    static float gyro_bias_y = 0.0f;
    static float gyro_bias_z = 0.0f;

    float accel_norm_g = sqrtf(mpu_data->accelX * mpu_data->accelX +
                               mpu_data->accelY * mpu_data->accelY +
                               mpu_data->accelZ * mpu_data->accelZ);

    float abs_gx = fabsf(mpu_data->gyroX);
    float abs_gy = fabsf(mpu_data->gyroY);
    float abs_gz = fabsf(mpu_data->gyroZ);

    int stationary = (fabsf(accel_norm_g - 1.0f) < 0.08f) &&
                     (abs_gx < 1.5f) &&
                     (abs_gy < 1.5f) &&
                     (abs_gz < 1.5f);

    if (stationary) {
        float tau_s = 8.0f;
        float beta = dt / (tau_s + dt);
        gyro_bias_x += beta * (mpu_data->gyroX - gyro_bias_x);
        gyro_bias_y += beta * (mpu_data->gyroY - gyro_bias_y);
        gyro_bias_z += beta * (mpu_data->gyroZ - gyro_bias_z);
    }

    MPU9250_Data corrected = *mpu_data;
    corrected.gyroX = mpu_data->gyroX - gyro_bias_x;
    corrected.gyroY = mpu_data->gyroY - gyro_bias_y;
    corrected.gyroZ = mpu_data->gyroZ - gyro_bias_z;
    
    /* Compute instantaneous angles purely from accel+mag readings */
    IMU_Angles_t accel_angles;
    IMU_AccelMagAngles(&corrected, &accel_angles);
    
    /* Update angles from gyroscope (convert body rates to Euler rates)
     *
     * The raw gyroZ value is rotation about the body z-axis, which no longer
     * corresponds directly to change in yaw when the platform is tilted.  To
     * get the inertial yaw rate we must transform body rates using the current
     * roll/pitch.  Similarly, roll and pitch integration should use the full
     * Euler rate equations for consistency.
     *
     * Equations (p=gyroX, q=gyroY, r=gyroZ):
     *   phi_dot   = p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta)
     *   theta_dot = q*cos(phi) - r*sin(phi)
     *   psi_dot   = q*sin(phi)/cos(theta) + r*cos(phi)/cos(theta)
     *
     * where phi = roll, theta = pitch, psi = yaw.
     */
    /* integrate gyro body rates into Euler-angle estimates */
    IMU_Angles_t gyro_angles;
    IMU_IntegrateGyro(angles, &corrected, dt, &gyro_angles);

    float gyro_roll  = gyro_angles.roll;
    float gyro_pitch = gyro_angles.pitch;
    float gyro_yaw   = gyro_angles.yaw;

    /* fuse roll/pitch with accelerometer */
    angles->pitch = alpha * gyro_pitch + (1.0f - alpha) * accel_angles.pitch;
    angles->roll  = alpha * gyro_roll  + (1.0f - alpha) * accel_angles.roll;

    /* precompute cosines of current roll/pitch for later weighting */
    float phi   = angles->roll  * M_PI/180.0f;
    float theta = angles->pitch * M_PI/180.0f;
    float cos_phi = cosf(phi);
    float cos_theta = cosf(theta);

    /* yaw fusion follows below */

    /* Yaw fusion with magnetometer heading, correcting for discontinuities.
     *
     * The raw mag‑based yaw (accel_angles.yaw) can jump when the heading
     * crosses the 0/360 boundary or when the sensor reading glitches. We
     * compute the error relative to the gyro prediction and wrap it into
     * [-180,180] before blending. */
    float mag_yaw = accel_angles.yaw;
    if (mag_yaw > 180.0f) {
        mag_yaw -= 360.0f;
    }
    mag_yaw *= IMU_YAW_SIGN;

     /* compute yaw error once here */
    float yaw_error = mag_yaw - gyro_yaw;
    if (yaw_error > 180.0f) yaw_error -= 360.0f;
    else if (yaw_error < -180.0f) yaw_error += 360.0f;

     /* Fuse yaw error back into gyro prediction.
      * Reduce mag trust when highly tilted to suppress pitch/roll-induced yaw drift. */
     float mag_weight = fabsf(cos_phi * cos_theta);

     /* Extra tilt gating: above ~8 deg tilt, attenuate mag correction quickly. */
     float tilt_deg = sqrtf(angles->roll * angles->roll + angles->pitch * angles->pitch);
     if (tilt_deg > 8.0f) {
         float excess = tilt_deg - 8.0f;
         float tilt_gate = expf(-excess / 10.0f);
         if (tilt_gate < 0.05f) tilt_gate = 0.05f;
         mag_weight *= tilt_gate;
     }

     if (mag_weight < 0.03f) mag_weight = 0.03f;
     if (mag_weight > 1.0f) mag_weight = 1.0f;

     angles->yaw = gyro_yaw + (1.0f - alpha) * mag_weight * yaw_error;
    /* normalize final yaw back into [-180, 180] */
    if (angles->yaw > 180.0f) {
        angles->yaw -= 360.0f;
    } else if (angles->yaw < -180.0f) {
        angles->yaw += 360.0f;
    }
}
