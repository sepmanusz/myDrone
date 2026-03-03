#ifndef IMU_KALMAN_H
#define IMU_KALMAN_H

#include "mpu9250.h"  /* for IMU_Angles_t, MPU9250_Data */

/* Simple 1‑D Kalman filter state for angle estimation using gyro + magnetometer
 * (or accel‑derived heading).  The state vector contains the current angle
 * estimate and an estimate of the gyro bias.  The filter assumes the following
 * discrete‑time process model:
 *
 *     angle_k   = angle_{k-1} + (gyro_rate - bias) * dt
 *     bias_k    = bias_{k-1}
 *
 * with measurements z_k = angle_true + noise (e.g. heading from mag/accel).
 *
 * The covariance matrix P (2x2) is maintained internally.
 */

typedef struct {
    float angle;          /* estimated angle (degrees)      */
    float bias;           /* estimated gyro bias (deg/s)    */
    float P[2][2];        /* error covariance matrix        */
    /* tuning parameters (process / measurement noise) */
    float q_angle;        /* process noise variance for angle */
    float q_bias;         /* process noise variance for bias  */
    float r_measure;      /* measurement noise variance       */
} KalmanFilter_t;

/** Initialize Kalman filter state. */
void KalmanFilter_Init(KalmanFilter_t *kf, float initial_angle,
                       float q_angle, float q_bias, float r_measure);

/**
 * Update the filter with a new gyro rate and a new angle measurement.
 *
 * @param kf         pointer to filter state
 * @param gyro_rate  measured angular rate (deg/s) from gyro
 * @param dt         elapsed time since previous update (s)
 * @param measured_angle  heading/angle measurement (deg)
 * @return           filtered angle estimate (deg)
 */
float KalmanFilter_Update(KalmanFilter_t *kf,
                          float gyro_rate,
                          float dt,
                          float measured_angle);

#endif /* IMU_KALMAN_H */
