#include "imu_kalman.h"

void KalmanFilter_Init(KalmanFilter_t *kf, float initial_angle,
                       float q_angle, float q_bias, float r_measure)
{
    if (!kf) return;
    kf->angle = initial_angle;
    kf->bias  = 0.0f;
    kf->P[0][0] = 0.0f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 0.0f;
    kf->q_angle   = q_angle;
    kf->q_bias    = q_bias;
    kf->r_measure = r_measure;
}

float KalmanFilter_Update(KalmanFilter_t *kf,
                          float gyro_rate,
                          float dt,
                          float measured_angle)
{
    if (!kf || dt <= 0.0f) return kf ? kf->angle : 0.0f;

    /* Predict step ----------------------------------------------------- */
    /* angle prediction: previous angle plus gyro (minus bias) * dt */
    float rate = gyro_rate - kf->bias;
    kf->angle += rate * dt;

    /* update covariance matrix P = A*P*A' + Q
       with A = [[1, -dt], [0,1]] and Q = [[q_angle, 0], [0, q_bias]] */
    float P00 = kf->P[0][0] + dt * (dt * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->q_angle);
    float P01 = kf->P[0][1] - dt * kf->P[1][1];
    float P10 = kf->P[1][0] - dt * kf->P[1][1];
    float P11 = kf->P[1][1] + kf->q_bias;
    kf->P[0][0] = P00;
    kf->P[0][1] = P01;
    kf->P[1][0] = P10;
    kf->P[1][1] = P11;

    /* Measurement update ------------------------------------------------ */
    /* innovation y = z - angle */
    float y = measured_angle - kf->angle;
    /* innovation covariance S = P00 + r_measure */
    float S = kf->P[0][0] + kf->r_measure;
    /* Kalman gain K = P * [1;0] / S */
    float K0 = kf->P[0][0] / S;
    float K1 = kf->P[1][0] / S;

    /* apply correction */
    kf->angle += K0 * y;
    kf->bias  += K1 * y;

    /* update covariance P = (I - K*[1 0]) * P */
    kf->P[0][0] -= K0 * kf->P[0][0];
    kf->P[0][1] -= K0 * kf->P[0][1];
    kf->P[1][0] -= K1 * kf->P[0][0];
    kf->P[1][1] -= K1 * kf->P[0][1];

    return kf->angle;
}
