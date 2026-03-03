#ifndef IMU_FILTER_H
#define IMU_FILTER_H

#include "mpu9250.h"  /* for IMU_Angles_t, MPU9250_Data */

/* complementary filter: fuse gyro, accel and mag into stable Euler angles.
 * angles  - in/out structure holding current Euler angles (degrees)
 * mpu_data - sensor readings (scaled to deg/s, g, uT)
 * dt      - elapsed time in seconds
 * alpha   - complementary coefficient (0..1). 0=all accel/mag, 1=all gyro.
 */
/* integrate gyroscope rates to produce Euler-angle prediction
 * "angles" is the previous state, mpu_data provides gyroX/Y/Z (deg/s).  
 * The result is written into "out" (degrees). */
void IMU_IntegrateGyro(const IMU_Angles_t *angles,
                       const MPU9250_Data *mpu_data,
                       float dt,
                       IMU_Angles_t *out);

void IMU_ComplementaryFilter(IMU_Angles_t *angles,
                             const MPU9250_Data *mpu_data,
                             float dt,
                             float alpha);

#endif /* IMU_FILTER_H */
