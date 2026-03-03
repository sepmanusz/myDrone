# IMU Kalman Filter

This document describes the simple Kalman filter implementation provided in
`Core/Inc/imu_kalman.h` / `imu_kalman.c`.

## Purpose

We want to estimate a single angle (typically yaw/heading) by fusing a noisy
angle measurement (from magnetometer or accel-derived heading) with a
high-frequency rate measurement from the gyroscope.  The gyroscope output is
accurate on short time scales but drifts over time; the magnetometer/accel
reading provides a slowly-varying absolute reference but is noisy and
sensitive to disturbances.

The Kalman filter maintains an internal state vector:

```
 x = [ angle ]
     [ bias  ]
```

`angle` is the current estimated orientation in degrees.
`bias` represents the slowly-varying offset in the gyro rate (deg/s).

### Process model

At each time step `k` with elapsed time `dt`, we predict the new state from the
previous state and the gyroscope reading (`gyro_rate`) according to

```
angle_k   = angle_{k-1} + (gyro_rate - bias_{k-1}) * dt
bias_k    = bias_{k-1}
```

The bias is assumed constant (random walk) and the only process noise comes
from uncertainty in the angle and bias evolution.  We parameterize that noise
with variances `q_angle` and `q_bias`.

The covariance matrix `P` (2×2) quantifies the filter's uncertainty in `angle`
and `bias`.

### Measurement model

An external sensor provides a direct observation of the angle:

```
z_k = angle_true + v
```

where `v` is zero-mean Gaussian measurement noise with variance `r_measure`.

During the update step the Kalman gain `K` is computed, and the state and
covariance are corrected based on the innovation `y = z_k - angle_k`.

## API

- `KalmanFilter_Init(KalmanFilter_t *kf, float initial_angle,
  float q_angle, float q_bias, float r_measure)`
  Initializes the filter state and noise parameters.

- `float KalmanFilter_Update(KalmanFilter_t *kf, float gyro_rate, float dt,
  float measured_angle)`
  Performs a single predict/update cycle and returns the filtered angle.

## Usage notes

1. Choose noise parameters based on sensor characteristics.  Smaller `q_angle`
   makes the filter trust gyro integration more (slower response to measurement
   changes); higher `r_measure` reduces the influence of the angle measurement.
2. Call `Update` at a consistent rate with accurate `dt` (seconds).  Pass the
   same `measured_angle` units (degrees) as the gyroscope readings are
   converted to.
3. This is a generic 1‑D filter; it can be used for pitch, roll, yaw, or any
   other single-degree-of-freedom quantity with a rate + absolute measurement.

## Why a Kalman filter?

Unlike a complementary filter, the Kalman filter dynamically adjusts its
blending coefficients (`K`) based on the estimated uncertainties.  It can
learn the gyro bias online and compensate for it, reducing drift over time.
It also provides a principled way to tune the filter by specifying noise
variances instead of a single alpha parameter.

The 2×2 state simplified here is suitable for embedded systems because it
requires only a few floating-point operations per step.
