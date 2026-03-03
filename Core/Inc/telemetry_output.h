#ifndef TELEMETRY_OUTPUT_H
#define TELEMETRY_OUTPUT_H

#include "main.h"
#include "mpu9250.h"

/* Try to print telemetry at configured interval using integer formatting. */
void Telemetry_TryPrint(const MPU9250_Data *mpu_data,
                        const IMU_Angles_t *angles,
                        unsigned long now_ms);

#endif /* TELEMETRY_OUTPUT_H */
