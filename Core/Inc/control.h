/*
 * control.h
 * Control loop extraction for main while
 */
#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>
#include <string.h>
#include "stm32f7xx_hal.h"
#include "mpu9250.h"
#include "mpu9250_calibration.h"
#include "timer_measure.h"



void Control_Step(SPI_HandleTypeDef *hspi, MPU9250_Data *mpu_data, uint32_t *last_read_time,
                  TimerMeasure_t *timer_accel_read, TimerMeasure_t *timer_mpu_read,
                  TimerMeasure_t *timer_total, uint16_t tick);

/* Get current IMU angles for external use (e.g., motor control) */
void Control_GetAngles(IMU_Angles_t *angles);

#endif /* CONTROL_H */
