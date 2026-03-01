# myDrone
Flight controller project repository


/*
 * CALIBRATION USAGE EXAMPLE
 * How to integrate MPU9250 calibration into your firmware
 */

/*
 * OPTION 1: Full automatic calibration (recommended for first time)
 * 
 * Add this to main() after MPU9250_MagInit():
 * 
 *   if (MPU9250_CalibrateAll(&hspi1) == HAL_OK) {
 *       uart_printf("All calibrations successful\r\n");
 *   } else {
 *       uart_printf("Calibration failed\r\n");
 *   }
 * 
 * Sequence:
 *   1. Gyro calibration: Keep device STATIONARY for ~2 seconds (200 samples @ 10ms)
 *   2. Accel calibration: Place flat (+Z up), then flip upside down (-Z up)
 *   3. Mag calibration: Rotate device in figure-8 pattern for 15 seconds
 */

/*
 * OPTION 2: Individual calibration (if you want to skip some sensors)
 * 
 *   MPU9250_CalibrateGyro(&hspi1, 200);      // 200 samples, ~2 seconds
 *   HAL_Delay(1000);
 *   MPU9250_CalibrateAccel(&hspi1, 100);     // 100 samples per orientation
 *   HAL_Delay(1000);
 *   MPU9250_CalibrateMag(&hspi1, 10000);     // 10 second rotation
 */

/*
 * OPTION 3: Manual calibration with custom parameters
 * 
 *   MPU9250_CalibrateGyro(&hspi1, 500);      // More samples for higher precision
 *   MPU9250_CalibrateMag(&hspi1, 20000);     // Longer rotation period
 */

/*
 * CALIBRATION DATA PERSISTENCE (Optional enhancement)
 * 
 * To save calibration to flash/EEPROM:
 * 
 *   // After calibration succeeds
 *   save_calibration_to_flash(&mpu9250_cal);
 * 
 * To restore on startup:
 * 
 *   if (load_calibration_from_flash(&mpu9250_cal) == OK) {
 *       uart_printf("Calibration loaded from flash\r\n");
 *   } else {
 *       uart_printf("No stored calibration, performing new calibration\r\n");
 *       MPU9250_CalibrateAll(&hspi1);
 *   }
 */

/*
 * CALIBRATION RESULTS INTERPRETATION
 * 
 * Gyroscope:
 *   - Offsets should be small, typically ±2-5 deg/s
 *   - If very large (>50 deg/s), check sensor orientation or initialization
 * 
 * Accelerometer:
 *   - X/Y offsets should be near 0.0g (±0.1g typical)
 *   - Z offset should be near 0.0g (±0.1g typical)
 *   - If larger, device may not have been held level during calibration
 * 
 * Magnetometer hard-iron:
 *   - Offsets indicate magnetic material near device (permanent magnets, steel)
 *   - Typical range: ±100 uT depending on environment
 *   - If very large (>500 uT), strong magnetic interference likely
 * 
 * Magnetometer soft-iron (scale factors):
 *   - Should be close to 1.0 (typical: 0.95-1.05)
 *   - Indicates gain mismatch between axes
 *   - Very different values (e.g., X=1.5, Y=0.8) suggest sensor damage
 */

/*
 * CALIBRATION PRECISION TIPS
 * 
 * 1. Gyro: Minimize vibration; place on stable surface away from movement
 * 2. Accel: Use level surface; verify with bubble level for best accuracy
 * 3. Mag: Perform rotation away from ferrous objects (steel tables, monitors)
 *    - If heading drifts, repeat mag calibration in the intended operating area
 * 
 * 4. Environmental factors:
 *    - Temperature: Recalibrate if temperature changes >10°C
 *    - Location: Magnetic declination varies; use Budapest offsets (~-4.0°)
 */

/*
 * REAL-TIME CALIBRATION STATUS
 * 
 * Check which sensors are calibrated:
 * 
 *   if (mpu9250_cal.gyro_calibrated)
 *       uart_printf("Gyro calibrated\r\n");
 *   if (mpu9250_cal.accel_calibrated)
 *       uart_printf("Accel calibrated\r\n");
 *   if (mpu9250_cal.mag_calibrated)
 *       uart_printf("Mag calibrated\r\n");
 * 
 * View stored offsets anytime:
 * 
 *   uart_printf("Gyro offset: %.3f, %.3f, %.3f dps\r\n",
 *              mpu9250_cal.gyro_offset_x,
 *              mpu9250_cal.gyro_offset_y,
 *              mpu9250_cal.gyro_offset_z);
 */

/*
 * RECALIBRATION / RESET
 * 
 * To clear all calibration and return to defaults:
 * 
 *   MPU9250_CalibrationReset();
 *   // Calibration values are now zero, offsets no longer applied
 */

