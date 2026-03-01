/**
 * drone_stabilization_example.c
 * 
 * Gyakorlati exemplo: Drón stabilizáció az IMU szögekből
 * Ez egy template, amit beépíthetsz a main.c-be vagy egy külön modulba
 */

#include "stm32f7xx_hal.h"
#include "control.h"
#include "math.h"

/* ===== PID Controller Struktura ===== */
typedef struct {
    float Kp;           // Arányos erősítés
    float Ki;           // Integrális erősítés
    float Kd;           // Derivatív erősítés
    float integral;     // Integrátor
    float prev_error;   // Előző hiba (deriválthoz)
    float output_max;   // Szaturáció
    float output_min;
} PID_Controller_t;

/* ===== Motor PWM Output ===== */
typedef struct {
    float throttle;     // 0.0 - 1.0 (teljesítmény)
    float motor1;       // Frontális jobb
    float motor2;       // Hátsó jobb
    float motor3;       // Hátsó bal
    float motor4;       // Frontális bal
} Motor_Output_t;

/* ===== Drón Kontroller Struktúra ===== */
typedef struct {
    IMU_Angles_t target_angles;     // Kívánt szögek
    IMU_Angles_t current_angles;    // Aktuális szögek
    PID_Controller_t pid_pitch;
    PID_Controller_t pid_roll;
    PID_Controller_t pid_yaw;
    Motor_Output_t motor_output;
    uint8_t stabilization_enabled;
} Drone_Controller_t;

/* Globális drone controller instance */
static Drone_Controller_t drone = {0};

/**
 * Initialize PID controller with typical drone tuning values
 */
void PID_Init(PID_Controller_t *pid, float Kp, float Ki, float Kd, 
              float max_output, float min_output)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output_max = max_output;
    pid->output_min = min_output;
}

/**
 * PID controller update
 * error: hiba (kívánt - aktuális)
 * dt: időlépés (másodperc)
 * return: kontroller kimenet
 */
float PID_Update(PID_Controller_t *pid, float error, float dt)
{
    if (dt <= 0.0f) return 0.0f;
    
    /* Arányos tag */
    float P = pid->Kp * error;
    
    /* Integrális tag (anti-windup: csak ha kimenet nem szaturált) */
    pid->integral += error * dt;
    // Integrátor limit
    if (pid->integral > 10.0f) pid->integral = 10.0f;
    if (pid->integral < -10.0f) pid->integral = -10.0f;
    float I = pid->Ki * pid->integral;
    
    /* Derivatív tag */
    float derivative = (error - pid->prev_error) / dt;
    float D = pid->Kd * derivative;
    pid->prev_error = error;
    
    /* Összesen */
    float output = P + I + D;
    
    /* Szaturáció */
    if (output > pid->output_max) output = pid->output_max;
    if (output < pid->output_min) output = pid->output_min;
    
    return output;
}

/**
 * Initialize drone controller with default PID gains
 * 
 * Tuning notes:
 * - Kp: Kezdd 0.5-vel, növeld amíg nem lesz oszcilláció
 * - Ki: Integráló rész a steady-state hibához
 * - Kd: Csillapítás az oszcillációhoz
 */
void Drone_ControllerInit(void)
{
    /* ===== Quadrotor PID alap tuning ===== */
    
    /* Pitch szabályozó (előre/hátra stabilizáció) */
    PID_Init(&drone.pid_pitch, 
             1.2f,    // Kp
             0.3f,    // Ki
             0.4f,    // Kd
             300.0f,  // Max output
             -300.0f);// Min output
    
    /* Roll szabályozó (balra/jobbra stabilizáció) */
    PID_Init(&drone.pid_roll,
             1.2f,    // Kp
             0.3f,    // Ki
             0.4f,    // Kd
             300.0f,
             -300.0f);
    
    /* Yaw szabályozó (irány/fordulás) */
    PID_Init(&drone.pid_yaw,
             0.8f,    // Kp (yaw kevésbé érzékeny)
             0.2f,    // Ki
             0.2f,    // Kd
             200.0f,
             -200.0f);
    
    drone.target_angles.pitch = 0.0f;
    drone.target_angles.roll = 0.0f;
    drone.target_angles.yaw = 0.0f;
    drone.stabilization_enabled = 1;
}

/**
 * Stabilization controller - main control loop
 * Hívd ezt meg 100 Hz-en (10ms-onként)
 * 
 * throttle: 0.0-1.0 (0% = off, 1.0 = full throttle)
 */
void Drone_UpdateStabilization(float throttle, float dt)
{
    if (!drone.stabilization_enabled) {
        drone.motor_output.motor1 = 0.0f;
        drone.motor_output.motor2 = 0.0f;
        drone.motor_output.motor3 = 0.0f;
        drone.motor_output.motor4 = 0.0f;
        return;
    }
    
    /* Get current angles from IMU */
    Control_GetAngles(&drone.current_angles);
    
    /* ===== Pitch Control ===== */
    float pitch_error = drone.target_angles.pitch - drone.current_angles.pitch;
    float pitch_output = PID_Update(&drone.pid_pitch, pitch_error, dt);
    
    /* ===== Roll Control ===== */
    float roll_error = drone.target_angles.roll - drone.current_angles.roll;
    float roll_output = PID_Update(&drone.pid_roll, roll_error, dt);
    
    /* ===== Yaw Control ===== */
    float yaw_error = drone.target_angles.yaw - drone.current_angles.yaw;
    // Normalize yaw error to ±180 degrees
    if (yaw_error > 180.0f) yaw_error -= 360.0f;
    if (yaw_error < -180.0f) yaw_error += 360.0f;
    float yaw_output = PID_Update(&drone.pid_yaw, yaw_error, dt);
    
    /* ===== Quadrotor Motor Mixing ===== */
    /* Standard X configuration:
     *     Motor1(FR)  Motor2(BR)
     *          \ /
     *           X
     *          / \
     *     Motor4(FL)  Motor3(BL)
     * 
     * Motor mapping:
     * M1 (Front Right): +pitch +roll -yaw
     * M2 (Back Right):  -pitch +roll +yaw
     * M3 (Back Left):   -pitch -roll -yaw
     * M4 (Front Left):  +pitch -roll +yaw
     */
    
    float base_speed = throttle * 1000.0f;  // 0-1000 PWM
    
    drone.motor_output.motor1 = base_speed + pitch_output + roll_output - yaw_output;
    drone.motor_output.motor2 = base_speed - pitch_output + roll_output + yaw_output;
    drone.motor_output.motor3 = base_speed - pitch_output - roll_output - yaw_output;
    drone.motor_output.motor4 = base_speed + pitch_output - roll_output + yaw_output;
    
    /* Safety: Clamp motor outputs */
    float motor_min = 0.0f;
    float motor_max = 1000.0f;
    
    if (drone.motor_output.motor1 < motor_min) drone.motor_output.motor1 = motor_min;
    if (drone.motor_output.motor1 > motor_max) drone.motor_output.motor1 = motor_max;
    
    if (drone.motor_output.motor2 < motor_min) drone.motor_output.motor2 = motor_min;
    if (drone.motor_output.motor2 > motor_max) drone.motor_output.motor2 = motor_max;
    
    if (drone.motor_output.motor3 < motor_min) drone.motor_output.motor3 = motor_min;
    if (drone.motor_output.motor3 > motor_max) drone.motor_output.motor3 = motor_max;
    
    if (drone.motor_output.motor4 < motor_min) drone.motor_output.motor4 = motor_min;
    if (drone.motor_output.motor4 > motor_max) drone.motor_output.motor4 = motor_max;
}

/**
 * Set target angles for drone stabilization
 * pitch, roll: -45...+45 degrees (javasolt tartomány)
 * yaw: 0-360 degrees
 */
void Drone_SetTargetAngles(float pitch, float roll, float yaw)
{
    /* Limit angles to reasonable values */
    if (pitch > 45.0f) pitch = 45.0f;
    if (pitch < -45.0f) pitch = -45.0f;
    if (roll > 45.0f) roll = 45.0f;
    if (roll < -45.0f) roll = -45.0f;
    
    drone.target_angles.pitch = pitch;
    drone.target_angles.roll = roll;
    drone.target_angles.yaw = yaw;
}

/**
 * Get motor PWM values for ESC control
 */
void Drone_GetMotorOutputs(Motor_Output_t *output)
{
    if (output) {
        *output = drone.motor_output;
    }
}

/**
 * Enable/disable stabilization
 * 0 = disabled (manual), 1 = enabled (auto-level)
 */
void Drone_SetStabilization(uint8_t enabled)
{
    drone.stabilization_enabled = enabled;
}

/**
 * Usage example in main.c:
 * 
 * int main(void) {
 *     // ... Hardware init ...
 *     
 *     Drone_ControllerInit();
 *     
 *     while(1) {
 *         // Update control every 10ms (100 Hz)
 *         if (HAL_GetTick() - last_ctrl_time >= 10) {
 *             last_ctrl_time = HAL_GetTick();
 *             
 *             // Set target angles (hover mode: 0,0,yaw)
 *             Drone_SetTargetAngles(0.0f, 0.0f, current_yaw);
 *             
 *             // Get throttle from joystick (0.0-1.0)
 *             float throttle = joystick_throttle();
 *             
 *             // Update stabilization (must call every 10ms)
 *             Drone_UpdateStabilization(throttle, 0.01f);
 *             
 *             // Get motor outputs
 *             Motor_Output_t motors;
 *             Drone_GetMotorOutputs(&motors);
 *             
 *             // Send to ESC PWM (convert PWM value)
 *             Set_ESC1_PWM((uint16_t)motors.motor1);
 *             Set_ESC2_PWM((uint16_t)motors.motor2);
 *             Set_ESC3_PWM((uint16_t)motors.motor3);
 *             Set_ESC4_PWM((uint16_t)motors.motor4);
 *         }
 *     }
 * }
 */
