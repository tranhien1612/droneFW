#ifndef ___PID_H
#define ___PID_H

#include <stdint.h>

#define PID_PERIOD 0.01f

// PID structure. If the CPU performance is powerful enough, it is recommended to use double.
// kp, ki, kd need to be determined during initialization. 
// Target values ​​and measured values ​​need to be assigned during calculation.
typedef struct{
    float kp; // Proportional coefficient. A larger value results in a faster response.
    float ki; // Integral coefficient. Eliminates steady-state error and prevents integral saturation. Generally not used.
    float kd; // Differential coefficient. A larger value results in stronger damping and suppresses overshoot.
    float err; // Error value
    float desire; // Target value
    float measure; // Measured value
    float last_err; // Previous error
    float integral; // Accumulated error
    float output; // Output value
} PID_Struct;

/**
 * @brief Single PID calculation
 *
 * This function calculates the PID output for a single control loop.
 * It updates the error, integral term, differential term,
 * and computes the final PID output value.
 *
 * Formula:
 * output = kp * error + ki * integral + kd * derivative
 *
 * @param pid Pointer to PID structure
 */
void PID_Calc(PID_Struct *pid);

/**
 * @brief Cascaded PID calculation
 *
 * This function performs a cascaded PID control calculation.
 * The outer PID loop is calculated first, then its output
 * is used as the target value of the inner PID loop.
 *
 * Commonly used in systems with position-speed or angle-speed control.
 *
 * @param out_pid Pointer to outer loop PID structure
 * @param in_pid Pointer to inner loop PID structure
 */
void PID_Calc_Chain(PID_Struct *out_pid, PID_Struct *in_pid);

/**
 * @brief Limit values ​​are within the specified range
 *
 * @param speed
 * @param max_speed
 * @param min_speed
 * @return int16_t
 */
int16_t limit(int16_t speed, int16_t max_speed, int16_t min_speed);

#endif