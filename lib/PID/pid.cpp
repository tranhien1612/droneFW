#include "pid.h"


void PID_Calc(PID_Struct *pid){
    // 1. Target value - Measured value => Error value
    pid->err = pid->measure - pid->desire;

    // 2. Calculate the integral term
    pid->integral += pid->err;

    if (pid->last_err == 0)
    {
        pid->last_err = pid->err;
    }

    // 3. Calculate the differential term
    float der = pid->err - pid->last_err;

    // 4. Calculate the output
    pid->output = pid->kp * pid->err + (pid->ki * pid->integral * PID_PERIOD) + (pid->kd * der / PID_PERIOD);

    // 5. Update the previous error
    pid->last_err = pid->err;
}

void PID_Calc_Chain(PID_Struct *out_pid, PID_Struct *in_pid){
    // 1. First calculate the outer ring
    PID_Calc(out_pid);
    // 2. Use the output value of the outer loop as the target value of the inner loop.
    in_pid->desire = out_pid->output;
    // 3. Calculate the inner loop
    PID_Calc(in_pid);
}

int16_t limit(int16_t speed, int16_t max_speed, int16_t min_speed)
{
    if (speed > max_speed)
    {
        return max_speed;
    }
    else if (speed < min_speed)
    {
        return min_speed;
    }
    return speed;
}