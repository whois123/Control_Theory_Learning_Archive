#include "PID.h"

void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, float min_output, float max_output)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->min_output = min_output;
    pid->max_output = max_output;
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->last_fdb = 0.0f;
    pid->delta_time = 0.001f;  // 1ms
    pid->output = 0.0f;
}

float PID_Calculate(PID_t *pid, float ref, float fdb, float dt)
{
    if (dt <= 0.0f)
        dt = 1e-6f;

    if (dt > 0.01f)
        dt = 0.01f;

    float error = ref - fdb;

    float derivative = -(fdb - pid->last_fdb) / dt;

    float new_integral = pid->integral + error * dt;

    float output = pid->Kp * error
                 + pid->Ki * new_integral
                 + pid->Kd * derivative;

    if (output > pid->max_output)
    {
        output = pid->max_output;

        if (error < 0)
            pid->integral = new_integral;
    }
    else if (output < pid->min_output)
    {
        output = pid->min_output;

        if (error > 0)
            pid->integral = new_integral;
    }
    else
    {
        pid->integral = new_integral;
    }

    pid->last_error = error;
    pid->last_fdb   = fdb;
    pid->delta_time = dt;
    pid->output     = output;

    return output;
}

void PID_Reset(PID_t *pid)
{
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->last_fdb = 0.0f;
}
