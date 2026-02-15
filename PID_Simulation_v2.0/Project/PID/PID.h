#ifndef __PID_H
#define __PID_H

#include "stdint.h"
#include "math.h"

typedef struct
{
    float Kp;
    float Ki;
    float Kd;

    float integral;
    float last_error;
    float last_fdb;

    float max_output;
    float min_output;
    
    float delta_time;
    float output;
} PID_t;

void PID_Init(PID_t *pid, float Kp, float Ki, float Kd, float min_output, float max_output);
float PID_Calculate(PID_t *pid, float ref, float fdb, float dt);
void PID_Reset(PID_t *pid);

#endif /* __PID_H */
