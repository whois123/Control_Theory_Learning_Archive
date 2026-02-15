#ifndef MOTOR_FEEDFORWARD_H
#define MOTOR_FEEDFORWARD_H

#include "motor_simulation.h"

float Motor_FeedForward(
    motorObject_t *motor,
    float omega_ref,
    float omega_ref_last,
    float omega_ref_last2,
    float dt,
    float maxAcc,
    float maxJerk);

#endif
