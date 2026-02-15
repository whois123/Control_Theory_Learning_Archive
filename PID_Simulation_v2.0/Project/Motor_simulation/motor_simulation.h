#ifndef MOTOR_SIMULATION_H
#define MOTOR_SIMULATION_H
#include "stdint.h"
#include "stdlib.h"
#include "math.h"

#define MAX_I (24 / 1.8f)
#define MAX_ANGLE (2 * 3.1415926f)
#define RESOLUTION_I 8192
#define RESOLUTION_ANGLE 8192

typedef struct motorparam
{
    float J;
    float b;
    float Kt;
    float Ke;
    float R;
    float L;
    float constFriction;
} motorParam_t;

typedef struct motor
{
    float U;
    float I;
    float Velocity; // rad/s
    float Angle;    // rad

    float dI;
    float dV;
    float lastdI;
    float lastdV;
    float lastVelocity;

    float MeasureI;
    float MeasureVelocity; // rad/s
    float MeasureAngle;    // rad

    float maxU;

    motorParam_t motorParam;
} motorObject_t;

/**
 * @brief Init motor object
 * @param[in]       *motor points to the motorObject_t struct
 */
void Motor_Object_Init(motorObject_t *motor);

/**
 * @brief Motor simulation implementation
 * @param[in]       *motor points to the motorObject_t struct
 * @param[in]       input voltage
 * @param[in]       simulation period in s
 */
void Motor_Simulation(motorObject_t *motor, float input, float dt);

/**
 * @brief Get motor current
 * @param[in]       *motor points to the motorObject_t struct
 * @param[out]      motor current
 */
float Get_Motor_Current(motorObject_t *motor);

/**
 * @brief Get motor velocity
 * @param[in]       *motor points to the motorObject_t struct
 * @param[out]      motor velocity
 */
float Get_Motor_Velocity(motorObject_t *motor);

/**
 * @brief Get motor angle
 * @param[in]       *motor points to the motorObject_t struct
 * @param[out]      motor angle
 */
float Get_Motor_Angle(motorObject_t *motor);

#endif
