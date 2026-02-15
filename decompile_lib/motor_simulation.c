#include "motor_simulation.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define OMEGA_THRESH    0.002f                   // 速度阈值（对应 0x3b03126e）

// 四舍五入取整（对应 float_rounding）
static int float_rounding(float x) {
    int i = (int)x;               // 向零取整
    float frac = x - (float)i;
    if (frac > 0.5f) {
        i += 1;
    } else if (frac < -0.5f) {
        i -= 1;
    }
    return i;
}

// 高斯随机数生成器（Box-Muller，对应 Gauss_Rand）
static int phase = 0;
static float U, V;

static float Gauss_Rand(void) {
    float result;
    if (phase == 0) {
        U = (float)rand() / (RAND_MAX + 1.0f);
        V = (float)rand() / (RAND_MAX + 1.0f);
        result = sqrtf(-2.0f * logf(V)) * sinf(2.0f * M_PI * U);
        phase = 1;
    } else {
        result = sqrtf(-2.0f * logf(V)) * cosf(2.0f * M_PI * U);
        phase = 0;
    }
    return result;
}

// 电机对象初始化
void Motor_Object_Init(motorObject_t *motor) {
    // 电机参数
    motor->motorParam.J              = 0.047f;    // 0x3d408312
    motor->motorParam.b              = 0.03f;     // 0x3cf5c28f
    motor->motorParam.Kt             = 0.741f;    // 0x3f3db22d
    motor->motorParam.Ke             = 0.7164f;   // 0x3f3765fe
    motor->motorParam.R              = 1.8f;      // 0x3fe66666
    motor->motorParam.L              = 0.00578f;  // 0x3bbd6627
    motor->motorParam.constFriction  = 0.5f;      // 0x3f000000

    // 限幅电压
    motor->maxU = 24.0f;                          // 0x41c00000

    // 状态变量清零
    motor->U = 0.0f;
    motor->I = 0.0f;
    motor->Velocity = 0.0f;
    motor->Angle = 0.0f;
    motor->dI = 0.0f;
    motor->dV = 0.0f;
    motor->lastdI = 0.0f;
    motor->lastdV = 0.0f;
    motor->lastVelocity = 0.0f;
    motor->MeasureI = 0.0f;
    motor->MeasureVelocity = 0.0f;
    motor->MeasureAngle = 0.0f;
}

// 电机仿真（对应 Motor_Simulation）
void Motor_Simulation(motorObject_t *motor, float u, float dt) {
    // 1. 电压限幅
    float u_sat = u;
    if (u_sat > motor->maxU)
        u_sat = motor->maxU;
    else if (u_sat < -motor->maxU)
        u_sat = -motor->maxU;
    motor->U = u_sat;   // 保存限幅后的电压（可选）

    // 2. 计算电流变化率 di/dt = (u - Ke*ω - R*I) / L
    float I_old = motor->I;
    float omega_old = motor->Velocity;
    float di_dt = (u_sat - motor->motorParam.Ke * omega_old - motor->motorParam.R * I_old) / motor->motorParam.L;
    motor->dI = di_dt;

    // 3. 梯形法更新电流：I_new = I_old + (dI + lastdI) * dt/2
    float I_new = I_old + (di_dt + motor->lastdI) * dt * 0.5f;
    motor->I = I_new;

    // 4. 计算角加速度 dω/dt
    float omega_abs = fabsf(omega_old);
    float Kt_i = motor->motorParam.Kt * I_new;
    float domega_dt;

    if (omega_abs < OMEGA_THRESH) {
        // 静摩擦区：如果转矩小于静摩擦，则角加速度为0
        if (fabsf(Kt_i) < motor->motorParam.constFriction) {
            domega_dt = 0.0f;
        } else {
            // 否则按动摩擦计算（此时速度很小，但仍用符号）
            float sign = (omega_old > 0.0f) ? 1.0f : -1.0f;
            domega_dt = (Kt_i - motor->motorParam.b * omega_old - motor->motorParam.constFriction * sign) / motor->motorParam.J;
        }
    } else {
        // 正常动摩擦区
        float sign = (omega_old > 0.0f) ? 1.0f : -1.0f;
        domega_dt = (Kt_i - motor->motorParam.b * omega_old - motor->motorParam.constFriction * sign) / motor->motorParam.J;
    }
    motor->dV = domega_dt;

    // 5. 梯形法更新角速度：Velocity_new = omega_old + (dV + lastdV) * dt/2
    float omega_new = omega_old + (domega_dt + motor->lastdV) * dt * 0.5f;
    motor->Velocity = omega_new;

    // 6. 梯形法更新角度：Angle_new = Angle_old + (Velocity_new + lastVelocity) * dt/2
    float angle_new = motor->Angle + (omega_new + motor->lastVelocity) * dt * 0.5f;
    motor->Angle = angle_new;

    // 7. 保存上一时刻值
    motor->lastVelocity = omega_new;
    motor->lastdI = di_dt;
    motor->lastdV = domega_dt;

    // 8. 输出电流量化（加噪声后量化）
    float noise = Gauss_Rand() * 0.3f;          // 噪声标准差 0.3
    float I_noisy = I_new + noise;
    // 量化：除以 MAX_I，乘 RESOLUTION_I，四舍五入，再还原
    float I_quantized = (float)float_rounding(I_noisy / MAX_I * RESOLUTION_I) / RESOLUTION_I * MAX_I;
    motor->MeasureI = I_quantized;

    // 9. 角度量化，并计算差分角速度
    float theta_norm = angle_new / MAX_ANGLE;
    float theta_quantized = (float)float_rounding(theta_norm * RESOLUTION_ANGLE) / RESOLUTION_ANGLE * MAX_ANGLE;
    motor->MeasureVelocity = (theta_quantized - motor->MeasureAngle) / dt;   // 差分角速度
    motor->MeasureAngle = theta_quantized;                                   // 保存本次量化角度
}

// 获取电机输出速度
float Get_Motor_Velocity(motorObject_t *motor) {
    return motor->MeasureVelocity;
}

// 获取电机输出电流
float Get_Motor_Current(motorObject_t *motor) {
    return motor->MeasureI;
}

// 获取电机输出角度（量化后的角度）
float Get_Motor_Angle(motorObject_t *motor) {
    return motor->MeasureAngle;
}