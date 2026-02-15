#include "Motor_FeedForward.h"
#include <math.h>

static float clamp(float x, float min, float max)
{
    if (x > max) return max;
    if (x < min) return min;
    return x;
}

float Motor_FeedForward(
    motorObject_t *motor,
    float omega_ref,
    float omega_ref_last,
    float omega_ref_last2,
    float dt,
    float maxAcc,
    float maxJerk)
{
    if (dt <= 0.0f)
        dt = 1e-6f;

    // ===== 电机参数 =====
    float J  = motor->motorParam.J;
    float b  = motor->motorParam.b;
    float Kt = motor->motorParam.Kt;
    float Ke = motor->motorParam.Ke;
    float R  = motor->motorParam.R;
    float L  = motor->motorParam.L;
    float Tc = motor->motorParam.constFriction;

    // ===== 一阶导数 =====
    float domega = (omega_ref - omega_ref_last) / dt;
    domega = clamp(domega, -maxAcc, maxAcc);

    // ===== 二阶导数 =====
    float ddomega = (omega_ref - 2.0f*omega_ref_last + omega_ref_last2) / (dt * dt);
    ddomega = clamp(ddomega, -maxJerk, maxJerk);

    // ===== 平滑库仑摩擦 =====
    float sign = tanhf(omega_ref * 50.0f);

    // ===== 前馈电流 =====
    float I_ff =
        (J * domega
        + b * omega_ref
        + Tc * sign)
        / Kt;

    // ===== 电流导数 =====
    float dI_ff =
        (J * ddomega
        + b * domega)
        / Kt;

    // ===== 完整电压前馈（含 L 项）=====
    float u_ff =
        L * dI_ff
        + R * I_ff
        + Ke * omega_ref;

    return u_ff;
}
