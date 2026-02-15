#include "Generate_Reference.h"

void Generate_Reference(float t)
{
  float omega_vel = 36.0f;
  float omega_ang = 4.8f;
  float omega_cas = 5.8f;

  switch(TestMode)
  {
    // ================= 速度环 =================
    case VEL_STEP:
      VelocityRef = 10.0f;
      break;

    case VEL_RAMP:
      VelocityRef = t;
      break;

    case VEL_SINE:
      VelocityRef = 10.0f * sinf(omega_vel * t);
      break;

    // ================= 角度单环 =================
    case ANG_SINGLE_STEP:
      AngleRef = 2.0f * 3.1415926f;
      break;

    case ANG_SINGLE_SINE:
      AngleRef = 2.0f * 3.1415926f * sinf(omega_ang * t);
      break;

    // ================= 串级 =================
    case ANG_CAS_STEP:
      AngleRef = 2.0f * 3.1415926f;
      break;

    case ANG_CAS_SINE:
      AngleRef = 2.0f * 3.1415926f * sinf(omega_cas * t);
      break;

    case ANG_DISTURBANCE:
      AngleRef = 0.0f;
      break;

    // ================= 开环 =================
    case OPEN_LOOP:
      // 无参考
      break;
  }
}
