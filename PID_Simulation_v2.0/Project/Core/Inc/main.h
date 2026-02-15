/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern float Current;
extern float Velocity;
extern float Angle;

extern float Input;
extern float AngleRef;
extern float VelocityRef;

extern float VelRef_last;
extern float VelRef_last2;

extern float MaxAcc;  
extern float MaxJerk; 

typedef enum
{
    // ===== 速度环 =====
    VEL_STEP,
    VEL_RAMP,
    VEL_SINE,

    // ===== 角度单环 =====
    ANG_SINGLE_STEP,
    ANG_SINGLE_SINE,

    // ===== 角度串级 =====
    ANG_CAS_STEP,
    ANG_CAS_SINE,

    // ===== 扰动测试 =====
    ANG_DISTURBANCE,

    // ===== 开环 =====
    
    OPEN_LOOP

} TestMode_t;

typedef enum
{
    MODE_ANG_SINGLE,
    MODE_ANG_CAS,
} DISTURBANCE_ControlMode_t;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern TestMode_t TestMode;
extern DISTURBANCE_ControlMode_t DisturbanceControlMode;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
