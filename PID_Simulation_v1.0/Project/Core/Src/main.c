/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor_simulation.h"
#include "bsp_dwt.h"
#include "PID.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
motorObject_t Motor;

PID_t AnglePID;
PID_t VelocityPID;

float dt = 0.0f;
float t  = 0.0f;

float Current  = 0.0f;
float Velocity = 0.0f;
float Angle    = 0.0f;

float Input       = 0.0f;
float AngleRef    = 0.0f;
float VelocityRef = 0.0f;

uint32_t DWT_CNT = 0;

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

TestMode_t TestMode = OPEN_LOOP;
DISTURBANCE_ControlMode_t DisturbanceControlMode = MODE_ANG_CAS;

float VelocityRaw = 0.0f;
float VelocityFiltered = 0.0f;
float VelFilterAlpha = 0.3f;   


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Reset_System(void)
{
  DWT_CNT = DWT->CYCCNT; 
  
  PID_Reset(&AnglePID);
  PID_Reset(&VelocityPID);
  Motor_Object_Init(&Motor);

  Input = 0.0f;
  VelocityRef = 0.0f;
  AngleRef = 0.0f;
  VelocityFiltered = 0.0f;


  t = 0.0f;
}
void Generate_Reference(float t)
{
  float omega_vel = 36.0f;
  float omega_ang = 4.8f;
  float omega_cas = 5.5f;

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

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */

  DWT_Init(72);//DWT 定时器初始化
  Motor_Object_Init(&Motor);//电机初始化

  // ================= 内环：速度环 =================
  PID_Init(&VelocityPID,
          5.0f,      // Kp
          40.0f,     // Ki
          0.01f,     // Kd
          -24.0f,     // min_output
          24.0f);    // max_output

  // ================= 外环：角度环 =================
  PID_Init(&AnglePID,
          30.0f,      // Kp
          0.0f,      // Ki
          0.27f,      // Kd
          -20.0f,     // min velocity ref
          20.0f);    // max velocity ref

  Reset_System();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if(isnan(Input) || isnan(Velocity) || isnan(Angle))
    {
    Reset_System();
    }

    dt = DWT_GetDeltaT(&DWT_CNT);

    if(dt > 0.01f)   // 超过10ms认为异常
    {
        dt = 0.001f;
    }

    t += dt;

    // ===== 读取状态 =====
    Current  = Get_Motor_Current(&Motor);

    VelocityRaw = Get_Motor_Velocity(&Motor);
   // 一阶低通滤波
    VelocityFiltered = (1.0f - VelFilterAlpha) * VelocityFiltered
                      + VelFilterAlpha * VelocityRaw;

    Velocity = VelocityFiltered;

    Angle    = Get_Motor_Angle(&Motor);

    Generate_Reference(t);

    // ===== 控制器结构选择 =====
    switch(TestMode)
    {
        // ========= 速度闭环 =========
        case VEL_STEP:
        case VEL_RAMP:
        case VEL_SINE:

            Input = PID_Calculate(&VelocityPID,
                                  VelocityRef,
                                  Velocity,
                                  dt);
            break;

        // ========= 单级角度 =========
        case ANG_SINGLE_STEP:
        case ANG_SINGLE_SINE:

            Input = PID_Calculate(&AnglePID,
                                  AngleRef,
                                  Angle,
                                  dt);
            break;

        // ========= 串级控制 =========
        case ANG_CAS_STEP:
        case ANG_CAS_SINE:
        case ANG_DISTURBANCE:

            VelocityRef = PID_Calculate(&AnglePID,
                                        AngleRef,
                                        Angle,
                                        dt);

            Input = PID_Calculate(&VelocityPID,
                                  VelocityRef,
                                  Velocity,
                                  dt);
            break;
        case OPEN_LOOP:
            Input = 24.0f;
            break;
    }

    // ===== 扰动测试 =====
    if(TestMode == ANG_DISTURBANCE)
    {
      
      switch (DisturbanceControlMode)
      {
        case MODE_ANG_SINGLE:

           Input = PID_Calculate(&AnglePID,
                                  AngleRef,
                                  Angle,
                                  dt);
            break;
        
        case MODE_ANG_CAS:
            VelocityRef = PID_Calculate(&AnglePID,
                                         AngleRef,
                                         Angle,
                                         dt);

            Input = PID_Calculate(&VelocityPID,
                                   VelocityRef,
                                   Velocity,
                                   dt);
            break;
      }

      if(t > 0.5f && t < 0.7f)
            Input += 10.0f;
        
    }

    // ===== 电机仿真 =====
    Motor_Simulation(&Motor, Input, dt);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    HAL_Delay(1);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
