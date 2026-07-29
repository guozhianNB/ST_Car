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
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "yaw.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* 控制频率（主循环 PID 更新间隔） */
#define CONTROL_INTERVAL_MS  20

/* 默认目标速度 */
#define STRAIGHT_SPEED                500
#define TARGET_DISTANCE_CM             100.0f
#define ENCODER_COUNTS_PER_REV         780.0f
#define WHEEL_CIRCUMFERENCE_CM         20.73f
#define DISTANCE_TOLERANCE_CM          1.0f
#define DISTANCE_CONFIRM_COUNT         5U
#define YAW_KP                         18.0f
#define YAW_OUTPUT_LIMIT               200
#define DRIVE_TIMEOUT_MS               15000U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

COM_InitTypeDef BspCOMInit;

/* USER CODE BEGIN PV */

/* 调试用变量（可在调试器中查看） */
static int g_motorL = 0;
static int g_motorR = 0;
static float g_distance_cm = 0.0f;
static float g_yaw_deg = 0.0f;
static uint8_t g_drive_finished = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static int ClampMotorCommand(int value)
{
  if (value > 1000) return 1000;
  if (value < -1000) return -1000;
  return value;
}

static int ClampYawCorrection(float value)
{
  if (value > (float)YAW_OUTPUT_LIMIT) return YAW_OUTPUT_LIMIT;
  if (value < -(float)YAW_OUTPUT_LIMIT) return -YAW_OUTPUT_LIMIT;
  return (int)value;
}

static void Car_DriveStraightCm(float target_cm)
{
  uint16_t last_count_l = (uint16_t)Motor_GetEncoderCountL();
  uint16_t last_count_r = (uint16_t)Motor_GetEncoderCountR();
  uint32_t last_control_tick = HAL_GetTick();
  uint32_t start_tick = last_control_tick;
  uint8_t reached_count = 0U;

  g_distance_cm = 0.0f;
  while (reached_count < DISTANCE_CONFIRM_COUNT)
  {
    uint32_t now_tick = HAL_GetTick();
    if ((now_tick - start_tick) >= DRIVE_TIMEOUT_MS) break;
    if ((now_tick - last_control_tick) < CONTROL_INTERVAL_MS) continue;
    last_control_tick = now_tick;

    {
      uint16_t count_l = (uint16_t)Motor_GetEncoderCountL();
      uint16_t count_r = (uint16_t)Motor_GetEncoderCountR();
      int16_t delta_l = (int16_t)(count_l - last_count_l);
      int16_t delta_r = (int16_t)(count_r - last_count_r);
      int yaw_correction;

      last_count_l = count_l;
      last_count_r = count_r;
      g_distance_cm += ((float)(delta_l + delta_r) * 0.5f)
                       * WHEEL_CIRCUMFERENCE_CM / ENCODER_COUNTS_PER_REV;
      g_yaw_deg = GetYaw();
      yaw_correction = ClampYawCorrection(YAW_KP * g_yaw_deg);

      g_motorL = STRAIGHT_SPEED;
      g_motorR = STRAIGHT_SPEED;
      Motor_SetSpeed(ClampMotorCommand(g_motorL - yaw_correction),
                     ClampMotorCommand(g_motorR + yaw_correction));

      if ((target_cm - g_distance_cm) <= DISTANCE_TOLERANCE_CM)
        ++reached_count;
      else
        reached_count = 0U;
    }
  }

  Motor_SetSpeed(0, 0);
  g_drive_finished = 1U;
}

/*
 * 注：PID 控制器实现在 pid_control.c 中。
 *    main.c 只需在主循环中调用：
 *      PID_Control_SetTarget(target_speed, track_error, &motorL, &motorR);
 *      Motor_SetSpeed(motorL, motorR);
 */

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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  /* 初始化电机（PWM + 编码器） */
  Motor_Init();

  /* 初始化 PID 控制器 */

  if (!Yaw_Init())
  {
    Motor_SetSpeed(0, 0);
    Error_Handler();
  }

  /* 指示系统就绪（LED亮） */
  BSP_LED_Init(LED_GREEN);
  BSP_LED_On(LED_GREEN);
  /* USER CODE END 2 */

  /* Initialize led */
  BSP_LED_Init(LED_GREEN);

  /* Initialize USER push-button, will be used to trigger an interrupt each time it's pressed.*/
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

  /* Initialize COM1 port (115200, 8 bits (7-bit data + 1 stop bit), no parity */
  BspCOMInit.BaudRate   = 115200;
  BspCOMInit.WordLength = COM_WORDLENGTH_8B;
  BspCOMInit.StopBits   = COM_STOPBITS_1;
  BspCOMInit.Parity     = COM_PARITY_NONE;
  BspCOMInit.HwFlowCtl  = COM_HWCONTROL_NONE;
  if (BSP_COM_Init(COM1, &BspCOMInit) != BSP_ERROR_NONE)
  {
    Error_Handler();
  }

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  Car_DriveStraightCm(TARGET_DISTANCE_CM);
  uint32_t last_tick = HAL_GetTick();

  while (1)
  {
    if (g_drive_finished != 0U)
    {
      Motor_SetSpeed(0, 0);
      HAL_Delay(100U);
      continue;
    }
    uint32_t now_tick = HAL_GetTick();
    int dt_ms = now_tick - last_tick;

    /* 固定频率控制：每隔 CONTROL_INTERVAL_MS 执行一次 PID 更新 */
    if (dt_ms >= CONTROL_INTERVAL_MS)
    {
      last_tick = now_tick;

      /*
       * 获取寻迹误差（后续接入红外传感器接口）
       * 目前填 0，表示"没有偏离"，车按编码器直线修正走直
       */

      /* 调用 PID 控制器，计算左右轮速度 */
      g_motorL = STRAIGHT_SPEED;
      g_motorR = STRAIGHT_SPEED;

      /* 驱动电机 */
      Motor_SetSpeed(g_motorL, g_motorR);
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /*
     * 此处可添加其他非实时任务，例如：
     *   - 读取红外传感器 → 更新 g_track_error
     *   - 按键检测 → 启停控制
     *   - OLED 显示调试信息
     */
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
