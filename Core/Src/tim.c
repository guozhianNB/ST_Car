#include "tim.h"

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim15;

static void EncoderTimer_Init(TIM_HandleTypeDef *htim, TIM_TypeDef *instance,
                              uint32_t period)
{
  TIM_Encoder_InitTypeDef encoder = {0};
  TIM_MasterConfigTypeDef master = {0};

  htim->Instance = instance;
  htim->Init.Prescaler = 0;
  htim->Init.CounterMode = TIM_COUNTERMODE_UP;
  htim->Init.Period = period;
  htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  encoder.EncoderMode = TIM_ENCODERMODE_TI12;
  encoder.IC1Polarity = TIM_ICPOLARITY_RISING;
  encoder.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  encoder.IC1Prescaler = TIM_ICPSC_DIV1;
  encoder.IC1Filter = 4;
  encoder.IC2Polarity = TIM_ICPOLARITY_RISING;
  encoder.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  encoder.IC2Prescaler = TIM_ICPSC_DIV1;
  encoder.IC2Filter = 4;
  if (HAL_TIM_Encoder_Init(htim, &encoder) != HAL_OK) {
    Error_Handler();
  }
  master.MasterOutputTrigger = TIM_TRGO_RESET;
  master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(htim, &master) != HAL_OK) {
    Error_Handler();
  }
}

void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef clock = {0};
  TIM_MasterConfigTypeDef master = {0};
  TIM_OC_InitTypeDef pwm = {0};
  TIM_BreakDeadTimeConfigTypeDef break_dead_time = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 8499; /* 170 MHz / 8500 = 20 kHz. */
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK) Error_Handler();
  clock.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &clock) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) Error_Handler();
  master.MasterOutputTrigger = TIM_TRGO_RESET;
  master.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &master) != HAL_OK) Error_Handler();

  pwm.OCMode = TIM_OCMODE_PWM1;
  pwm.Pulse = 0;
  pwm.OCPolarity = TIM_OCPOLARITY_HIGH;
  pwm.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  pwm.OCFastMode = TIM_OCFAST_DISABLE;
  pwm.OCIdleState = TIM_OCIDLESTATE_RESET;
  pwm.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &pwm, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &pwm, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &pwm, TIM_CHANNEL_3) != HAL_OK) Error_Handler();

  break_dead_time.OffStateRunMode = TIM_OSSR_DISABLE;
  break_dead_time.OffStateIDLEMode = TIM_OSSI_DISABLE;
  break_dead_time.LockLevel = TIM_LOCKLEVEL_OFF;
  break_dead_time.DeadTime = 0;
  break_dead_time.BreakState = TIM_BREAK_DISABLE;
  break_dead_time.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  break_dead_time.BreakFilter = 0;
  break_dead_time.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  break_dead_time.Break2State = TIM_BREAK2_DISABLE;
  break_dead_time.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  break_dead_time.Break2Filter = 0;
  break_dead_time.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  break_dead_time.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &break_dead_time) != HAL_OK) Error_Handler();
  HAL_TIM_MspPostInit(&htim1);
}

void MX_TIM2_Init(void)
{
  EncoderTimer_Init(&htim2, TIM2, 0xFFFFFFFFUL);
}

void MX_TIM3_Init(void)
{
  EncoderTimer_Init(&htim3, TIM3, 0xFFFFU);
}

void MX_TIM4_Init(void)
{
  EncoderTimer_Init(&htim4, TIM4, 0xFFFFU);
}

void MX_TIM15_Init(void)
{
  TIM_SlaveConfigTypeDef slave = {0};
  TIM_IC_InitTypeDef input = {0};
  TIM_MasterConfigTypeDef master = {0};

  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 169; /* 1 us capture tick at 170 MHz. */
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 0xFFFFU;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim15) != HAL_OK) Error_Handler();

  slave.SlaveMode = TIM_SLAVEMODE_RESET;
  slave.InputTrigger = TIM_TS_TI1FP1;
  slave.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  slave.TriggerPrescaler = TIM_ICPSC_DIV1;
  slave.TriggerFilter = 2;
  if (HAL_TIM_SlaveConfigSynchro(&htim15, &slave) != HAL_OK) Error_Handler();

  input.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  input.ICSelection = TIM_ICSELECTION_DIRECTTI;
  input.ICPrescaler = TIM_ICPSC_DIV1;
  input.ICFilter = 2;
  if (HAL_TIM_IC_ConfigChannel(&htim15, &input, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
  input.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  input.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim15, &input, TIM_CHANNEL_2) != HAL_OK) Error_Handler();

  master.MasterOutputTrigger = TIM_TRGO_RESET;
  master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &master) != HAL_OK) Error_Handler();
}

void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *htim)
{
  GPIO_InitTypeDef gpio = {0};

  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  if (htim->Instance == TIM2) {
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    gpio.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(GPIOA, &gpio);
  } else if (htim->Instance == TIM3) {
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOA, &gpio);
  } else if (htim->Instance == TIM4) {
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &gpio);
  }
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1) __HAL_RCC_TIM1_CLK_ENABLE();
}

void HAL_TIM_IC_MspInit(TIM_HandleTypeDef *htim)
{
  GPIO_InitTypeDef gpio = {0};
  if (htim->Instance == TIM15) {
    __HAL_RCC_TIM15_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_14;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF1_TIM15;
    HAL_GPIO_Init(GPIOB, &gpio);
    HAL_NVIC_SetPriority(TIM1_BRK_TIM15_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn);
  }
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim)
{
  GPIO_InitTypeDef gpio = {0};
  if (htim->Instance == TIM1) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF6_TIM1;
    HAL_GPIO_Init(GPIOA, &gpio);
  }
}
