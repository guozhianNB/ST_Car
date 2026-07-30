#include "gpio.h"

void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /* Safe boot: all bridge inputs, PWM compare values and STBY pins are low. */
  HAL_GPIO_WritePin(GPIOC, MOTOR_L_IN1_Pin | MOTOR_L_IN2_Pin |
                           MOTOR_R_IN1_Pin | MOTOR_R_IN2_Pin |
                           MOTOR_BEAM_IN1_Pin | MOTOR_BEAM_IN2_Pin,
                    GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, CHASSIS_STBY_Pin | BEAM_STBY_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);

  gpio.Pin = MOTOR_L_IN1_Pin | MOTOR_L_IN2_Pin |
             MOTOR_R_IN1_Pin | MOTOR_R_IN2_Pin |
             MOTOR_BEAM_IN1_Pin | MOTOR_BEAM_IN2_Pin;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &gpio);

  gpio.Pin = CHASSIS_STBY_Pin | BEAM_STBY_Pin;
  HAL_GPIO_Init(GPIOB, &gpio);

  gpio.Pin = STATUS_LED_Pin;
  HAL_GPIO_Init(STATUS_LED_GPIO_Port, &gpio);

  /* Digital line sensors. Active level is normalized in line_sensor.c. */
  gpio.Pin = LINE_0_Pin | LINE_1_Pin | LINE_2_Pin | LINE_3_Pin | LINE_4_Pin;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &gpio);
  gpio.Pin = LINE_5_Pin | LINE_6_Pin | LINE_7_Pin;
  HAL_GPIO_Init(GPIOC, &gpio);

  gpio.Pin = MODE_BUTTON_Pin;
  gpio.Mode = GPIO_MODE_IT_FALLING;
  gpio.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(MODE_BUTTON_GPIO_Port, &gpio);

  gpio.Pin = START_BUTTON_Pin;
  gpio.Mode = GPIO_MODE_IT_RISING;
  gpio.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(START_BUTTON_GPIO_Port, &gpio);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}
