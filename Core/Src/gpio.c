#include "gpio.h"
#include "app_config.h"

void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();

  /* Safe boot: wheel bridge, stepper DIR/EN and retired controls are low. */
  HAL_GPIO_WritePin(GPIOC, MOTOR_L_IN1_Pin | MOTOR_L_IN2_Pin |
                           MOTOR_R_IN1_Pin | MOTOR_R_IN2_Pin |
                           STEPPER_DIR_Pin,
                    GPIO_PIN_RESET);
  HAL_GPIO_WritePin(STEPPER_EN_GPIO_Port, STEPPER_EN_Pin,
                    APP_STEPPER_ENABLE_ACTIVE_HIGH ? GPIO_PIN_RESET
                                                   : GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, CHASSIS_STBY_Pin | LEGACY_BEAM_STBY_Pin,
                    GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LEGACY_BEAM_PWM_GPIO_Port, LEGACY_BEAM_PWM_Pin,
                    GPIO_PIN_RESET);
  HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);

  gpio.Pin = MOTOR_L_IN1_Pin | MOTOR_L_IN2_Pin |
             MOTOR_R_IN1_Pin | MOTOR_R_IN2_Pin |
             STEPPER_DIR_Pin | STEPPER_EN_Pin;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &gpio);

  gpio.Pin = CHASSIS_STBY_Pin | LEGACY_BEAM_STBY_Pin;
  HAL_GPIO_Init(GPIOB, &gpio);

  gpio.Pin = LEGACY_BEAM_PWM_Pin;
  HAL_GPIO_Init(LEGACY_BEAM_PWM_GPIO_Port, &gpio);

  gpio.Pin = STATUS_LED_Pin;
  HAL_GPIO_Init(STATUS_LED_GPIO_Port, &gpio);

  /* Digital line sensors. Active level is normalized in line_sensor.c. */
  gpio.Pin = LINE_0_Pin | LINE_1_Pin | LINE_2_Pin | LINE_3_Pin | LINE_4_Pin;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &gpio);
  gpio.Pin = LINE_5_Pin | LINE_6_Pin | LINE_7_Pin;
  HAL_GPIO_Init(GPIOC, &gpio);

  gpio.Pin = LEVEL_UP_BUTTON_Pin | LEVEL_DOWN_BUTTON_Pin;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &gpio);

  gpio.Pin = START_BUTTON_Pin;
  gpio.Mode = GPIO_MODE_IT_RISING;
  gpio.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(START_BUTTON_GPIO_Port, &gpio);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}
