#include "i2c.h"

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

static void I2C_CommonInit(I2C_HandleTypeDef *hi2c)
{
  /* 400 kHz fast mode, 170 MHz I2C kernel clock, analog filter enabled. */
  hi2c->Init.Timing = 0x00F02B86;
  hi2c->Init.OwnAddress1 = 0;
  hi2c->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c->Init.OwnAddress2 = 0;
  hi2c->Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(hi2c) != HAL_OK) Error_Handler();
  if (HAL_I2CEx_ConfigAnalogFilter(hi2c, I2C_ANALOGFILTER_ENABLE) != HAL_OK) Error_Handler();
  if (HAL_I2CEx_ConfigDigitalFilter(hi2c, 0) != HAL_OK) Error_Handler();
}

void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  I2C_CommonInit(&hi2c1);
}

void MX_I2C3_Init(void)
{
  hi2c3.Instance = I2C3;
  I2C_CommonInit(&hi2c3);
}

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
  GPIO_InitTypeDef gpio = {0};
  RCC_PeriphCLKInitTypeDef clock = {0};

  gpio.Mode = GPIO_MODE_AF_OD;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  if (hi2c->Instance == I2C1) {
    clock.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
    clock.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&clock) != HAL_OK) Error_Handler();
    __HAL_RCC_I2C1_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    gpio.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &gpio);
    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 7, 0);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_SetPriority(I2C1_ER_IRQn, 7, 0);
    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
  } else if (hi2c->Instance == I2C3) {
    clock.PeriphClockSelection = RCC_PERIPHCLK_I2C3;
    clock.I2c3ClockSelection = RCC_I2C3CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&clock) != HAL_OK) Error_Handler();
    __HAL_RCC_I2C3_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    gpio.Alternate = GPIO_AF8_I2C3;
    HAL_GPIO_Init(GPIOC, &gpio);
  }
}
