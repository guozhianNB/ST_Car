#include "main.h"
#include "app.h"
#include "dma.h"
#include "gpio.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"

void SystemClock_Config(void);

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM15_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_LPUART1_UART_Init();
  MX_UART4_Init();

  App_Init();
  //__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 4250);
  while (1) {
    App_Run();
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef oscillator = {0};
  RCC_ClkInitTypeDef clock = {0};

  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  oscillator.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  oscillator.HSIState = RCC_HSI_ON;
  oscillator.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  oscillator.PLL.PLLState = RCC_PLL_ON;
  oscillator.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  oscillator.PLL.PLLM = RCC_PLLM_DIV4;
  oscillator.PLL.PLLN = 85;
  oscillator.PLL.PLLP = RCC_PLLP_DIV2;
  oscillator.PLL.PLLQ = RCC_PLLQ_DIV2;
  oscillator.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&oscillator) != HAL_OK) Error_Handler();

  clock.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                    RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clock.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clock.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clock.APB1CLKDivider = RCC_HCLK_DIV1;
  clock.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&clock, FLASH_LATENCY_4) != HAL_OK) Error_Handler();
}

void Error_Handler(void)
{
  __disable_irq();
  HAL_GPIO_WritePin(CHASSIS_STBY_GPIO_Port, CHASSIS_STBY_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BEAM_STBY_GPIO_Port, BEAM_STBY_Pin, GPIO_PIN_RESET);
  while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif
