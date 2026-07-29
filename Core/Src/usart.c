#include "usart.h"

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_uart4_rx;

static void UART_CommonInit(UART_HandleTypeDef *huart)
{
  huart->Init.BaudRate = 115200;
  huart->Init.WordLength = UART_WORDLENGTH_8B;
  huart->Init.StopBits = UART_STOPBITS_1;
  huart->Init.Parity = UART_PARITY_NONE;
  huart->Init.Mode = UART_MODE_TX_RX;
  huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart->Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(huart) != HAL_OK) Error_Handler();
  if (HAL_UARTEx_SetTxFifoThreshold(huart, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) Error_Handler();
  if (HAL_UARTEx_SetRxFifoThreshold(huart, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) Error_Handler();
  if (HAL_UARTEx_DisableFifoMode(huart) != HAL_OK) Error_Handler();
}

void MX_LPUART1_UART_Init(void)
{
  hlpuart1.Instance = LPUART1;
  UART_CommonInit(&hlpuart1);
}

void MX_UART4_Init(void)
{
  huart4.Instance = UART4;
  UART_CommonInit(&huart4);
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
  GPIO_InitTypeDef gpio = {0};
  RCC_PeriphCLKInitTypeDef clock = {0};

  if (huart->Instance == LPUART1) {
    clock.PeriphClockSelection = RCC_PERIPHCLK_LPUART1;
    clock.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&clock) != HAL_OK) Error_Handler();
    __HAL_RCC_LPUART1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF12_LPUART1;
    HAL_GPIO_Init(GPIOA, &gpio);
    HAL_NVIC_SetPriority(LPUART1_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(LPUART1_IRQn);
  } else if (huart->Instance == UART4) {
    clock.PeriphClockSelection = RCC_PERIPHCLK_UART4;
    clock.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&clock) != HAL_OK) Error_Handler();
    __HAL_RCC_UART4_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_10 | GPIO_PIN_11;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF5_UART4;
    HAL_GPIO_Init(GPIOC, &gpio);

    hdma_uart4_rx.Instance = DMA1_Channel1;
    hdma_uart4_rx.Init.Request = DMA_REQUEST_UART4_RX;
    hdma_uart4_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_uart4_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_uart4_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_uart4_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_uart4_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_uart4_rx.Init.Mode = DMA_CIRCULAR;
    hdma_uart4_rx.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_uart4_rx) != HAL_OK) Error_Handler();
    __HAL_LINKDMA(huart, hdmarx, hdma_uart4_rx);

    HAL_NVIC_SetPriority(UART4_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(UART4_IRQn);
  }
}
