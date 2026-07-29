#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern UART_HandleTypeDef hlpuart1;
extern UART_HandleTypeDef huart4;
extern DMA_HandleTypeDef hdma_uart4_rx;

void MX_LPUART1_UART_Init(void);
void MX_UART4_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */
