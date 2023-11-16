#ifndef __BSP_USART_IDLE_H
#define __BSP_USART_IDLE_H
/* Includes ------------------------------------------------------------------*/
#include "usart.h"
#include "stdint.h"
#include "stdlib.h"
#include "string.h"

void USART_IDLE_Init(UART_HandleTypeDef *huart, uint8_t *rx_buf, uint16_t dma_buf_num);
void USART_IDLE_IRQHandler(UART_HandleTypeDef *huart);
void RC_Restart(uint16_t dma_buf_num);

__weak void USER_UART_RxIdleCallback(UART_HandleTypeDef *huart);

#endif
