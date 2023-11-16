#ifndef _SERIAL_DEBUG_H
#define _SERIAL_DEBUG_H

#include "usart.h"
#include "stdint.h"
#include <stdarg.h>

#define DEBUG_BUFFER_LEN 200

void Serial_Debug_Init(UART_HandleTypeDef *huart);
void Serial_Debug(UART_HandleTypeDef *huart, uint16_t debug_period, float a, float b, float c, float d, float e, float f);
void Serial_Debug_Indeterminate_Length(uint8_t num, ...);
#endif
