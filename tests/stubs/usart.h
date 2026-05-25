#ifndef TEST_STUB_USART_H
#define TEST_STUB_USART_H

#include "main.h"

extern UART_HandleTypeDef huart3;

int HAL_UART_DeInit(UART_HandleTypeDef *huart);
int HAL_UART_Init(UART_HandleTypeDef *huart);

#endif
