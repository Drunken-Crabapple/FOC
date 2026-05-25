#ifndef TEST_STUB_MAIN_H
#define TEST_STUB_MAIN_H

#include <stdint.h>

#define LED_G_GPIO_Port ((void *)0)
#define LED_G_Pin 0U
#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET 1
#define HAL_OK 0

typedef struct {
    uint32_t BaudRate;
} UART_InitTypeDef;

typedef struct {
    UART_InitTypeDef Init;
} UART_HandleTypeDef;

static inline void HAL_GPIO_WritePin(void *port, uint32_t pin, uint32_t state) {
    (void)port;
    (void)pin;
    (void)state;
}

static inline void Error_Handler(void) {
}

#endif
