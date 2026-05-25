/**
 * @file Encoder_MT6826S.h
 * @brief C adapter for MT6826S SPI magnetic encoder.
 */

#ifndef ENCODER_MT6826S_H
#define ENCODER_MT6826S_H

#include <stdint.h>
#include "Encoder.h"
#include "gpio.h"
#include "spi.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    GPIO_TypeDef *cs_gpio_port;
    uint16_t cs_gpio_pin;
    SPI_HandleTypeDef *hspi;
} Encoder_MT6826S_Context_t;

void Encoder_MT6826S_Bind(Encoder_t *encoder,
                          Encoder_MT6826S_Context_t *context,
                          GPIO_TypeDef *cs_gpio_port,
                          uint16_t cs_gpio_pin,
                          SPI_HandleTypeDef *hspi);

#ifdef __cplusplus
}
#endif

#endif
