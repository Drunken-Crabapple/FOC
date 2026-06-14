#include "Encoder_MT6826S.h"
#include "Encoder.h"
#include "stm32g4xx_hal_def.h"
#include "stm32g4xx_hal_gpio.h"
#include "stm32g4xx_hal_spi.h"
#include <math.h>
#include <stdint.h>

#ifndef FOC_PI
#define FOC_PI 3.14159265358979323846f
#endif

static Encoder_MT6826S_Context_t *Encoder_Context(Encoder_t *encoder)
{
    return (Encoder_MT6826S_Context_t *)encoder->context;
}

static void Encoder_Init(Encoder_t *encoder)
{
    encoder->initialized = true;
}

static void Encoder_Enable(Encoder_t *encoder)
{
    if(!encoder->initialized) return;
    static uint8_t tx_data[2] = {[0] = 0xA0, [1] = 0x03};
    Encoder_MT6826S_Context_t *ctx = Encoder_Context(encoder);
    HAL_GPIO_WritePin(ctx->cs_gpio_port, ctx->cs_gpio_pin, GPIO_PIN_SET);
    HAL_SPI_Transmit(ctx->hspi, tx_data, 1, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(ctx->cs_gpio_port, ctx->cs_gpio_pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(ctx->hspi, tx_data, 2, HAL_MAX_DELAY);
    encoder->enabled = true;
}

static void Encoder_Disable(Encoder_t *encoder)
{
    if(!encoder->initialized) return;
    Encoder_MT6826S_Context_t *ctx = Encoder_Context(encoder);
    HAL_GPIO_WritePin(ctx->cs_gpio_port, ctx->cs_gpio_pin, GPIO_PIN_SET);
    encoder->enabled = false;
}

static float Encoder_GetAngle(Encoder_t *encoder)
{
    static uint8_t rx_data[4];
    if(!encoder->enabled) return 0.0f;
    Encoder_MT6826S_Context_t *ctx = Encoder_Context(encoder);
    HAL_SPI_Receive(ctx->hspi, rx_data, 4, HAL_MAX_DELAY);
    return (float)(((uint16_t)rx_data[0] << 7) | ((uint16_t)rx_data[1] >> 1)) / 32768.0f * 2.0f * FOC_PI;
    //把读出的数据进行拼接后转化为弧度
}

void Encoder_MT6826S_Bind(Encoder_t *encoder,
                          Encoder_MT6826S_Context_t *context,
                          GPIO_TypeDef *cs_gpio_port,
                          uint16_t cs_gpio_pin,
                          SPI_HandleTypeDef *hspi)
{
    context->cs_gpio_port = cs_gpio_port;
    context->cs_gpio_pin = cs_gpio_pin;
    context->hspi = hspi;

    encoder->initialized = false;
    encoder->enabled = false;

    encoder->init = Encoder_Init;
    encoder->enable = Encoder_Enable;
    encoder->disable = Encoder_Disable;
    encoder->get_angle = Encoder_GetAngle;

    encoder->context = context;
    //硬件数据层挂载在驱动函数层
}