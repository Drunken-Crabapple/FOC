#include "BLDC_Driver_DRV8300.h"

static BLDC_Driver_DRV8300_Context_t *DRV8300_Context(BLDC_Driver_t *driver) {
    return (BLDC_Driver_DRV8300_Context_t *)driver->context;
}

static void DRV8300_Init(BLDC_Driver_t *driver) {
    driver->initialized = true;
}

static void DRV8300_Enable(BLDC_Driver_t *driver) {
    BLDC_Driver_DRV8300_Context_t *ctx = DRV8300_Context(driver);
    HAL_TIM_PWM_Start(ctx->htim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(ctx->htim, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(ctx->htim, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(ctx->htim, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(ctx->htim, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(ctx->htim, TIM_CHANNEL_3);
    driver->enabled = true;
}

static void DRV8300_Disable(BLDC_Driver_t *driver) {
    BLDC_Driver_DRV8300_Context_t *ctx = DRV8300_Context(driver);
    driver->set_duty(driver, 0.0f, 0.0f, 0.0f);
    HAL_TIM_PWM_Stop(ctx->htim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(ctx->htim, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(ctx->htim, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Stop(ctx->htim, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(ctx->htim, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(ctx->htim, TIM_CHANNEL_3);
    driver->enabled = false;
}

static void DRV8300_SetDuty(BLDC_Driver_t *driver, float u, float v, float w) {
    if (!driver->enabled) return;
    BLDC_Driver_DRV8300_Context_t *ctx = DRV8300_Context(driver);
    __HAL_TIM_SET_COMPARE(ctx->htim, TIM_CHANNEL_1, (uint32_t)(u * ctx->max_duty));
    __HAL_TIM_SET_COMPARE(ctx->htim, TIM_CHANNEL_3, (uint32_t)(v * ctx->max_duty));
    __HAL_TIM_SET_COMPARE(ctx->htim, TIM_CHANNEL_2, (uint32_t)(w * ctx->max_duty));
}

void BLDC_Driver_DRV8300_Bind(BLDC_Driver_t *driver,
                              BLDC_Driver_DRV8300_Context_t *context,
                              TIM_HandleTypeDef *htim,
                              uint16_t max_duty) {
    context->htim = htim;
    context->max_duty = max_duty;
    driver->initialized = true;
    driver->enabled = false;
    driver->init = DRV8300_Init;
    driver->enable = DRV8300_Enable;
    driver->disable = DRV8300_Disable;
    driver->set_duty = DRV8300_SetDuty;
    driver->context = context;
}
