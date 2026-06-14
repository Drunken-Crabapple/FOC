#include "CurrentSensor_Embed.h"
#include "CurrentSensor.h"
#include "stm32g4xx_hal_adc.h"
#include "stm32g4xx_hal_adc_ex.h"

static CurrentSensor_Embed_Context_t *CurrentSensor_Context(CurrentSensor_t *sensor)
{
    return (CurrentSensor_Embed_Context_t *)sensor->context;
}

static void CurrentSensor_Init(CurrentSensor_t *sensor)
{
    CurrentSensor_Embed_Context_t *ctx = CurrentSensor_Context(sensor);
    HAL_ADCEx_Calibration_Start(ctx->hadc1, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_Start(ctx->hadc2, ADC_SINGLE_ENDED);
    sensor->initialized = true;
}

static void CurrentSensor_Enable(CurrentSensor_t *sensor)
{
    CurrentSensor_Embed_Context_t *ctx = CurrentSensor_Context(sensor);
    ADC_Enable(ctx->hadc1);
    ADC_Enable(ctx->hadc2);
    HAL_ADCEx_InjectedStart_IT(ctx->hadc1);
    sensor->enabled = true;
}

static void CurrentSensor_Disable(CurrentSensor_t *sensor)
{
    CurrentSensor_Embed_Context_t *ctx = CurrentSensor_Context(sensor);
    ADC_Disable(ctx->hadc1);
    ADC_Disable(ctx->hadc2);
    HAL_ADCEx_InjectedStop_IT(ctx->hadc1);
    sensor->enabled = false;
}

static void CurrentSensor_Update(CurrentSensor_t *sensor)
{
    static const float v_ref = 3.3f;
    static const float adc_resolution = 4096.0f - 1.0f;
    static const float op_amp_gain = 20.0f;
    static const float r_sense = 0.05f;

    CurrentSensor_Embed_Context_t *ctx = CurrentSensor_Context(sensor);

    const float iu = 2048.0f - (float)ctx->hadc2->Instance->JDR1;
    sensor->iu = iu / adc_resolution * v_ref / op_amp_gain / r_sense;

    const float iv = (float)ctx->hadc1->Instance->JDR1 - 2048.0f;
    sensor->iv = iv / adc_resolution * v_ref / op_amp_gain / r_sense;

    sensor->iw = -(sensor->iu + sensor->iv); 
}

void CurrentSensor_Embed_Bind(CurrentSensor_t *sensor,
                              CurrentSensor_Embed_Context_t *context,
                              ADC_HandleTypeDef *hadc1,
                              ADC_HandleTypeDef *hadc2)
{
    context->hadc1 = hadc1;
    context->hadc2 = hadc2;

    sensor->initialized = false;
    sensor->enabled = false;

    sensor->iu = 0.0f;
    sensor->iv = 0.0f;
    sensor->iw = 0.0f;

    sensor->init = CurrentSensor_Init;
    sensor->enable = CurrentSensor_Enable;
    sensor->disable = CurrentSensor_Disable;
    sensor->update = CurrentSensor_Update;

    sensor->context = context;
}                              
