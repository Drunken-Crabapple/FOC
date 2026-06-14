#ifndef CURRENT_SENSOR_EMBED_H
#define CURRENT_SENSOR_EMBED_H

#include "CurrentSensor.h"
#include "adc.h"
#include "stm32g4xx_hal_adc.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    ADC_HandleTypeDef *hadc1;
    ADC_HandleTypeDef *hadc2;
} CurrentSensor_Embed_Context_t;

void CurrentSensor_Embed_Bind(CurrentSensor_t *sensor,
                              CurrentSensor_Embed_Context_t *context,
                              ADC_HandleTypeDef *hadc1,
                              ADC_HandleTypeDef *hadc2);

#ifdef __cplusplus
}
#endif


#endif

