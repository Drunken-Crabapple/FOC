/**
 * @file BLDC_Driver_DRV8300.h
 * @brief C adapter for DRV8300 three phase PWM driver.
 */

#ifndef BLDC_DRIVER_DRV8300_H
#define BLDC_DRIVER_DRV8300_H

#include <stdint.h>
#include "tim.h"
#include "BLDC_Driver.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    TIM_HandleTypeDef *htim;
    uint16_t max_duty;
} BLDC_Driver_DRV8300_Context_t;

void BLDC_Driver_DRV8300_Bind(BLDC_Driver_t *driver,
                              BLDC_Driver_DRV8300_Context_t *context,
                              TIM_HandleTypeDef *htim,
                              uint16_t max_duty);

#ifdef __cplusplus
}
#endif

#endif
