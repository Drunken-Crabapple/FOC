/**
 * @file BLDC_Driver.h
 * @brief C hardware abstraction for a three phase BLDC driver.
 */

#ifndef BLDC_DRIVER_H
#define BLDC_DRIVER_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct BLDC_Driver BLDC_Driver_t;

struct BLDC_Driver {
    bool initialized;
    bool enabled;
    void (*init)(BLDC_Driver_t *driver);
    void (*enable)(BLDC_Driver_t *driver);
    void (*disable)(BLDC_Driver_t *driver);
    void (*set_duty)(BLDC_Driver_t *driver, float u, float v, float w);
    void *context;
};

#ifdef __cplusplus
}
#endif

#endif
