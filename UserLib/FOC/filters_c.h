/**
 * @file filters_c.h
 * @brief C filter implementations used by the FOC loop.
 */

#ifndef FILTERS_C_H
#define FILTERS_C_H

#include <math.h>
#include <string.h>
#include "Filter.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef FOC_PI
#define FOC_PI 3.14159265358979323846f
#endif

typedef struct {
    Filter_t base;
    float ts;
    float fc;
    float wc;
    float b0;
    float b1;
    float b2;
    float a0;
    float a1;
    float a2;
    float xin[3];
    float yout[3];
} LowPassFilter2_t;

static inline float LowPassFilter2_Apply(Filter_t *filter, float value) {
    LowPassFilter2_t *self = (LowPassFilter2_t *)filter->context;
    self->xin[2] = value;
    self->yout[2] = (self->b0 * self->xin[2] +
                     self->b1 * self->xin[1] +
                     self->b2 * self->xin[0] -
                     self->a1 * self->yout[1] -
                     self->a2 * self->yout[0]) / self->a0;
    self->xin[0] = self->xin[1];
    self->xin[1] = self->xin[2];
    self->yout[0] = self->yout[1];
    self->yout[1] = self->yout[2];
    return self->yout[2];
}

static inline float LowPassFilter2_GetTs(Filter_t *filter) {
    LowPassFilter2_t *self = (LowPassFilter2_t *)filter->context;
    return self->ts;
}

static inline void LowPassFilter2_Init(LowPassFilter2_t *filter, float ts, float fc, float damping_ratio) {
    memset(filter, 0, sizeof(*filter));
    filter->ts = ts;
    filter->fc = fc;
    filter->wc = 2.0f / ts * tanf(FOC_PI * fc * ts);
    filter->b0 = filter->wc * filter->wc * ts * ts;
    filter->b1 = 2.0f * filter->b0;
    filter->b2 = filter->b0;
    filter->a0 = 4.0f + 4.0f * damping_ratio * filter->wc * ts + filter->b0;
    filter->a1 = -8.0f + 2.0f * filter->b0;
    filter->a2 = 4.0f - 4.0f * damping_ratio * filter->wc * ts + filter->b0;
    filter->base.apply = LowPassFilter2_Apply;
    filter->base.get_ts = LowPassFilter2_GetTs;
    filter->base.context = filter;
}

#ifdef __cplusplus
}
#endif

#endif
