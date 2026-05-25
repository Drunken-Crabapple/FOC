/**
 * @file CurrentSensor.h
 * @brief C hardware abstraction for current sensing.
 */

#ifndef CURRENT_SENSOR_H
#define CURRENT_SENSOR_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct CurrentSensor CurrentSensor_t;

struct CurrentSensor {
    bool initialized;
    bool enabled;
    float iu;
    float iv;
    float iw;
    void (*init)(CurrentSensor_t *sensor);
    void (*enable)(CurrentSensor_t *sensor);
    void (*disable)(CurrentSensor_t *sensor);
    void (*update)(CurrentSensor_t *sensor);
    void *context;
};

#ifdef __cplusplus
}
#endif

#endif
