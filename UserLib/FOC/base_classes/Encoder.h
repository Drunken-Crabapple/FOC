/**
 * @file Encoder.h
 * @brief C hardware abstraction for rotor position encoders.
 */

#ifndef ENCODER_H
#define ENCODER_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct Encoder Encoder_t;

struct Encoder {
    bool initialized;
    bool enabled;
    void (*init)(Encoder_t *encoder);
    void (*enable)(Encoder_t *encoder);
    void (*disable)(Encoder_t *encoder);
    float (*get_angle)(Encoder_t *encoder);
    void *context;
};

#ifdef __cplusplus
}
#endif

#endif
