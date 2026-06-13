#ifndef ENCODER_H
#define ENCODER_H

#include <stdbool.h>

typedef struct Encoder Encoder_t;

struct Encoder
{
    bool initialized;
    bool enabled;

    void (*init)(Encoder_t *encoder);
    void (*enable)(Encoder_t *encoder);
    void (*disable)(Encoder_t *encoder);

    float (*get_angle)(Encoder_t *encoder);

    void *context;
};


#endif
