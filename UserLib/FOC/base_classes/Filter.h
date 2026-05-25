/**
 * @file Filter.h
 * @brief C filter abstraction.
 */

#ifndef FILTER_H
#define FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

typedef struct Filter Filter_t;

struct Filter {
    float (*apply)(Filter_t *filter, float value);
    float (*get_ts)(Filter_t *filter);
    void *context;
};

#ifdef __cplusplus
}
#endif

#endif
