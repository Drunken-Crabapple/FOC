/**
 * @file Storage.h
 * @brief C storage abstraction.
 */

#ifndef STORAGE_H
#define STORAGE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct Storage Storage_t;

struct Storage {
    bool initialized;
    uint32_t storage_size;
    void (*init)(Storage_t *storage);
    void (*write)(Storage_t *storage, uint32_t addr, const void *buff, uint32_t count);
    void (*read)(Storage_t *storage, uint32_t addr, void *buff, uint32_t count);
    void *context;
};

#ifdef __cplusplus
}
#endif

#endif
