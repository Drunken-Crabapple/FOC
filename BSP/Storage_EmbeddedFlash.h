/**
 * @file Storage_EmbeddedFlash.h
 * @brief C adapter for STM32 embedded flash storage.
 */

#ifndef STORAGE_EMBEDDED_FLASH_H
#define STORAGE_EMBEDDED_FLASH_H

#include "Storage.h"
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t storage_address_base;
    uint8_t page_buffer[FLASH_PAGE_SIZE];
} Storage_EmbeddedFlash_Context_t;

extern Storage_t storage;
extern Storage_EmbeddedFlash_Context_t storage_context;

void Storage_EmbeddedFlash_Bind(Storage_t *storage,
                                Storage_EmbeddedFlash_Context_t *context,
                                uint32_t storage_address_base,
                                uint32_t storage_size);

#ifdef __cplusplus
}
#endif

#endif
