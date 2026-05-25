#include "Storage_EmbeddedFlash.h"
#include <string.h>

Storage_t storage;
Storage_EmbeddedFlash_Context_t storage_context;

static Storage_EmbeddedFlash_Context_t *Storage_Context(Storage_t *storage_) {
    return (Storage_EmbeddedFlash_Context_t *)storage_->context;
}

static uint32_t min_u32(uint32_t a, uint32_t b) {
    return a < b ? a : b;
}

static void Storage_Init(Storage_t *storage_) {
    storage_->initialized = true;
}

static void write_page_bytes(Storage_t *storage_, uint32_t page, uint32_t addr, const void *pdata, uint32_t count) {
    Storage_EmbeddedFlash_Context_t *ctx = Storage_Context(storage_);
    if (count == 0U) return;
    count = min_u32(count, FLASH_PAGE_SIZE - addr);

    for (uint32_t i = 0; i < FLASH_PAGE_SIZE; i++) {
        ctx->page_buffer[i] = *(volatile uint8_t *)(FLASH_BASE + page * FLASH_PAGE_SIZE + i);
    }
    memcpy(ctx->page_buffer + addr, pdata, count);

    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef erase_init;
    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.Banks = FLASH_BANK_1;
    erase_init.Page = page;
    erase_init.NbPages = 1;
    uint32_t page_error = 0;
    while (HAL_OK != HAL_FLASHEx_Erase(&erase_init, &page_error)) {}

    for (uint32_t i = 0; i < FLASH_PAGE_SIZE / 8U; i++) {
        uint64_t double_word;
        memcpy(&double_word, ctx->page_buffer + i * 8U, sizeof(double_word));
        while (HAL_OK != HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
                                           FLASH_BASE + page * FLASH_PAGE_SIZE + i * 8U,
                                           double_word)) {}
    }

    HAL_FLASH_Lock();
}

static void Storage_Write(Storage_t *storage_, uint32_t addr, const void *buff, uint32_t count) {
    Storage_EmbeddedFlash_Context_t *ctx = Storage_Context(storage_);
    count = min_u32(count, storage_->storage_size - addr);

    const uint32_t start_page = (ctx->storage_address_base + addr - FLASH_BASE) / FLASH_PAGE_SIZE;
    const uint32_t end_page = (ctx->storage_address_base + addr + count - FLASH_BASE) / FLASH_PAGE_SIZE;
    const uint32_t page_num = end_page - start_page + 1U;
    const uint32_t offset = addr % FLASH_PAGE_SIZE;
    const uint8_t *bytes = (const uint8_t *)buff;

    if (page_num == 1U) {
        write_page_bytes(storage_, start_page, offset, buff, count);
    } else {
        for (uint32_t i = 0; i < page_num; i++) {
            if (i == 0U) {
                write_page_bytes(storage_, start_page, offset, bytes, FLASH_PAGE_SIZE - offset);
            } else if (i == page_num - 1U) {
                write_page_bytes(storage_, start_page + i, 0U, bytes + FLASH_PAGE_SIZE * i - offset,
                                 count + offset - FLASH_PAGE_SIZE * i);
            } else {
                write_page_bytes(storage_, start_page + i, 0U, bytes + FLASH_PAGE_SIZE * i - offset,
                                 FLASH_PAGE_SIZE);
            }
        }
    }
}

static void Storage_Read(Storage_t *storage_, uint32_t addr, void *buff, uint32_t count) {
    Storage_EmbeddedFlash_Context_t *ctx = Storage_Context(storage_);
    count = min_u32(count, storage_->storage_size - addr);
    const uint8_t *src = (const uint8_t *)(ctx->storage_address_base + addr);
    memcpy(buff, src, count);
}

void Storage_EmbeddedFlash_Bind(Storage_t *storage_,
                                Storage_EmbeddedFlash_Context_t *context,
                                uint32_t storage_address_base,
                                uint32_t storage_size) {
    context->storage_address_base = storage_address_base;
    memset(context->page_buffer, 0, sizeof(context->page_buffer));
    storage_->initialized = false;
    storage_->storage_size = storage_size;
    storage_->init = Storage_Init;
    storage_->write = Storage_Write;
    storage_->read = Storage_Read;
    storage_->context = context;
}
