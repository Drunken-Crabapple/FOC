/**
 * @name        sys_public.h
 * @brief       用于支持简单平台移植的中间层
 */

#ifndef SYS_PUBLIC_H
#define SYS_PUBLIC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "cmsis_os2.h"
#include "stm32g4xx_hal.h"

#define SYS_LOG_ENABLE          0
#define MAX_DELAY               portMAX_DELAY
#define TX_BUFFER_LENGTH        64

#define delay(ms)               delay_ms(ms)
#define delay_ms(ms)            osDelay(ms)
#define delay_us(us)            do { \
                                    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; \
                                    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; \
                                    const uint32_t _start = DWT->CYCCNT; \
                                    const uint32_t _ticks = (SystemCoreClock / 1000000U) * (uint32_t)(us); \
                                    while ((DWT->CYCCNT - _start) < _ticks) { __asm volatile ("nop"); } \
                                } while (0)
#define get_tick()              xTaskGetTickCount()
#define sys_log_function(...)   printf(__VA_ARGS__)

#if SYS_LOG_ENABLE == 1
#define sys_log(...)            sys_log_function(__VA_ARGS__)
#define sys_log_info(...)       sys_log_function("INFO: " __VA_ARGS__)
#define sys_log_warning(...)    sys_log_function("WARNING: " __VA_ARGS__)
#define sys_log_error(...)      sys_log_function("ERROR: " __VA_ARGS__)
#else
#define sys_log(...)            ((void)0U)
#define sys_log_info(...)       ((void)0U)
#define sys_log_warning(...)    ((void)0U)
#define sys_log_error(...)      ((void)0U)
#endif

#ifdef __cplusplus
}
#endif

#endif
