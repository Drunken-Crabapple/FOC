#include "task_public.h"
#include "FreeRTOS.h"
#include "task.h"

void StartDebugTask(void *argument) {
    (void)argument;
    vTaskDelete(NULL);
    for (;;) {
        delay(1000);
    }
}
