#include "task_public.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include "shell.h"
#include "retarget/retarget.h"
#include "FreeRTOS.h"
#include "task.h"

Shell shell;
char shellBuffer[256];

void USB_Disconnected(void) {
    __HAL_RCC_USB_FORCE_RESET();
    HAL_Delay(200);
    __HAL_RCC_USB_RELEASE_RESET();

    GPIO_InitTypeDef gpio_init;
    __HAL_RCC_GPIOA_CLK_ENABLE();

    gpio_init.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    gpio_init.Mode = GPIO_MODE_OUTPUT_PP;
    gpio_init.Pull = GPIO_PULLDOWN;
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &gpio_init);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_Delay(300);
}

void StartStartShell(void *argument) {
    (void)argument;
    USB_Disconnected();
    MX_USB_Device_Init();
    delay(100);
    RetargetInit();
    shell.read = shellRead;
    shell.write = shellWrite;
    shellInit(&shell, shellBuffer, 256);
    xTaskCreate(shellTask, "LetterShellTask", 512, &shell, osPriorityNormal, NULL);
    vTaskDelete(NULL);
}
