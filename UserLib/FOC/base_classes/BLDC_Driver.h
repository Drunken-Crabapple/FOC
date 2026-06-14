#ifndef BLDC_DRIVER_H
#define BLDC_DRIVER_H

#include <stdbool.h>

typedef struct BLDC_Driver BLDC_Driver_t;

struct BLDC_Driver
{
    bool initialized;
    bool enabled;

    void (*init)(BLDC_Driver_t *driver);    //指针函数
    void (*enable)(BLDC_Driver_t *driver);
    void (*disable)(BLDC_Driver_t *driver);
    void (*set_duty)(BLDC_Driver_t *driver,float u,float v,float w);

    void *context;  //硬件层私有数据
};


#endif
