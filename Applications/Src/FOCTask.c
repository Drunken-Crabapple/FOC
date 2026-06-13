#include "CurrentSensor.h"
#include "Encoder.h"
#include "PID.h"
#include "main.h"
#include "stm32g431xx.h"
#include "stm32g4xx_hal_tim.h"
#include "stm32g4xx_ll_adc.h"
#include "task_public.h"
#include "FOC.h"
#include "FOC_config.h"
#include "tim.h"
#include "spi.h"
#include "adc.h"
#include "cmsis_os2.h"
#include "Encoder_MT6826S.h"
#include "BLDC_Driver_DRV8300.h"
#include "Storage_EmbeddedFlash.h"
#include "CurrentSensor_Embed.h"
#include "filters_c.h"
#include "QD4310.h"

static BLDC_Driver_t bldc_driver;
static BLDC_Driver_DRV8300_Context_t bldc_driver_context;

static Encoder_t bldc_encoder;
static Encoder_MT6826S_Context_t bldc_encoder_context;

static CurrentSensor_t current_sensor;
static CurrentSensor_Embed_Context_t current_sensor_context;

static LowPassFilter2_t current_q_filter;
static LowPassFilter2_t current_d_filter;
static LowPassFilter2_t speed_filter;

static void FOCTask_InitObjects(void)
{
    BLDC_Driver_DRV8300_Bind(&bldc_driver, &bldc_driver_context,&htim1,2125);
    Encoder_MT6826S_Bind(&bldc_encoder, &bldc_encoder_context, SPI1_CSn_GPIO_Port, SPI1_CSn_Pin, &hspi1);
    CurrentSensor_Embed_Bind(&current_sensor, &current_sensor_context, &hadc1, &hadc2);
    Storage_EmbeddedFlash_Bind(&storage, &storage_context, 0x0801D800U, 0x00002800U);

    LowPassFilter2_Init(&current_q_filter, 0.00005f, 1500.0f, 0.707f);
    LowPassFilter2_Init(&current_d_filter, 0.00005f, 1500.0f, 0.707f);
    LowPassFilter2_Init(&speed_filter, 0.00005f, 300.0f, 0.707f);

    PID_t pid_current_q;
    PID_t pid_current_d;
    PID_t pid_speed;
    PID_t pid_angle;

    PID_Init(&pid_current_q,PID_DELTA_TYPE,FOC_CURRENT_KP,FOC_CURRENT_KI,FOC_CURRENT_KD,NAN,NAN,1.0f,-1.0f);
    PID_Init(&pid_current_d,PID_DELTA_TYPE,FOC_CURRENT_KP,FOC_CURRENT_KI,FOC_CURRENT_KD,NAN,NAN,1.0f,-1.0f);
    PID_Init(&pid_speed,PID_POSITION_TYPE,FOC_SPEED_KP,FOC_SPEED_KI,FOC_SPEED_KD,1e4f,-1e4f,FOC_MAX_CURRENT,-FOC_MAX_CURRENT);
    PID_Init(&pid_angle,PID_POSITION_TYPE,FOC_ANGLE_KP,FOC_ANGLE_KI,FOC_ANGLE_KD,NAN,NAN,FOC_MAX_SPEED,-FOC_MAX_SPEED);

    QD4310_InitObject(&qd4310, FOC_POLE_PAIRS, 5000U, 20000U, &current_q_filter.base, &current_d_filter.base, &speed_filter.base, &bldc_driver, &bldc_encoder, &storage, &current_sensor, &pid_current_q, &pid_current_d, &pid_speed, &pid_angle);
}

void StartFOCTask(void *argument)
{
    (void)argument;
    osDelay(1500);
    FOCTask_InitObjects();
    HAL_TIM_Base_Start_IT(&htim6);                          // 5kHz 定时调用外环控制函数
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);      //  20000Hz 电流环控制频率
    QD4310_Init(&qd4310);
    FOC_Enable(&qd4310.foc);
    while(true)
    {
        if(!LL_ADC_REG_IsConversionOngoing(hadc1.Instance))
        {
            LL_ADC_REG_StartConversion(hadc1.Instance);
            FOC_UpdateVoltage(&qd4310.foc, hadc1.Instance->DR / 4095.0f * 3.3f / 2.0f * 17.0f);
            LL_ADC_REG_StopConversion(hadc1.Instance);
        }
        delay(1);
    }
}

__attribute__((section(".ccmram_func")))
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (&hadc1 == hadc) {
        current_sensor.update(&current_sensor);
        FOC_LoopCtrl(&qd4310.foc);
    }
}

__attribute__((section(".ccmram_func")))
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (&htim6 == htim) {
        FOC_CtrlISR(&qd4310.foc);
    }
}
