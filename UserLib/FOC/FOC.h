#ifndef FOC_H
#define FOC_H

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "PID.h"
#include "BLDC_Driver.h"
#include "Encoder.h"
#include "CurrentSensor.h"
#include "Filter.h"

#ifndef FOC_PI
#define FOC_PI  3.14159265358979323846f
#endif

#ifndef FOC_MAP_LEN
#define FOC_MAP_LEN 2000U
#endif


typedef enum
{
    FOC_CTRL_CURRENT = 0,       //电流控制模式
    FOC_CTRL_SPEED = 1,         //速度控制模式
    FOC_CTRL_ANGLE = 2,         //绝对角度
    FOC_CTRL_STEP_ANGLE = 3,    //相对角度
    FOC_CTRL_LOW_SPEED = 4,     //低速连续
} FOC_CtrlType;


typedef struct 
{
    uint8_t pole_pairs;                 //电机极对数，用于后续讲机械角度转化为电角度
    uint16_t ctrl_frequency;            //外环控制频率
    uint16_t current_ctrl_frequency;    //电流环控制频率

    //标志位
    bool initialized;
    bool enabled;
    bool started;
    bool calibrated;
    bool anticogging_enabled;
    bool anticogging_calibrated;
    bool anticogging_calibrating;

    FOC_CtrlType ctrl_type;
    
    PID_t pid_current_q;    //电流环
    PID_t pid_current_d;
    PID_t pid_speed;        //速度环
    PID_t pid_angle;        //角度环

    BLDC_Driver_t *driver;              //电机驱动接口
    Encoder_t *encoder;                 //编码器接口    
    CurrentSensor_t *current_sensor;    //电流传感器接口

    Filter_t *current_q_filter;
    Filter_t *current_d_filter;
    Filter_t *speed_filter;

    bool encoder_direction;     //编码器方向
    float phase_resistance;     //相电阻
    float phase_inductance;     //相电感
    float iu_offset;            //u相电流零点偏移
    float iv_offset;            //v相
    float zero_electric_angle;  //零电角度偏移

    float *anticogging_map;

    float target_iq;
    float angle;
    float previous_angle;
    float electrical_angle;
    float speed;
    float low_speed;        //低速模式专用


    /*
        ADC采样出两相电流 据此算出第三相 做clark得出 ia,ib,做park得出id,iq
        输入电流环pid 得到ud uq,做反clark 反park得出三相电压 输入控制器
    */
    float uu;
    float uv;
    float uw;

    float ua;
    float ub;

    float uq;
    float ud;

    float iu;
    float iv;
    float iw;

    float ia;
    float ib;

    float iq;
    float id;

    float voltage;

} FOC_t;  

float FOC_Wrap(float value,float min,float max);

void FOC_InitObject(FOC_t *foc,
                    uint8_t pole_pairs,
                    uint16_t ctrl_frequency,
                    uint16_t current_ctrl_frequency,
                    Filter_t *current_q_filter,
                    Filter_t *current_d_filter,
                    Filter_t *speed_filter,
                    BLDC_Driver_t *driver,
                    Encoder_t *encoder,
                    CurrentSensor_t *current_sensor,
                    const PID_t *pid_current_q,
                    const PID_t *pid_current_d,
                    const PID_t *pid_speed,
                    const PID_t *pid_angle);

void FOC_Init(FOC_t *foc);
void FOC_Enable(FOC_t *foc);

void FOC_Disable(FOC_t *foc);
void FOC_Start(FOC_t *foc);
void FOC_Stop(FOC_t *foc);

void FOC_UpdateVoltage(FOC_t *foc,float voltage);
void FOC_CurrentCalibrate(FOC_t *foc);
void FOC_Calibrate(FOC_t *foc);


void FOC_Ctrl(FOC_t *foc,FOC_CtrlType ctrl_type,float value);
void FOC_CtrlISR(FOC_t *foc);
void FOC_LoopCtrl(FOC_t *foc);
void FOC_AnticoggingCalibrate(FOC_t *foc);



#endif
