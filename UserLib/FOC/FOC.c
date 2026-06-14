#include "FOC.h"
#include "BLDC_Driver.h"
#include "CurrentSensor.h"
#include "Encoder.h"
#include "Filter.h"
#include "PID.h"
#include "FOC_config.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "sys_public.h"

static float clampf_local(float value,float min_value,float max_value)
{
    if(value < min_value) return min_value;
    if(value > max_value) return max_value;
    return value;
}

/*
    将输入角度归一化到 0 - 2PI  和后面处理正反绕过零点不是一个东西
*/
float FOC_Wrap(float value,float min,float max)
{
    value = fmodf(value - min, max - min);
    return value < 0.0f ? value + max : value + min;
}


//链接各个外部小模块到foc主结构体
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
                    const PID_t *pid_angle)
{
    memset(foc,0,sizeof(*foc));

    foc->pole_pairs = pole_pairs;
    foc->ctrl_frequency = ctrl_frequency;
    foc->current_ctrl_frequency = current_ctrl_frequency;

    foc->current_q_filter = current_q_filter;
    foc->current_d_filter = current_d_filter;
    foc->speed_filter = speed_filter;

    foc->driver = driver;
    foc->encoder = encoder;
    foc->current_sensor = current_sensor;

    foc->pid_current_q = *pid_current_q;
    foc->pid_current_d = *pid_current_d;
    foc->pid_speed = *pid_speed;
    foc->pid_angle = *pid_angle;

    foc->anticogging_map = (float *)calloc(FOC_MAP_LEN, sizeof(float));

    foc->encoder_direction = true;
    foc->phase_resistance = NAN;
    foc->phase_inductance = NAN;
    foc->voltage = 1.0f;

    foc->ctrl_type = FOC_CTRL_CURRENT;  //默认电流模式
}

void FOC_Init(FOC_t *foc)
{
    if(!foc->driver->initialized) foc->driver->init(foc->driver);
    if(!foc->encoder->initialized) foc->encoder->init(foc->encoder);
    if(!foc->current_sensor->initialized) foc->current_sensor->init(foc->current_sensor);

    foc->initialized = true;
}

void FOC_Enable(FOC_t *foc)
{
    if(!foc->initialized) return;
    if(!foc->driver->enabled)
    {
        foc->driver->enable(foc->driver);
        foc->driver->set_duty(foc->driver,0.0f,0.0f,0.0f);  //默认设置三相占空比为零  避免残留数值导致乱飞
    }

    if(!foc->encoder->enabled) foc->encoder->enable(foc->encoder);

    if(!foc->current_sensor->enabled) foc->current_sensor->enable(foc->current_sensor);

    foc->enabled = true;
}

void FOC_Disable(FOC_t *foc)
{
    if(foc->driver->enabled)        //失能的前提是使能
    {
        foc->driver->set_duty(foc->driver,0.0f,0.0f,0.0f);  //保证关闭前清空状态
        foc->driver->disable(foc->driver);
    }

    if(foc->encoder->enabled) foc->encoder->disable(foc->encoder);

    foc->enabled = false;
}

void FOC_Start(FOC_t *foc)
{
    if(foc->enabled && foc->calibrated) foc->started = true;
}

void FOC_Stop(FOC_t *foc)
{
    foc->started = false;
}

/*
    保存三相电流 进行clark park变换求得q d电流并进行滤波
*/
static void FOC_UpdateCurrent(FOC_t *foc,float iu,float iv,float iw)
{
    foc->iu = iu;
    foc->iv = iv;
    foc->iw = iw;

    //进行clark变换
    foc->ia = foc->iu;
    foc->ib = (foc->iu + 2.0 * foc->iv) * 0.5773502691896258f;


    //进行park变换
    const float cos_angle = cosf(foc->electrical_angle);
    const float sin_angle = sinf(foc->electrical_angle);

    //对park变换后得到的q轴 d轴电流进行二阶低通滤波
    foc->iq = foc->current_q_filter->apply(foc->current_q_filter,
                                           foc->ib * cos_angle - foc->ia * sin_angle);

    foc->id = foc->current_d_filter->apply(foc->current_d_filter,
                                           foc->ib * sin_angle + foc->ia * cos_angle);
}

/*
    对电流环输出的ud uq做反park 反clark变换求出三相电压 并输入控制器
    是FOC的最终输出函数
*/
static void FOC_SetPhaseVoltage(FOC_t *foc,float ud,float uq,float electrical_angle)
{

    ud *= 0.99f;
    uq *= 0.99f;

    foc->ud = ud;
    foc->uq = uq;

    const float cos_angle = cosf(electrical_angle);
    const float sin_angle = sinf(electrical_angle);

    foc->ua = (-uq * sin_angle + ud * cos_angle) * 0.5f;
    foc->ub = (uq * cos_angle + ud * sin_angle) * 0.5f;

    foc->uu = foc->ua + 0.5f;
    foc->uv = -foc->ua * 0.5f + foc->ub * 0.8660254037844386f + 0.5f;
    foc->uw = 1.5f - foc->uu - foc->uv;

    foc->driver->set_duty(foc->driver,foc->uu,foc->uv,foc->uw);
}

void FOC_UpdateVoltage(FOC_t *foc,float voltage)
{
    if(voltage == 0.0f) return;

    foc->pid_current_q.kp *= foc->voltage / voltage;
    foc->pid_current_d.kp *= foc->voltage / voltage;

    foc->pid_current_q.ki *= foc->voltage / voltage;
    foc->pid_current_d.ki *= foc->voltage / voltage;

    foc->voltage = voltage;
}

void FOC_CurrentCalibrate(FOC_t *foc)
{
    if(!foc->enabled) return;
    if(foc->started) return;

    const bool calibrate_status = foc->calibrated;
    foc->calibrated = false;

    FOC_SetPhaseVoltage(foc,0.0f, 0.0f, 0.0f);
    delay(30);

    foc->iu_offset = 0.0f;
    foc->iv_offset = 0.0f;

    for(int i = 0; i < 200; i++)
    {
        foc->iu_offset += foc->current_sensor->iu / 200.0f;
        foc->iv_offset += foc->current_sensor->iv / 200.0f;
        delay(1);
    }

    foc->calibrated = calibrate_status;
}

void FOC_Calibrate(FOC_t *foc)
{
    if (!foc->enabled) return;
    if (foc->started) return;
    foc->calibrated = false;

    FOC_CurrentCalibrate(foc);

    foc->driver->set_duty(foc->driver, 0.0f, 0.0f, 0.0f);
    float uu = 0.0f;
    for (uu = 0.0f; uu < 0.8f && foc->current_sensor->iu - foc->iu_offset < FOC_MAX_CURRENT * 0.9f;) {
        uu += 0.001f;
        foc->driver->set_duty(foc->driver, uu, 0.0f, 0.0f);
        delay(1);
    }
    foc->phase_resistance = 0.0f;
    for (int i = 0; i < 100; ++i) {
        foc->phase_resistance += uu * foc->voltage / (foc->current_sensor->iu - foc->iu_offset) / 3.0f * 4.0f / 100.0f;
        delay(1);
    }
    FOC_SetPhaseVoltage(foc, 0.0f, 0.0f, 0.0f);
    const float voltage_align = clampf_local(uu * 4.0f / 3.0f, -1.0f, 1.0f);

    FOC_SetPhaseVoltage(foc, voltage_align * 0.6f, 0.0f, 0.0f);
    delay(100);
    float begin_angle = 0.0f;
    for (int i = 0; i < 100; ++i) {
        begin_angle += foc->encoder->get_angle(foc->encoder) / 100.0f;
    }
    for (int i = 0; i < 500; ++i) {
        const float angle = 2.0f * FOC_PI * (float)i / 500.0f;
        FOC_SetPhaseVoltage(foc, voltage_align * 0.6f, 0.0f, angle);
        delay(1);
    }
    float end_angle = 0.0f;
    for (int i = 0; i < 100; ++i) {
        end_angle += foc->encoder->get_angle(foc->encoder) / 100.0f;
    }
    FOC_SetPhaseVoltage(foc, 0.0f, 0.0f, 0.0f);
    if ((end_angle > begin_angle && end_angle < begin_angle + FOC_PI) ||
        end_angle < begin_angle - FOC_PI) {
        foc->encoder_direction = true;
    } else {
        foc->encoder_direction = false;
    }

    float sum_offset_angle = 0.0f;
    for (int i = 0; i < foc->pole_pairs; ++i) {
        for (int j = 0; j < 250; ++j) {
            const float angle = 2.0f * FOC_PI * (float)j / 250.0f;
            FOC_SetPhaseVoltage(foc, voltage_align * 0.6f, 0.0f, angle);
            delay(1);
        }
        FOC_SetPhaseVoltage(foc, voltage_align, 0.0f, 0.0f);
        delay(300);
        for (int j = 0; j < 100; ++j) {
            if (foc->encoder_direction) {
                sum_offset_angle += (2.0f * FOC_PI - foc->encoder->get_angle(foc->encoder)) / 100.0f;
            } else {
                sum_offset_angle += foc->encoder->get_angle(foc->encoder) / 100.0f;
            }
            delay(1);
        }
    }
    FOC_SetPhaseVoltage(foc, 0.0f, 0.0f, 0.0f);
    foc->zero_electric_angle = (sum_offset_angle - FOC_PI * (float)(foc->pole_pairs - 1U)) / (float)foc->pole_pairs;

    foc->calibrated = true;
    delay(10);
}

void FOC_Ctrl(FOC_t *foc,FOC_CtrlType ctrl_type,float value)
{
    switch(ctrl_type)
    {
        case FOC_CTRL_LOW_SPEED:    //低速模式 实际上也是作用于角度环 每次调用固定增加一个角度在前面拉着走
            foc->low_speed = value;
            PID_SetTarget(&foc->pid_angle, foc->angle); //把当前角度设置为目标值
            break;
        
        case FOC_CTRL_STEP_ANGLE:   //相对角度模式  
            if(foc->ctrl_type == ctrl_type)
            {
                PID_SetTarget(&foc->pid_angle, foc->pid_angle.target + value);   //前一次还是此模式 直接用目标值加
            }
            else
            {
                PID_SetTarget(&foc->pid_angle, foc->angle + value); //第一次进入 先用当前角度加 过渡一下
            }
            break;

        case FOC_CTRL_ANGLE:
            value = FOC_Wrap(value, 0.0f, 2.0 * FOC_PI);
            if(value - foc->angle > FOC_PI) value -= 2.0f * FOC_PI;
            else if (value - foc->angle < -FOC_PI) value += 2.0f * FOC_PI;
            PID_SetTarget(&foc->pid_angle, value);
            break;

        case FOC_CTRL_SPEED:
            value = clampf_local(value, foc->pid_angle.output_limit_n, foc->pid_angle.output_limit_p);
            PID_SetTarget(&foc->pid_speed, value);
            break;
        
        case FOC_CTRL_CURRENT:
            value = clampf_local(value, foc->pid_speed.output_limit_n, foc->pid_speed.output_limit_p);
            foc->target_iq = value;
            break;
    }

    foc->ctrl_type = ctrl_type;
}

void FOC_CtrlISR(FOC_t *foc)
{
    static float previous_angle_isr = 0.0f;

    if(!foc->enabled) return;
    if(!foc->calibrated) return;

    if(!foc->started && !foc->anticogging_calibrating)
    {
        previous_angle_isr = foc->angle;
        return;
    }

    const float angle = foc->angle;

    switch (foc->ctrl_type)
    {
        case FOC_CTRL_LOW_SPEED:    //低速转动模式 实际还是作用于角度环 而非速度环
            PID_SetTarget(&foc->pid_angle,foc->pid_angle.target + 2.0f * FOC_PI * foc->low_speed / (float)foc->ctrl_frequency / 60.0f);
            //前面预备处理函数 将当前角度值设置为目标 所以这里其实是当前角度加上小幅度角度 此处low_speed是rpm 需转化为机械角度rad
            /* 注意此处没有break 穿透效应 会一直执行后续case 实际就是将各个环串起来 */

        case FOC_CTRL_ANGLE:
        case FOC_CTRL_STEP_ANGLE:
            if(previous_angle_isr - angle > FOC_PI) foc->pid_angle.target -= 2.0f * FOC_PI;
            //此处即正转绕过零点  导致两次差值出现错误
            else if(previous_angle_isr - angle < -FOC_PI) foc->pid_angle.target += 2.0f * FOC_PI;
            //同上 此处是反转绕过零点

            PID_SetTarget(&foc->pid_speed,PID_Calc(&foc->pid_angle, angle));
            //设置速度环目标值 即位置环输出值
            /* 依旧穿透 */
        
        case FOC_CTRL_SPEED:    
            foc->target_iq = PID_Calc(&foc->pid_speed, foc->speed);
            //设置电流环目标值(未加抗齿槽补偿) 即速度环的输出

            /* 穿透 */

        //注意此处并不是真正的电流环输出函数 此处只是预处理电流环目标值 实际电流环控制输出在后
        case FOC_CTRL_CURRENT:
            if(foc->anticogging_enabled && foc->anticogging_calibrated && foc->anticogging_calibrating)
            {
                const uint16_t index = (uint16_t)(angle * (float)FOC_MAP_LEN * 0.5f / FOC_PI + 0.5f) % FOC_MAP_LEN;
                //抗齿槽补偿电流值

                PID_SetTarget(&foc->pid_current_q, foc->target_iq + foc->anticogging_map[index]);
            }

            else 
            {
                PID_SetTarget(&foc->pid_current_q, foc->target_iq);
            }
            break;
    }

    previous_angle_isr = angle;
}
/*
    真正的电流内环输出函数

*/
void FOC_LoopCtrl(FOC_t *foc)
{
    if(!foc->enabled) return;
    if(!foc->calibrated) return;

    //更新三相电流偏移量
    FOC_UpdateCurrent(foc, 
                      foc->current_sensor->iu - foc->iu_offset,
                      foc->current_sensor->iv - foc->iv_offset,
                      foc->current_sensor->iw + foc->iu_offset + foc->iv_offset);

    //限制在 0 - 2PI范围内
    foc->angle = foc->encoder_direction ? 
                 FOC_Wrap(foc->zero_electric_angle + foc->encoder->get_angle(foc->encoder), 0.0f, 2.0f * FOC_PI) :
                 FOC_Wrap(foc->zero_electric_angle - foc->encoder->get_angle(foc->encoder), 0.0f, 2.0f * FOC_PI);
                 
    foc->electrical_angle = FOC_Wrap(foc->angle * (float)foc->pole_pairs, 0.0f, 2.0f * FOC_PI);

    float temp = foc->angle - foc->previous_angle;
    if(foc->previous_angle - foc->angle > FOC_PI) temp += 2.0f * FOC_PI;
    else if(foc->previous_angle - foc->angle < -FOC_PI) temp -= 2.0f * FOC_PI;
    //处理正反转过零度的情况 算出精确速度 用于速度环反馈

    foc->speed = foc->speed_filter->apply(foc->speed_filter,temp * 60.0f * (float)foc->current_ctrl_frequency / (2.0f * FOC_PI));
    //将机械角度变化量转化为rpm并进行二阶低通滤波
    foc->previous_angle = foc->angle;

    static float ud = 0.0f;
    static float uq = 0.0f;
    if(foc->started || foc->anticogging_calibrating)
    {
        //将电流id iq输入电流环 得出ud uq
        ud = PID_Calc(&foc->pid_current_d, foc->id);
        uq = PID_Calc(&foc->pid_current_q, foc->iq);
    }
    else 
    {
        ud = 0.0f;
        uq = 0.0f;
    }
    FOC_SetPhaseVoltage(foc, ud, uq, foc->electrical_angle);
}


/*
    抗齿槽转矩校准
*/
void FOC_AnticoggingCalibrate(FOC_t *foc) {
    if (!foc->enabled) return;
    if (!foc->calibrated) return;
    if (foc->started) return;
    if (foc->anticogging_map == NULL) return;

    foc->anticogging_calibrated = false;
    FOC_Ctrl(foc, FOC_CTRL_CURRENT, 0.0f);
    foc->anticogging_calibrating = true;
    delay(5);

    uint16_t index = (uint16_t)(foc->angle / (2.0f * FOC_PI) * (float)FOC_MAP_LEN);
    FOC_Ctrl(foc, FOC_CTRL_ANGLE, 2.0f * FOC_PI * (float)index / (float)FOC_MAP_LEN);
    delay(20);

    float angle = 0.0f;
    float iq = 0.0f;
    for (int i = 0; i < (int)FOC_MAP_LEN + 30; ++i) {
        index = (uint16_t)((index + 1U) % FOC_MAP_LEN);
        const float target_angle = 2.0f * FOC_PI * (float)index / (float)FOC_MAP_LEN;
        FOC_Ctrl(foc, FOC_CTRL_ANGLE, target_angle);
        float speed = 0.3f;
        while (fabsf(angle - target_angle) > 2.0f * FOC_PI / (float)FOC_MAP_LEN / 10.0f ||
               fabsf(speed) > 0.08f) {
            float tmp = foc->angle;
            if (target_angle > 6.2f && tmp < 0.1f) tmp += 2.0f * FOC_PI;
            if (target_angle < 0.1f && tmp > 6.2f) tmp -= 2.0f * FOC_PI;
            angle = angle * 0.8f + tmp * 0.2f;
            speed = speed * 0.97f + foc->speed * 0.03f;
            iq = iq * 0.80f + foc->iq * 0.20f;
            delay(1);
        }
        foc->anticogging_map[index] = iq;
    }

    FOC_Ctrl(foc, FOC_CTRL_CURRENT, 0.0f);
    foc->anticogging_calibrating = false;
    float avg = 0.0f;
    for (uint16_t i = 0; i < FOC_MAP_LEN; ++i) avg += foc->anticogging_map[i] / (float)FOC_MAP_LEN;
    for (uint16_t i = 0; i < FOC_MAP_LEN; ++i) foc->anticogging_map[i] -= avg;
    foc->anticogging_calibrated = true;
}
