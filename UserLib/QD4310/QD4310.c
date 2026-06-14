#include "QD4310.h"
#include "BLDC_Driver.h"
#include "CurrentSensor.h"
#include "Encoder.h"
#include "FOC.h"
#include "FOC_config.h"
#include "Filter.h"
#include "PID.h"
#include "Storage.h"
#include "main.h"
#include "stm32g4xx_hal_gpio.h"
#include "stm32g4xx_hal_uart.h"
#include "usart.h"

#include <math.h>
#include <stdint.h>

#define QD4310_STORAGE_MAGIC 0xAAU
#define QD4310_ANTICOGGING_BYTES (FOC_MAP_LEN * sizeof(float))

QD4310_t qd4310;
FOC_t *foc = &qd4310.foc;

void QD4310_InitObject(QD4310_t *motor,
                       uint8_t pole_pairs,
                       uint16_t ctrl_frequency,
                       uint16_t current_ctrl_frequency,
                       Filter_t *current_q_filter,
                       Filter_t *current_d_filter,
                       Filter_t *speed_filter,
                       BLDC_Driver_t *driver,
                       Encoder_t *encoder,
                       Storage_t *storage,
                       CurrentSensor_t *current_sensor,
                       const PID_t *pid_current_q,
                       const PID_t *pid_current_d,
                       const PID_t *pid_speed,
                       const PID_t *pid_angle)
{

    FOC_InitObject(&motor->foc,
                   pole_pairs,
                   ctrl_frequency,
                   current_ctrl_frequency,
                   current_q_filter,
                   current_d_filter,
                   speed_filter,
                   driver,
                   encoder,
                   current_sensor,
                   pid_current_q,
                   pid_current_d,
                   pid_speed,
                   pid_angle);

    motor->storage = storage;
    motor->id = 0U;
    motor->uart_baud_rate = 115200U;
    motor->zero_pos = 0.0f;
}

void QD4310_Init(QD4310_t *motor)
{
    FOC_Init(&motor->foc);

    if(!motor->storage->initialized)
    {
        motor->storage->init(motor->storage);
    }

    QD4310_LoadStorageCalibration(motor);
}

void QD4310_Start(QD4310_t *motor)
{
    FOC_Start(&motor->foc);
    if(motor->foc.started) HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
}

void QD4310_Stop(QD4310_t *motor)
{
    FOC_Stop(&motor->foc);
    HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
}

void QD4310_Calibrate(QD4310_t *motor)
{
    if(!motor->foc.enabled) return;
    if(motor->foc.started) return;

    FOC_Calibrate(&motor->foc);
    if(motor->foc.calibrated)
    {
        QD4310_FreezeStorageCalibration(motor,QD4310_STORAGE_BASE_CALIBRATE_OK);
    }
}

void QD4310_AnticoggingCalibrate(QD4310_t *motor)
{
    if(!motor->foc.enabled) return;
    if(!motor->foc.calibrated) return;
    if(motor->foc.started) return;
    FOC_AnticoggingCalibrate(&motor->foc);
    if(motor->foc.anticogging_calibrated)
    {
        QD4310_FreezeStorageCalibration(motor,QD4310_STORAGE_ANTICOGGING_CALIBRATE_OK);
    }
}

float QD4310_GetAngle(const QD4310_t *motor)
{
    return FOC_Wrap(motor->foc.angle - motor->zero_pos, 0.0f, 2.0f * FOC_PI);
}

void QD4310_Ctrl(QD4310_t *motor,FOC_CtrlType ctrl_type,float value)
{
    if(ctrl_type == FOC_CTRL_ANGLE)
    {
        value = FOC_Wrap(value + motor->zero_pos, 0.0f, 2.0f * FOC_PI);
    }

    FOC_Ctrl(&motor->foc,ctrl_type,value);
}

bool QD4310_SetID(QD4310_t *motor,uint8_t id)
{
    if(id > 7U) return false;
    motor->id = id;
    return true;
}

bool QD4310_SetPID(QD4310_t *motor,float pid_speed_kp,float pid_speed_ki,float pid_speed_kd,
                                   float pid_angle_kp,float pid_angle_ki,float pid_angle_kd)
{
    if(!isnan(pid_speed_kp)) motor->foc.pid_speed.kp = pid_speed_kp;
    if(!isnan(pid_speed_ki)) motor->foc.pid_speed.ki = pid_speed_ki;
    if(!isnan(pid_speed_kd)) motor->foc.pid_speed.kd = pid_speed_kd;

    if(!isnan(pid_angle_kp)) motor->foc.pid_angle.kp = pid_angle_kp;
    if(!isnan(pid_angle_ki)) motor->foc.pid_angle.ki = pid_angle_ki;
    if(!isnan(pid_angle_kd)) motor->foc.pid_angle.kd = pid_angle_kd;

    return true;
}                   

bool QD4310_SetLimit(QD4310_t *motor,float speed_limit,float current_limit)
{
    if(!isnan(speed_limit))
    {
        motor->foc.pid_angle.output_limit_p = speed_limit;
        motor->foc.pid_angle.output_limit_n = -speed_limit;
    }
    if(!isnan(current_limit))
    {
        motor->foc.pid_speed.output_limit_p = current_limit;
        motor->foc.pid_speed.output_limit_n = -current_limit;
    }
    return true;
}

bool QD4310_SetZeroPosition(QD4310_t *motor,float position)
{
    motor->zero_pos = FOC_Wrap(motor->zero_pos + position, 0.0f, 2.0f * FOC_PI);
    QD4310_FreezeStorageCalibration(motor,QD4310_STORAGE_ZERO_POS_OK);
    return true;
}

bool QD4310_SetUartBaudRate(QD4310_t *motor,uint32_t baud_rate)
{
    if(baud_rate < 50000U || baud_rate > 10000000U) return false;
    motor->uart_baud_rate = baud_rate;
    HAL_UART_DeInit(&huart3);
    huart3.Init.BaudRate = baud_rate;
    if(HAL_UART_Init(&huart3) != HAL_OK)
    {
        Error_Handler();
    }
    return true;
}

void QD4310_RestoreCalibration(QD4310_t *motor)
{
    QD4310_SetPID(motor, FOC_SPEED_KP, FOC_SPEED_KI, FOC_SPEED_KD, FOC_ANGLE_KP, FOC_ANGLE_KI, FOC_ANGLE_KD);
    QD4310_SetLimit(motor, FOC_MAX_SPEED, FOC_MAX_CURRENT);
    QD4310_SetID(motor, 0U);
    QD4310_SetUartBaudRate(motor, 115200U);
    QD4310_FreezeStorageCalibration(motor,(QD4310_StorageStatus)(QD4310_STORAGE_PID_PARAMETER_OK | 
                                                                 QD4310_STORAGE_LIMIT_OK |
                                                                 QD4310_STORAGE_PLUG_OK));
}

void QD4310_LoadStorageCalibration(QD4310_t *motor) {
    uint8_t storage_magic = 0U;
    motor->storage->read(motor->storage, 0x000U, &storage_magic, sizeof(storage_magic));
    if (storage_magic != QD4310_STORAGE_MAGIC) return;

    QD4310_StorageStatus storage_status = QD4310_STORAGE_NONE;
    motor->storage->read(motor->storage, 0x010U, &storage_status, sizeof(storage_status));

    if ((storage_status & QD4310_STORAGE_BASE_CALIBRATE_OK) == QD4310_STORAGE_BASE_CALIBRATE_OK) {
        motor->storage->read(motor->storage, 0x100U, &motor->foc.encoder_direction, sizeof(motor->foc.encoder_direction));
        motor->storage->read(motor->storage, 0x110U, &motor->foc.zero_electric_angle, sizeof(motor->foc.zero_electric_angle));
        motor->storage->read(motor->storage, 0x120U, &motor->foc.iu_offset, sizeof(motor->foc.iu_offset));
        motor->storage->read(motor->storage, 0x130U, &motor->foc.iv_offset, sizeof(motor->foc.iv_offset));
        motor->storage->read(motor->storage, 0x140U, &motor->foc.phase_resistance, sizeof(motor->foc.phase_resistance));
        motor->storage->read(motor->storage, 0x150U, &motor->foc.phase_inductance, sizeof(motor->foc.phase_inductance));
        motor->foc.calibrated = true;
    }
    if ((storage_status & QD4310_STORAGE_ANTICOGGING_CALIBRATE_OK) == QD4310_STORAGE_ANTICOGGING_CALIBRATE_OK) {
        if (motor->foc.anticogging_map != NULL) {
            motor->storage->read(motor->storage, 0x800U, motor->foc.anticogging_map, QD4310_ANTICOGGING_BYTES);
            motor->foc.anticogging_calibrated = true;
        }
    }
    if ((storage_status & QD4310_STORAGE_PID_PARAMETER_OK) == QD4310_STORAGE_PID_PARAMETER_OK) {
        motor->storage->read(motor->storage, 0x210U, &motor->foc.pid_speed.kp, sizeof(motor->foc.pid_speed.kp));
        motor->storage->read(motor->storage, 0x220U, &motor->foc.pid_speed.ki, sizeof(motor->foc.pid_speed.ki));
        motor->storage->read(motor->storage, 0x230U, &motor->foc.pid_speed.kd, sizeof(motor->foc.pid_speed.kd));
        motor->storage->read(motor->storage, 0x240U, &motor->foc.pid_angle.kp, sizeof(motor->foc.pid_angle.kp));
        motor->storage->read(motor->storage, 0x250U, &motor->foc.pid_angle.ki, sizeof(motor->foc.pid_angle.ki));
        motor->storage->read(motor->storage, 0x260U, &motor->foc.pid_angle.kd, sizeof(motor->foc.pid_angle.kd));
    }
    if ((storage_status & QD4310_STORAGE_LIMIT_OK) == QD4310_STORAGE_LIMIT_OK) {
        motor->storage->read(motor->storage, 0x300U, &motor->foc.pid_angle.output_limit_p, sizeof(motor->foc.pid_angle.output_limit_p));
        motor->foc.pid_angle.output_limit_n = -motor->foc.pid_angle.output_limit_p;
        motor->storage->read(motor->storage, 0x310U, &motor->foc.pid_speed.output_limit_p, sizeof(motor->foc.pid_speed.output_limit_p));
        motor->foc.pid_speed.output_limit_n = -motor->foc.pid_speed.output_limit_p;
    }
    if ((storage_status & QD4310_STORAGE_PLUG_OK) == QD4310_STORAGE_PLUG_OK) {
        motor->storage->read(motor->storage, 0x400U, &motor->id, sizeof(motor->id));
        motor->storage->read(motor->storage, 0x410U, &motor->uart_baud_rate, sizeof(motor->uart_baud_rate));
        QD4310_SetUartBaudRate(motor, motor->uart_baud_rate);
    }
    if ((storage_status & QD4310_STORAGE_ZERO_POS_OK) == QD4310_STORAGE_ZERO_POS_OK) {
        motor->storage->read(motor->storage, 0x500U, &motor->zero_pos, sizeof(motor->zero_pos));
    }
}

void QD4310_FreezeStorageCalibration(QD4310_t *motor, QD4310_StorageStatus storage_type) {
    uint8_t storage_magic = 0U;
    QD4310_StorageStatus storage_status = QD4310_STORAGE_NONE;
    motor->storage->read(motor->storage, 0x000U, &storage_magic, sizeof(storage_magic));
    if (storage_magic != QD4310_STORAGE_MAGIC) {
        storage_magic = QD4310_STORAGE_MAGIC;
        motor->storage->write(motor->storage, 0x000U, &storage_magic, sizeof(storage_magic));
        motor->storage->write(motor->storage, 0x010U, &storage_status, sizeof(storage_status));
    }

    motor->storage->read(motor->storage, 0x010U, &storage_status, sizeof(storage_status));
    if ((storage_type & QD4310_STORAGE_BASE_CALIBRATE_OK) == QD4310_STORAGE_BASE_CALIBRATE_OK) {
        motor->storage->write(motor->storage, 0x100U, &motor->foc.encoder_direction, sizeof(motor->foc.encoder_direction));
        motor->storage->write(motor->storage, 0x110U, &motor->foc.zero_electric_angle, sizeof(motor->foc.zero_electric_angle));
        motor->storage->write(motor->storage, 0x120U, &motor->foc.iu_offset, sizeof(motor->foc.iu_offset));
        motor->storage->write(motor->storage, 0x130U, &motor->foc.iv_offset, sizeof(motor->foc.iv_offset));
        motor->storage->write(motor->storage, 0x140U, &motor->foc.phase_resistance, sizeof(motor->foc.phase_resistance));
        motor->storage->write(motor->storage, 0x150U, &motor->foc.phase_inductance, sizeof(motor->foc.phase_inductance));
    }
    if ((storage_type & QD4310_STORAGE_ANTICOGGING_CALIBRATE_OK) == QD4310_STORAGE_ANTICOGGING_CALIBRATE_OK) {
        if (motor->foc.anticogging_map != NULL) {
            motor->storage->write(motor->storage, 0x800U, motor->foc.anticogging_map, QD4310_ANTICOGGING_BYTES);
        }
    }
    if ((storage_type & QD4310_STORAGE_PID_PARAMETER_OK) == QD4310_STORAGE_PID_PARAMETER_OK) {
        motor->storage->write(motor->storage, 0x210U, &motor->foc.pid_speed.kp, sizeof(motor->foc.pid_speed.kp));
        motor->storage->write(motor->storage, 0x220U, &motor->foc.pid_speed.ki, sizeof(motor->foc.pid_speed.ki));
        motor->storage->write(motor->storage, 0x230U, &motor->foc.pid_speed.kd, sizeof(motor->foc.pid_speed.kd));
        motor->storage->write(motor->storage, 0x240U, &motor->foc.pid_angle.kp, sizeof(motor->foc.pid_angle.kp));
        motor->storage->write(motor->storage, 0x250U, &motor->foc.pid_angle.ki, sizeof(motor->foc.pid_angle.ki));
        motor->storage->write(motor->storage, 0x260U, &motor->foc.pid_angle.kd, sizeof(motor->foc.pid_angle.kd));
    }
    if ((storage_type & QD4310_STORAGE_LIMIT_OK) == QD4310_STORAGE_LIMIT_OK) {
        motor->storage->write(motor->storage, 0x300U, &motor->foc.pid_angle.output_limit_p, sizeof(motor->foc.pid_angle.output_limit_p));
        motor->storage->write(motor->storage, 0x310U, &motor->foc.pid_speed.output_limit_p, sizeof(motor->foc.pid_speed.output_limit_p));
    }
    if ((storage_type & QD4310_STORAGE_PLUG_OK) == QD4310_STORAGE_PLUG_OK) {
        motor->storage->write(motor->storage, 0x400U, &motor->id, sizeof(motor->id));
        motor->storage->write(motor->storage, 0x410U, &motor->uart_baud_rate, sizeof(motor->uart_baud_rate));
    }
    if ((storage_type & QD4310_STORAGE_ZERO_POS_OK) == QD4310_STORAGE_ZERO_POS_OK) {
        motor->storage->write(motor->storage, 0x500U, &motor->zero_pos, sizeof(motor->zero_pos));
    }

    storage_status = (QD4310_StorageStatus)(storage_status | storage_type);
    motor->storage->write(motor->storage, 0x010U, &storage_status, sizeof(storage_status));
}

