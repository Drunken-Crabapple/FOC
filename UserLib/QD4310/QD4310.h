/**
 * @file QD4310.h
 * @brief C product wrapper around the FOC core.
 */

#ifndef QD4310_H
#define QD4310_H

#include "FOC.h"
#include "Storage.h"
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef uint8_t QD4310_StorageStatus;

enum {
    QD4310_STORAGE_NONE = 0x00,
    QD4310_STORAGE_BASE_CALIBRATE_OK = 0x01,
    QD4310_STORAGE_ANTICOGGING_CALIBRATE_OK = 0x02,
    QD4310_STORAGE_PID_PARAMETER_OK = 0x04,
    QD4310_STORAGE_LIMIT_OK = 0x08,
    QD4310_STORAGE_PLUG_OK = 0x10,
    QD4310_STORAGE_ZERO_POS_OK = 0x20,
    QD4310_STORAGE_ALL_OK = QD4310_STORAGE_BASE_CALIBRATE_OK |
                             QD4310_STORAGE_ANTICOGGING_CALIBRATE_OK |
                             QD4310_STORAGE_PID_PARAMETER_OK |
                             QD4310_STORAGE_LIMIT_OK |
                             QD4310_STORAGE_PLUG_OK |
                             QD4310_STORAGE_ZERO_POS_OK,
};

typedef struct {
    FOC_t foc;
    Storage_t *storage;
    uint8_t id;
    uint32_t uart_baud_rate;
    float zero_pos;
} QD4310_t;

extern QD4310_t qd4310;
extern FOC_t *foc;

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
                       const PID_t *pid_angle);

void QD4310_Init(QD4310_t *motor);
void QD4310_Start(QD4310_t *motor);
void QD4310_Stop(QD4310_t *motor);
void QD4310_Calibrate(QD4310_t *motor);
void QD4310_AnticoggingCalibrate(QD4310_t *motor);
float QD4310_GetAngle(const QD4310_t *motor);
void QD4310_Ctrl(QD4310_t *motor, FOC_CtrlType ctrl_type, float value);
bool QD4310_SetID(QD4310_t *motor, uint8_t id);
bool QD4310_SetPID(QD4310_t *motor, float pid_speed_kp, float pid_speed_ki, float pid_speed_kd,
                   float pid_angle_kp, float pid_angle_ki, float pid_angle_kd);
bool QD4310_SetLimit(QD4310_t *motor, float speed_limit, float current_limit);
bool QD4310_SetZeroPosition(QD4310_t *motor, float position);
bool QD4310_SetUartBaudRate(QD4310_t *motor, uint32_t baud_rate);
void QD4310_RestoreCalibration(QD4310_t *motor);
void QD4310_LoadStorageCalibration(QD4310_t *motor);
void QD4310_FreezeStorageCalibration(QD4310_t *motor, QD4310_StorageStatus storage_type);

#ifdef __cplusplus
}
#endif

#endif
