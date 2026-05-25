/**
 * @file FOC.h
 * @brief C FOC controller core.
 */

#ifndef FOC_H
#define FOC_H

#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "BLDC_Driver.h"
#include "CurrentSensor.h"
#include "Encoder.h"
#include "Filter.h"
#include "PID.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef FOC_PI
#define FOC_PI 3.14159265358979323846f
#endif

#ifndef FOC_MAP_LEN
#define FOC_MAP_LEN 2000U
#endif

typedef enum {
    FOC_CTRL_CURRENT = 0,
    FOC_CTRL_SPEED = 1,
    FOC_CTRL_ANGLE = 2,
    FOC_CTRL_STEP_ANGLE = 3,
    FOC_CTRL_LOW_SPEED = 4,
} FOC_CtrlType;

typedef struct {
    uint8_t pole_pairs;
    uint16_t ctrl_frequency;
    uint16_t current_ctrl_frequency;

    bool initialized;
    bool enabled;
    bool started;
    bool calibrated;
    bool anticogging_enabled;
    bool anticogging_calibrated;
    bool anticogging_calibrating;

    FOC_CtrlType ctrl_type;

    PID_t pid_current_q;
    PID_t pid_current_d;
    PID_t pid_speed;
    PID_t pid_angle;

    BLDC_Driver_t *driver;
    Encoder_t *encoder;
    CurrentSensor_t *current_sensor;
    Filter_t *current_q_filter;
    Filter_t *current_d_filter;
    Filter_t *speed_filter;

    bool encoder_direction;
    float phase_resistance;
    float phase_inductance;
    float iu_offset;
    float iv_offset;
    float zero_electric_angle;
    float anticogging_map[FOC_MAP_LEN];

    float target_iq;
    float angle;
    float previous_angle;
    float electrical_angle;
    float speed;
    float low_speed;

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
void FOC_Calibrate(FOC_t *foc);
void FOC_CurrentCalibrate(FOC_t *foc);
void FOC_AnticoggingCalibrate(FOC_t *foc);
void FOC_Ctrl(FOC_t *foc, FOC_CtrlType ctrl_type, float value);
void FOC_CtrlISR(FOC_t *foc);
void FOC_LoopCtrl(FOC_t *foc);
void FOC_UpdateVoltage(FOC_t *foc, float voltage);
float FOC_Wrap(float value, float min, float max);

#ifdef __cplusplus
}
#endif

#endif
