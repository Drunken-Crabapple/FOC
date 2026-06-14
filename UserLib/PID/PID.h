/**
 * @file PID.h
 * @brief C PID controller.
 */

#ifndef PID_H
#define PID_H

#include <math.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    PID_POSITION_TYPE = 0,
    PID_DELTA_TYPE = 1,
} PID_Type;

typedef struct {
    PID_Type type;
    float target;
    float kp;
    float ki;
    float kd;
    float sum_error_limit_p;
    float sum_error_limit_n;
    float output_limit_p;
    float output_limit_n;

    float error;
    float pre_error;
    float sum_error;
    float output;
} PID_t;

static inline void PID_Init(PID_t *pid, PID_Type type, float kp, float ki, float kd,
                            float sum_error_limit_p, float sum_error_limit_n,
                            float output_limit_p, float output_limit_n) {
    if (pid == NULL) return;
    pid->type = type;
    pid->target = 0.0f;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->sum_error_limit_p = sum_error_limit_p;
    pid->sum_error_limit_n = sum_error_limit_n;
    pid->output_limit_p = output_limit_p;
    pid->output_limit_n = output_limit_n;
    pid->error = 0.0f;
    pid->pre_error = 0.0f;
    pid->sum_error = 0.0f;
    pid->output = 0.0f;
}

static inline void PID_SetTarget(PID_t *pid, float target) {
    if (pid == NULL) return;
    pid->target = target;
}

static inline void PID_SetSumError(PID_t *pid, float sum_error) {
    if (pid == NULL) return;
    pid->sum_error = sum_error;
}

static inline float PID_Calc(PID_t *pid, float input) {
    if (pid == NULL) return 0.0f;

    const float error_ = pid->target - input;
    if (pid->type == PID_POSITION_TYPE) {
        pid->sum_error += error_;
        if (!isnan(pid->sum_error_limit_p) && pid->sum_error >= pid->sum_error_limit_p) {
            pid->sum_error = pid->sum_error_limit_p;
        }
        if (!isnan(pid->sum_error_limit_n) && pid->sum_error <= pid->sum_error_limit_n) {
            pid->sum_error = pid->sum_error_limit_n;
        }
        pid->output = pid->kp * error_ +
                      pid->ki * pid->sum_error +
                      pid->kd * (error_ - pid->error);
        pid->error = error_;
    } else {
        pid->output += pid->kp * (error_ - pid->error) +
                       pid->ki * error_ +
                       pid->kd * (error_ - 2.0f * pid->error + pid->pre_error);
        pid->pre_error = pid->error;
        pid->error = error_;
    }

    if (!isnan(pid->output_limit_p) && pid->output >= pid->output_limit_p) {
        pid->output = pid->output_limit_p;
    }
    if (!isnan(pid->output_limit_n) && pid->output <= pid->output_limit_n) {
        pid->output = pid->output_limit_n;
    }
    return pid->output;
}

#ifdef __cplusplus
}
#endif

#endif
