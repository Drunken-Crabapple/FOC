#include "FOC.h"
#include "FOC_config.h"
#include "sys_public.h"
#include <string.h>
#include <stdlib.h>

static float clampf_local(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

float FOC_Wrap(float value, float min, float max) {
    value = fmodf(value - min, max - min);
    return value < 0.0f ? value + max : value + min;
}

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
                    const PID_t *pid_angle) {
    memset(foc, 0, sizeof(*foc));
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
    foc->encoder_direction = true;
    foc->phase_resistance = NAN;
    foc->phase_inductance = NAN;
    foc->voltage = 1.0f;
    foc->ctrl_type = FOC_CTRL_CURRENT;
}

void FOC_Init(FOC_t *foc) {
    if (!foc->driver->initialized) foc->driver->init(foc->driver);
    if (!foc->encoder->initialized) foc->encoder->init(foc->encoder);
    if (!foc->current_sensor->initialized) foc->current_sensor->init(foc->current_sensor);
    foc->initialized = true;
}

void FOC_Enable(FOC_t *foc) {
    if (!foc->initialized) return;
    if (!foc->driver->enabled) {
        foc->driver->enable(foc->driver);
        foc->driver->set_duty(foc->driver, 0.0f, 0.0f, 0.0f);
    }
    if (!foc->encoder->enabled) foc->encoder->enable(foc->encoder);
    if (!foc->current_sensor->enabled) foc->current_sensor->enable(foc->current_sensor);
    foc->enabled = true;
}

void FOC_Disable(FOC_t *foc) {
    if (foc->driver->enabled) {
        foc->driver->set_duty(foc->driver, 0.0f, 0.0f, 0.0f);
        foc->driver->disable(foc->driver);
    }
    if (foc->encoder->enabled) foc->encoder->disable(foc->encoder);
    foc->enabled = false;
}

void FOC_Start(FOC_t *foc) {
    if (foc->enabled && foc->calibrated) foc->started = true;
}

void FOC_Stop(FOC_t *foc) {
    foc->started = false;
}

static void FOC_UpdateCurrent(FOC_t *foc, float iu, float iv, float iw) {
    foc->iu = iu;
    foc->iv = iv;
    foc->iw = iw;

    foc->ia = foc->iu;
    foc->ib = (foc->iu + 2.0f * foc->iv) * 0.5773502691896258f;

    const float cos_angle = cosf(foc->electrical_angle);
    const float sin_angle = sinf(foc->electrical_angle);
    foc->iq = foc->current_q_filter->apply(
        foc->current_q_filter,
        foc->ib * cos_angle - foc->ia * sin_angle);
    foc->id = foc->current_d_filter->apply(
        foc->current_d_filter,
        foc->ib * sin_angle + foc->ia * cos_angle);
}

static void FOC_SetPhaseVoltage(FOC_t *foc, float ud, float uq, float electrical_angle) {
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

    foc->driver->set_duty(foc->driver, foc->uu, foc->uv, foc->uw);
}

void FOC_UpdateVoltage(FOC_t *foc, float voltage) {
    if (voltage == 0.0f) return;
    foc->pid_current_q.kp *= foc->voltage / voltage;
    foc->pid_current_d.kp *= foc->voltage / voltage;
    foc->pid_current_q.ki *= foc->voltage / voltage;
    foc->pid_current_d.ki *= foc->voltage / voltage;
    foc->voltage = voltage;
}

void FOC_CurrentCalibrate(FOC_t *foc) {
    if (!foc->enabled) return;
    if (foc->started) return;
    const bool calibrate_status = foc->calibrated;
    foc->calibrated = false;

    FOC_SetPhaseVoltage(foc, 0.0f, 0.0f, 0.0f);
    delay(30);
    foc->iu_offset = 0.0f;
    foc->iv_offset = 0.0f;
    for (int i = 0; i < 200; ++i) {
        foc->iu_offset += foc->current_sensor->iu / 200.0f;
        foc->iv_offset += foc->current_sensor->iv / 200.0f;
        delay(1);
    }
    foc->calibrated = calibrate_status;
}

void FOC_Calibrate(FOC_t *foc) {
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

void FOC_Ctrl(FOC_t *foc, FOC_CtrlType ctrl_type, float value) {
    switch (ctrl_type) {
        case FOC_CTRL_LOW_SPEED:
            foc->low_speed = value;
            PID_SetTarget(&foc->pid_angle, foc->angle);
            break;
        case FOC_CTRL_STEP_ANGLE:
            if (foc->ctrl_type == ctrl_type) {
                PID_SetTarget(&foc->pid_angle, foc->pid_angle.target + value);
            } else {
                PID_SetTarget(&foc->pid_angle, foc->angle + value);
            }
            break;
        case FOC_CTRL_ANGLE:
            value = FOC_Wrap(value, 0.0f, 2.0f * FOC_PI);
            if (value - foc->angle > FOC_PI) value -= 2.0f * FOC_PI;
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

void FOC_CtrlISR(FOC_t *foc) {
    static float previous_angle_isr = 0.0f;
    if (!foc->enabled) return;
    if (!foc->calibrated) return;
    if (!foc->started && !foc->anticogging_calibrating) {
        previous_angle_isr = foc->angle;
        return;
    }
    const float angle = foc->angle;

    switch (foc->ctrl_type) {
        case FOC_CTRL_LOW_SPEED:
            PID_SetTarget(&foc->pid_angle,
                          foc->pid_angle.target + 2.0f * FOC_PI * foc->low_speed /
                          (float)foc->ctrl_frequency / 60.0f);
            /* fall through */
        case FOC_CTRL_ANGLE:
        case FOC_CTRL_STEP_ANGLE:
            if (previous_angle_isr - angle > FOC_PI) foc->pid_angle.target -= 2.0f * FOC_PI;
            else if (previous_angle_isr - angle < -FOC_PI) foc->pid_angle.target += 2.0f * FOC_PI;
            PID_SetTarget(&foc->pid_speed, PID_Calc(&foc->pid_angle, angle));
            /* fall through */
        case FOC_CTRL_SPEED:
            foc->target_iq = PID_Calc(&foc->pid_speed, foc->speed);
            /* fall through */
        case FOC_CTRL_CURRENT:
            if (foc->anticogging_enabled && foc->anticogging_calibrated && !foc->anticogging_calibrating) {
                const uint16_t index = (uint16_t)(angle * (float)FOC_MAP_LEN * 0.5f / FOC_PI + 0.5f) % FOC_MAP_LEN;
                PID_SetTarget(&foc->pid_current_q, foc->target_iq + foc->anticogging_map[index]);
            } else {
                PID_SetTarget(&foc->pid_current_q, foc->target_iq);
            }
            break;
    }
    previous_angle_isr = angle;
}

void FOC_LoopCtrl(FOC_t *foc) {
    if (!foc->enabled) return;
    if (!foc->calibrated) return;

    FOC_UpdateCurrent(foc,
                      foc->current_sensor->iu - foc->iu_offset,
                      foc->current_sensor->iv - foc->iv_offset,
                      foc->current_sensor->iw + foc->iu_offset + foc->iv_offset);

    foc->angle = foc->encoder_direction ?
                 FOC_Wrap(foc->zero_electric_angle + foc->encoder->get_angle(foc->encoder), 0.0f, 2.0f * FOC_PI) :
                 FOC_Wrap(foc->zero_electric_angle - foc->encoder->get_angle(foc->encoder), 0.0f, 2.0f * FOC_PI);
    foc->electrical_angle = FOC_Wrap(foc->angle * (float)foc->pole_pairs, 0.0f, 2.0f * FOC_PI);

    float temp = foc->angle - foc->previous_angle;
    if (foc->previous_angle - foc->angle > FOC_PI) temp += 2.0f * FOC_PI;
    else if (foc->previous_angle - foc->angle < -FOC_PI) temp -= 2.0f * FOC_PI;
    foc->speed = foc->speed_filter->apply(
        foc->speed_filter,
        temp * 60.0f * (float)foc->current_ctrl_frequency / (2.0f * FOC_PI));
    foc->previous_angle = foc->angle;

    static float ud = 0.0f;
    static float uq = 0.0f;
    if (foc->started || foc->anticogging_calibrating) {
        ud = PID_Calc(&foc->pid_current_d, foc->id);
        uq = PID_Calc(&foc->pid_current_q, foc->iq);
    } else {
        ud = 0.0f;
        uq = 0.0f;
    }
    FOC_SetPhaseVoltage(foc, ud, uq, foc->electrical_angle);
}

void FOC_AnticoggingCalibrate(FOC_t *foc) {
    if (!foc->enabled) return;
    if (!foc->calibrated) return;
    if (foc->started) return;

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
