#include "FOC.h"

#include <math.h>
#include <stdio.h>

static float passthrough_filter(Filter_t *filter, float value) {
    (void)filter;
    return value;
}

static float filter_ts(Filter_t *filter) {
    (void)filter;
    return 0.00005f;
}

static void driver_init(BLDC_Driver_t *driver) {
    driver->initialized = true;
}

static void driver_enable(BLDC_Driver_t *driver) {
    driver->enabled = true;
}

static void driver_disable(BLDC_Driver_t *driver) {
    driver->enabled = false;
}

static void driver_set_duty(BLDC_Driver_t *driver, float u, float v, float w) {
    (void)driver;
    (void)u;
    (void)v;
    (void)w;
}

static void encoder_init(Encoder_t *encoder) {
    encoder->initialized = true;
}

static void encoder_enable(Encoder_t *encoder) {
    encoder->enabled = true;
}

static void encoder_disable(Encoder_t *encoder) {
    encoder->enabled = false;
}

static float encoder_get_angle(Encoder_t *encoder) {
    (void)encoder;
    return 0.0f;
}

static void sensor_init(CurrentSensor_t *sensor) {
    sensor->initialized = true;
}

static void sensor_enable(CurrentSensor_t *sensor) {
    sensor->enabled = true;
}

static void sensor_disable(CurrentSensor_t *sensor) {
    sensor->enabled = false;
}

static void sensor_update(CurrentSensor_t *sensor) {
    (void)sensor;
}

static FOC_t make_foc(void) {
    static Filter_t current_q_filter = {passthrough_filter, filter_ts, NULL};
    static Filter_t current_d_filter = {passthrough_filter, filter_ts, NULL};
    static Filter_t speed_filter = {passthrough_filter, filter_ts, NULL};
    static BLDC_Driver_t driver = {
        false, false, driver_init, driver_enable, driver_disable, driver_set_duty, NULL
    };
    static Encoder_t encoder = {
        false, false, encoder_init, encoder_enable, encoder_disable, encoder_get_angle, NULL
    };
    static CurrentSensor_t sensor = {
        false, false, 0.0f, 0.0f, 0.0f, sensor_init, sensor_enable, sensor_disable, sensor_update, NULL
    };

    PID_t pid_current_q;
    PID_t pid_current_d;
    PID_t pid_speed;
    PID_t pid_angle;
    PID_Init(&pid_current_q, PID_DELTA_TYPE, 0.0f, 0.0f, 0.0f, NAN, NAN, 1.0f, -1.0f);
    PID_Init(&pid_current_d, PID_DELTA_TYPE, 0.0f, 0.0f, 0.0f, NAN, NAN, 1.0f, -1.0f);
    PID_Init(&pid_speed, PID_POSITION_TYPE, 0.0f, 0.0f, 0.0f, NAN, NAN, 1.0f, -1.0f);
    PID_Init(&pid_angle, PID_POSITION_TYPE, 0.0f, 0.0f, 0.0f, NAN, NAN, 1.0f, -1.0f);

    FOC_t foc;
    FOC_InitObject(&foc, 7, 5000, 20000,
                   &current_q_filter, &current_d_filter, &speed_filter,
                   &driver, &encoder, &sensor,
                   &pid_current_q, &pid_current_d, &pid_speed, &pid_angle);
    return foc;
}

static int anticogging_calibrate_returns_when_map_is_null(void) {
    FOC_t foc = make_foc();
    foc.enabled = true;
    foc.calibrated = true;
    foc.anticogging_calibrated = true;
    foc.anticogging_map = NULL;

    FOC_AnticoggingCalibrate(&foc);

    if (!foc.anticogging_calibrated) {
        fprintf(stderr, "anticogging_calibrated changed when map was NULL\n");
        return 1;
    }
    if (foc.anticogging_calibrating) {
        fprintf(stderr, "anticogging_calibrating started when map was NULL\n");
        return 1;
    }
    return 0;
}

int main(void) {
    return anticogging_calibrate_returns_when_map_is_null();
}
