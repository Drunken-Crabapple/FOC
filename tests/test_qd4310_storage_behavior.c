#include "QD4310.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

UART_HandleTypeDef huart3;

int HAL_UART_DeInit(UART_HandleTypeDef *huart) {
    (void)huart;
    return HAL_OK;
}

int HAL_UART_Init(UART_HandleTypeDef *huart) {
    (void)huart;
    return HAL_OK;
}

static float passthrough_filter(Filter_t *filter, float value) {
    (void)filter;
    return value;
}

static float filter_ts(Filter_t *filter) {
    (void)filter;
    return 0.00005f;
}

static void driver_noop(BLDC_Driver_t *driver) {
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

static void encoder_noop(Encoder_t *encoder) {
    encoder->initialized = true;
}

static void encoder_enable(Encoder_t *encoder) {
    encoder->enabled = true;
}

static void encoder_disable(Encoder_t *encoder) {
    encoder->enabled = false;
}

static float encoder_angle(Encoder_t *encoder) {
    (void)encoder;
    return 0.0f;
}

static void sensor_noop(CurrentSensor_t *sensor) {
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

typedef struct {
    uint8_t bytes[0x900];
    uint32_t anticogging_write_count;
    uint8_t final_status;
} FakeStorage;

static void storage_init(Storage_t *storage) {
    storage->initialized = true;
}

static void storage_write(Storage_t *storage, uint32_t addr, const void *buff, uint32_t count) {
    FakeStorage *fake = (FakeStorage *)storage->context;
    if (addr + count <= sizeof(fake->bytes)) {
        memcpy(fake->bytes + addr, buff, count);
    }
    if (addr == 0x800U) {
        fake->anticogging_write_count = count;
    }
    if (addr == 0x010U && count == sizeof(uint8_t)) {
        fake->final_status = *(const uint8_t *)buff;
    }
}

static void storage_read(Storage_t *storage, uint32_t addr, void *buff, uint32_t count) {
    FakeStorage *fake = (FakeStorage *)storage->context;
    if (addr + count <= sizeof(fake->bytes)) {
        memcpy(buff, fake->bytes + addr, count);
    } else {
        memset(buff, 0, count);
    }
}

static QD4310_t make_motor(Storage_t *storage) {
    static Filter_t current_q_filter = {passthrough_filter, filter_ts, NULL};
    static Filter_t current_d_filter = {passthrough_filter, filter_ts, NULL};
    static Filter_t speed_filter = {passthrough_filter, filter_ts, NULL};
    static BLDC_Driver_t driver = {
        false, false, driver_noop, driver_enable, driver_disable, driver_set_duty, NULL
    };
    static Encoder_t encoder = {
        false, false, encoder_noop, encoder_enable, encoder_disable, encoder_angle, NULL
    };
    static CurrentSensor_t sensor = {
        false, false, 0.0f, 0.0f, 0.0f, sensor_noop, sensor_enable, sensor_disable, sensor_update, NULL
    };

    PID_t pid_current_q;
    PID_t pid_current_d;
    PID_t pid_speed;
    PID_t pid_angle;
    PID_Init(&pid_current_q, PID_DELTA_TYPE, 1.0f, 2.0f, 3.0f, NAN, NAN, 1.0f, -1.0f);
    PID_Init(&pid_current_d, PID_DELTA_TYPE, 1.0f, 2.0f, 3.0f, NAN, NAN, 1.0f, -1.0f);
    PID_Init(&pid_speed, PID_POSITION_TYPE, 1.0f, 2.0f, 3.0f, NAN, NAN, 1.0f, -1.0f);
    PID_Init(&pid_angle, PID_POSITION_TYPE, 1.0f, 2.0f, 3.0f, NAN, NAN, 1.0f, -1.0f);

    QD4310_t motor;
    QD4310_InitObject(&motor, 14, 5000, 20000,
                      &current_q_filter, &current_d_filter, &speed_filter,
                      &driver, &encoder, storage, &sensor,
                      &pid_current_q, &pid_current_d, &pid_speed, &pid_angle);
    return motor;
}

int main(void) {
    FakeStorage fake = {0};
    Storage_t storage = {false, sizeof(fake.bytes), storage_init, storage_write, storage_read, &fake};
    QD4310_t motor = make_motor(&storage);

    QD4310_FreezeStorageCalibration(&motor, QD4310_STORAGE_ALL_OK);

    if (fake.final_status != QD4310_STORAGE_ALL_OK) {
        fprintf(stderr, "expected final status 0x%02x, got 0x%02x\n",
                QD4310_STORAGE_ALL_OK, fake.final_status);
        return 1;
    }
    if (fake.anticogging_write_count != sizeof(motor.foc.anticogging_map)) {
        fprintf(stderr, "expected anticogging write count %u, got %lu\n",
                (unsigned)sizeof(motor.foc.anticogging_map),
                (unsigned long)fake.anticogging_write_count);
        return 1;
    }
    return 0;
}
