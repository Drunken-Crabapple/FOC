/**
 * @file        FOC_config.h
 * @brief       用于定义FOC控制器的配置常量
 */

#ifndef FOC_CONFIG_H
#define FOC_CONFIG_H

#define FOC_HARDWARE_VERSION "4310_6.1.0"
#define FOC_SOFTWARE_VERSION "6.2.0"

/* 电机参数 */
#define FOC_KV                  33.0f
#define FOC_POLE_PAIRS          14
#define FOC_NOMINAL_VOLTAGE     24
#define FOC_PHASE_INDUCTANCE    4.74f
#define FOC_PHASE_RESISTANCE    10.9f
#define FOC_TORQUE_CONSTANT     0.27f

/* 驱动板参数 */
#define FOC_MAX_CURRENT         1.65f

/* 配置参数 */
#define FOC_MAX_SPEED           1000.0f

#define FOC_CURRENT_KP          10.0f
#define FOC_CURRENT_KI          1.0f
#define FOC_CURRENT_KD          0.0f
#define FOC_SPEED_KP            3e-3f
#define FOC_SPEED_KI            7.8e-5f
#define FOC_SPEED_KD            0.0f
#define FOC_ANGLE_KP            1200.0f
#define FOC_ANGLE_KI            0.0f
#define FOC_ANGLE_KD            0.0f

#endif
