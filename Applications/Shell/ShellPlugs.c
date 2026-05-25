#include <string.h>
#include <math.h>
#include <stdio.h>
#include "shell.h"
#include "retarget/retarget.h"
#include "QD4310.h"
#include "FOC_config.h"
#include "main.h"

extern Shell shell;

static float atof_lite(const char *s) {
    if (!s) return 0.0f;
    int sign = 1;
    if (*s == '+') ++s;
    else if (*s == '-') {
        sign = -1;
        ++s;
    }
    float int_part = 0.0f;
    bool has_digit = false;
    while (*s >= '0' && *s <= '9') {
        has_digit = true;
        int_part = int_part * 10.0f + (float)(*s - '0');
        ++s;
    }
    float frac_part = 0.0f;
    float scale = 1.0f;
    if (*s == '.') {
        ++s;
        while (*s >= '0' && *s <= '9') {
            has_digit = true;
            frac_part = frac_part * 10.0f + (float)(*s - '0');
            scale *= 10.0f;
            ++s;
        }
    }
    if (!has_digit) return 0.0f;
    const float result = int_part + frac_part / scale;
    return sign < 0 ? -result : result;
}

static signed short silent(char *data, unsigned short len) {
    (void)data;
    (void)len;
    return 0;
}

#define PRINT(...) do { if (shell.write != silent) { printf(__VA_ARGS__); printf("\r\n"); } } while (0)

int print_version(int argc, char *argv[]) {
    (void)argc;
    (void)argv;
    PRINT("Hardware version %s", FOC_HARDWARE_VERSION);
    PRINT("Software version %s", FOC_SOFTWARE_VERSION);
    return 0;
}

int foc_info(int argc, char *argv[]) {
    (void)argc;
    (void)argv;
    PRINT("Hardware Info:");
    PRINT("  Pole pairs       : %d ", FOC_POLE_PAIRS);
    PRINT("  KV rating        : %.1f rpm/V", FOC_KV);
    PRINT("  Nominal voltage  : %d V", FOC_NOMINAL_VOLTAGE);
    PRINT("  Phase inductance : %.2f mH", FOC_PHASE_INDUCTANCE);
    PRINT("  Phase resistance : %.2f ohm", FOC_PHASE_RESISTANCE);
    PRINT("  Torque constant  : %.2f Nm/A", FOC_TORQUE_CONSTANT);
    PRINT("  Max current      : %.2f A", FOC_MAX_CURRENT);
    return 0;
}

int foc_status(int argc, char *argv[]) {
    (void)argc;
    (void)argv;
    PRINT("Motor Status:");
    PRINT("  CAN ID       : %03d", qd4310.id);
    PRINT("  Status       : %s", qd4310.foc.started ? "enabled" : "disabled");
    PRINT("  CtrlMode     : %d", qd4310.foc.ctrl_type);
    PRINT("  Current      : %.2f A", qd4310.foc.iq);
    PRINT("  Speed        : %.2f rpm", qd4310.foc.speed);
    PRINT("  Angle        : %.2f rad", QD4310_GetAngle(&qd4310));
    PRINT("  Voltage      : %.2f V", qd4310.foc.voltage);
    return 0;
}

void foc_config_list(void) {
    PRINT("Current Configuration:");
    PRINT("pid.speed.kp = %.3g", qd4310.foc.pid_speed.kp);
    PRINT("pid.speed.ki = %.3g", qd4310.foc.pid_speed.ki);
    PRINT("pid.speed.kd = %.3g", qd4310.foc.pid_speed.kd);
    PRINT("pid.angle.kp = %.3g", qd4310.foc.pid_angle.kp);
    PRINT("pid.angle.ki = %.3g", qd4310.foc.pid_angle.ki);
    PRINT("pid.angle.kd = %.3g", qd4310.foc.pid_angle.kd);
    PRINT("limit.speed = %.3g rpm", qd4310.foc.pid_angle.output_limit_p);
    PRINT("limit.current = %.3g A", qd4310.foc.pid_speed.output_limit_p);
    PRINT("can.id = %03d", qd4310.id);
    PRINT("uart.baud_rate = %lu", qd4310.uart_baud_rate);
}

int foc_config(int argc, char *argv[]) {
    if (argc < 2 || strcmp(argv[1], "--list") == 0) {
        foc_config_list();
        return 0;
    }
    const char *key = argv[1];
    const char *value = NULL;
    char keybuf[128];
    const char *eq = strchr(key, '=');
    if (eq != NULL) {
        strncpy(keybuf, key, sizeof(keybuf) - 1U);
        keybuf[sizeof(keybuf) - 1U] = '\0';
        char *mutable_eq = strchr(keybuf, '=');
        *mutable_eq = '\0';
        key = keybuf;
        value = mutable_eq + 1;
    } else if (argc >= 3) {
        value = argv[2];
    }

    if (strcmp(key, "zero_pos") == 0) {
        QD4310_SetZeroPosition(&qd4310, value ? atof_lite(value) : QD4310_GetAngle(&qd4310));
        PRINT("Setting config [zero_pos]");
        return 0;
    }
    if (!value) {
        PRINT("Missing value for config [%s]", key);
        return 0;
    }
    float valf = atof_lite(value);
    if (strcmp(key, "pid.speed.kp") == 0) QD4310_SetPID(&qd4310, valf, NAN, NAN, NAN, NAN, NAN);
    else if (strcmp(key, "pid.speed.ki") == 0) QD4310_SetPID(&qd4310, NAN, valf, NAN, NAN, NAN, NAN);
    else if (strcmp(key, "pid.speed.kd") == 0) QD4310_SetPID(&qd4310, NAN, NAN, valf, NAN, NAN, NAN);
    else if (strcmp(key, "pid.angle.kp") == 0) QD4310_SetPID(&qd4310, NAN, NAN, NAN, valf, NAN, NAN);
    else if (strcmp(key, "pid.angle.ki") == 0) QD4310_SetPID(&qd4310, NAN, NAN, NAN, NAN, valf, NAN);
    else if (strcmp(key, "pid.angle.kd") == 0) QD4310_SetPID(&qd4310, NAN, NAN, NAN, NAN, NAN, valf);
    else if (strcmp(key, "limit.speed") == 0) QD4310_SetLimit(&qd4310, valf, NAN);
    else if (strcmp(key, "limit.current") == 0) QD4310_SetLimit(&qd4310, NAN, valf);
    else if (strcmp(key, "can.id") == 0) QD4310_SetID(&qd4310, (uint8_t)valf);
    else if (strcmp(key, "uart.baud_rate") == 0) QD4310_SetUartBaudRate(&qd4310, (uint32_t)valf);
    else {
        PRINT("Unknown config target: %s", key);
        return 0;
    }
    PRINT("Setting config [%s] = %.3g", key, valf);
    return 0;
}

int foc_ctrl(int argc, char *argv[]) {
    if (argc < 3) {
        PRINT("Usage: ctrl [current|low_speed|speed|step_angle|angle] VALUE");
        return 0;
    }
    const char *key = argv[1];
    float valf = atof_lite(argv[2]);
    if (strcmp(key, "current") == 0) QD4310_Ctrl(&qd4310, FOC_CTRL_CURRENT, valf);
    else if (strcmp(key, "speed") == 0) QD4310_Ctrl(&qd4310, FOC_CTRL_SPEED, valf);
    else if (strcmp(key, "angle") == 0) QD4310_Ctrl(&qd4310, FOC_CTRL_ANGLE, valf);
    else if (strcmp(key, "step_angle") == 0) QD4310_Ctrl(&qd4310, FOC_CTRL_STEP_ANGLE, valf);
    else if (strcmp(key, "low_speed") == 0) QD4310_Ctrl(&qd4310, FOC_CTRL_LOW_SPEED, valf);
    else {
        PRINT("Unknown ctrl target: %s", key);
        return 0;
    }
    PRINT("Setting %s = %.2f", key, valf);
    return 0;
}

int foc_enable(int argc, char *argv[]) {
    (void)argc;
    (void)argv;
    QD4310_Start(&qd4310);
    PRINT(qd4310.foc.started ? "QDrive enabled" : "enable failed, please calibrate first");
    return 0;
}

int foc_disable(int argc, char *argv[]) {
    (void)argc;
    (void)argv;
    QD4310_Stop(&qd4310);
    PRINT("QDrive disabled");
    return 0;
}

int foc_calibrate(int argc, char *argv[]) {
    (void)argc;
    (void)argv;
    if (qd4310.foc.started) {
        PRINT("QDrive is running, please disable it first");
        return 0;
    }
    PRINT("QDrive calibration started, please wait...");
    QD4310_Calibrate(&qd4310);
    PRINT(qd4310.foc.calibrated ? "QDrive calibration completed" : "QDrive calibration failed");
    return 0;
}

int foc_restore(int argc, char *argv[]) {
    (void)argc;
    (void)argv;
    QD4310_RestoreCalibration(&qd4310);
    PRINT("QDrive factory restore completed");
    foc_config_list();
    return 0;
}

int foc_store(int argc, char *argv[]) {
    (void)argc;
    (void)argv;
    if (qd4310.foc.started) {
        PRINT("QDrive is running, please disable it first");
        return 0;
    }
    QD4310_FreezeStorageCalibration(&qd4310,
        (QD4310_StorageStatus)(QD4310_STORAGE_PID_PARAMETER_OK |
                               QD4310_STORAGE_LIMIT_OK |
                               QD4310_STORAGE_PLUG_OK));
    PRINT("Store configuration completed");
    return 0;
}

int shell_reboot(int argc, char *argv[]) {
    (void)argc;
    (void)argv;
    NVIC_SystemReset();
    return 0;
}

int shell_silent(int argc, char *argv[]) {
    (void)argc;
    (void)argv;
    shell.write = silent;
    return 0;
}

SHELL_EXPORT_CMD(SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                 silent, shell_silent, "Disable shell output, reboot to enable again");
SHELL_EXPORT_CMD(SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                 version, print_version, Show version info);
SHELL_EXPORT_CMD(SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                 reboot, shell_reboot, reboot system);
SHELL_EXPORT_CMD(SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                 store, foc_store, Store configurations);
SHELL_EXPORT_CMD(SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                 restore, foc_restore, Factory restore);
SHELL_EXPORT_CMD(SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                 ctrl, foc_ctrl, Set control targets);
SHELL_EXPORT_CMD(SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                 config, foc_config, Configure system parameters);
SHELL_EXPORT_CMD(SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                 calibrate, foc_calibrate, Calibrate FOC system);
SHELL_EXPORT_CMD(SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                 disable, foc_disable, Disable FOC control);
SHELL_EXPORT_CMD(SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                 enable, foc_enable, Enable FOC control);
SHELL_EXPORT_CMD(SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                 status, foc_status, Show current motor status);
SHELL_EXPORT_CMD(SHELL_CMD_DISABLE_RETURN|SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_MAIN),
                 info, foc_info, Show hardware information);
