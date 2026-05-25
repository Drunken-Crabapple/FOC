#include "PID.h"

#include <math.h>
#include <stdio.h>

static int nearly_equal(float actual, float expected) {
    const float diff = fabsf(actual - expected);
    if (diff > 1e-6f) {
        fprintf(stderr, "expected %.9g, got %.9g\n", expected, actual);
        return 0;
    }
    return 1;
}

static int position_pid_matches_original_formula(void) {
    PID_t pid;
    PID_Init(&pid, PID_POSITION_TYPE, 2.0f, 0.5f, 0.25f, 3.0f, -3.0f, 10.0f, -10.0f);
    PID_SetTarget(&pid, 4.0f);

    if (!nearly_equal(PID_Calc(&pid, 1.0f), 8.25f)) return 1;
    if (!nearly_equal(PID_Calc(&pid, 2.0f), 5.25f)) return 1;
    if (!nearly_equal(PID_Calc(&pid, -10.0f), 10.0f)) return 1;
    return 0;
}

static int delta_pid_matches_original_formula(void) {
    PID_t pid;
    PID_Init(&pid, PID_DELTA_TYPE, 1.0f, 0.5f, 0.25f, NAN, NAN, 100.0f, -100.0f);
    PID_SetTarget(&pid, 5.0f);

    if (!nearly_equal(PID_Calc(&pid, 2.0f), 5.25f)) return 1;
    if (!nearly_equal(PID_Calc(&pid, 3.0f), 4.25f)) return 1;
    if (!nearly_equal(PID_Calc(&pid, 5.0f), 2.0f)) return 1;
    return 0;
}

int main(void) {
    if (position_pid_matches_original_formula()) return 1;
    if (delta_pid_matches_original_formula()) return 1;
    return 0;
}
