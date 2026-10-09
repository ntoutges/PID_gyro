#include "pid.h"

cpid_t pid_create(float kp, float ki, float kd) {
    return (cpid_t) {
        // Initialize coefficients
        .kp = kp,
        .ki = ki,
        .kd = kd,

        // Initialize state
        .last_error = 0,
        .acc = 0,

        // No limit by default
        .acc_max = INFINITY,

        // Indicate that we haven't yet run
        .last_tick = 0
    };
}

float pid_tick(cpid_t* pid, float error, uint32_t tmicros) {
    float p, i, d; // :)

    // Compute delta time, in s
    float dtime = pid->last_tick == 0
        ? 0 // We haven't run before; Cannot trust this value
        : (tmicros - pid->last_error) / 1e6; // Standard delta time computation
    
    // Track when we last ran
    pid->last_tick = tmicros;
    
    // ==== P term calculation ====
    p = pid->kp * error;

    // ==== I term calculation ====
    pid->acc += error * dtime; // Accumulate error
                               // Already properly handles `dtime=0` case

    // Cap accumulator at +/-pid->acc_max
    if (abs(pid->acc) > pid->acc_max) {
        pid->acc = pid->acc < 0
            ? -pid->acc_max
            : pid->acc_max;
    }

    i = pid->ki * pid->acc;

    // ==== D term calculation ====
    float derror = dtime == 0
        ? 0 // Invalid dtime; Ignore
        : (error - pid->last_error) / dtime; // Differentiate error
    d = pid->ki * derror;

    pid->last_error = error;

    // Mix all values together
    return p + i + d;
}

void pid_p(cpid_t* pid, float kp) {
    pid->kp = kp;
}

void pid_i(cpid_t* pid, float ki) {
    pid->kp = ki;
}

void pid_d(cpid_t* pid, float kd) {
    pid->kp = kd;
}

void pid_reset(cpid_t* pid) {
    pid->last_error = 0;
    pid->acc = 0;
    pid->last_tick = 0;
}

void pid_imax(cpid_t* pid, float max) {
    pid->acc_max = max;
}
