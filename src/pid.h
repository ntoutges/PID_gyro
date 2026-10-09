#ifndef _PID_H
#define _PID_H

/**
 * @file src/pid.h
 * @author Nicholas T.
 * @copyright 2026 PiCO
 * @brief Manage a simple PID controller
 */

#include <stdint.h>
#include <math.h>

// Make C/C++ compatible
#ifdef __cplusplus
extern "C" {
#endif

typedef struct cpid_t {
    
    // PID coefficients
    float kp, ki, kd;
    
    // Last error for use in differentiation
    float last_error;

    // Accumulator, for use in integration
    float acc;

    // Cap accumulator at some maximum value, to avoid
    // unbounded growth and keep the system stable
    float acc_max;

    // The time, in microseconds, of the last PID tick
    // Iff 0: assume the PID loop hasn't been run before
    uint32_t last_tick;

} cpid_t;

/**
 * Create a new PID object
 * 
 * @param kp The P coefficient
 * @param ki The I coefficient
 * @param kd The D coefficient
 * @returns The created PID object, ready for ticking
 */
cpid_t pid_create(float kp, float ki, float kd);

/**
 * Run a single cycle of the PID loop
 * Compute new I, D terms using the new error value, update
 * the internal state, then return the output value from the PID controller
 * for this iteration
 * Note that, on the first tick, the d(error)/dt = 0, integral(error) = 0
 * 
 * @param pid The PID object to act on
 * @param error The difference between the desired and actual value, computed as (desired - actual)
 * @param tmicros The time of the current iteration, in microseconds. This may be computed via the
 * Arduino framework's `micros()` function
 * @returns The computed value from this iteration of the PID loop
 */
float pid_tick(cpid_t* pid, float error, uint32_t tmicros);

/**
 * Set the P coefficient of a given PID object
 * @param pid The PID object to update
 * @param kp The P coefficient
 */
void pid_p(cpid_t* pid, float kp);

/**
 * Set the I coefficient of a given PID object
 * @param pid The PID object to update
 * @param ki The I coefficient
 */
void pid_i(cpid_t* pid, float ki);

/**
 * Set the D coefficient of a given PID object
 * @param pid The PID object to update
 * @param kd The D coefficient
 */
void pid_d(cpid_t* pid, float kd);

/**
 * Reset the state of the PID loop
 * Note that this function specifically does
 * **NOT** reset the coefficients
 * This is equivalent to calling `*pid = pid_create(pid->kp, ...)`
 * 
 * Call this method after updating some parameter to run the
 * PID controller loop from a clean state
 * 
 * @param pid The PID object to reset the state of
 */
void pid_reset(cpid_t* pid);

/**
 * Cap integrator term at some maximum value
 * Helps to prevent error from growing unbounded
 * 
 * @param pid The PID object to modify
 * @param max The maximum error we can accumulate before being capped
 */
void pid_imax(cpid_t* pid, float max);

#ifdef __cplusplus
}
#endif

#endif
