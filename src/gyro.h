#ifndef _GYRO_H
#define _GYRO_H

/**
 * @file src/gyro.h
 * @author Nicholas T.
 * @copyright 2026 PiCO
 * @brief Manages converting sensor input to some rotation
 *
 * Constantly polls the attached MPU6050 for gyro (rotational-velocity) readings
 * Integrate these readings to obtain the current rotational position
 */

#include <csch.h>
#include <stdint.h>

extern csch_t sched;

/**
 * Initialize the motor task
 * Expects global `csch_t sched` to already be initialized and ready-to-use
 */
void motor_init();

/**
 * The main gyro task. Add this to the `csch` scheduled tasks to begin running
 * Automagically scheduled by `gyro_init`. Do _NOT_ create this task a second time externally
 */
void t_gyro();

/**
 * Read the current angle in degrees.
 * This value is guaranteed to always be in the range [0, 360)
 * 
 * @returns The current orientation, in degrees
 */
float gyro_read();

/**
 * Calibrate gyro
 * Requires gyro to be held extremely still about the axis of rotation
 * Used to help offset gyro drift
 * 
 * @param cycles The number of samples to read from the gyro for calibration
 * A larger number takes longer, but is more accurate
 */
void gyro_calibrate(uint16_t cycles);

/**
 * Apply gyro some pre-computed calibration value to the gyro
 * @param value The calibration value to apply
 */
void gyro_calibration(float value);

/**
 * Get the current gyro calibration value
 * @returns The current gyro calibration value
 */
float gyro_get_calibration();

#endif
