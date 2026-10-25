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

#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <csch.h>
#include <stdint.h>

#define GYRO_UPPER_THRESH (32768 - (32768 / 4)) // Threshold to increase range (decrease precision)
#define GYRO_LOWER_THRESH (32768 / 2)           // Threshold to decrease range (increase precision)

extern csch_t sched;

typedef enum gyro_state_t {
    GYRO_S_INIT,  // Initializing module
    GYRO_S_BEGIN, // Attempting to verify MPU6050 is avaiable
    GYRO_S_IDLE,  // Poll MPU6050 for rotational data
    GYRO_S_CAL    // Running calibration
} gyro_state_t;

/**
 * Initialize the gyro task
 * Expects global `csch_t sched` to already be initialized and ready-to-use
 */
void gyro_init();

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
