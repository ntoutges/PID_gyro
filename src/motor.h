#ifndef _MOTOR_H
#define _MOTOR_H

/**
 * @file src/motor.h
 * @author Nicholas T.
 * @copyright 2026 PiCO
 * @brief Manage the rotation motor
 * Allows a simple PWM value to be given and abstracts
 * away all the non-idealities of starting the motor
 * (specifically at low PWM values)
 */

#include <csch.h>
#include <stdint.h>

extern csch_t sched;

/**
 * Initialize the motor task.
 * Expects global `csch_t sched` to already be initialized and ready-to-use
 */
void motor_init();

/**
 * The main motor task. Add this to the `csch` scheduled tasks to begin running
 * Automagically scheduled by `motor_init`. Do _NOT_ create this task a second time externally
 */
void t_motor();

/**
 * Set the motor speed.
 * Negative values rotate the motor in the opposite direction
 * Passes the PWM value to an internal state machine to avoid
 * external code needint to understand/deal with the non-idealities of
 * real-world low-speed motor startup conditions
 * 
 * @param pwm The speed to run the motor at. Must be a value [-255, 255]
 * Set to 0 to stop the motor
 */
void motor_set(int16_t pwm);

#endif