#ifndef _GYRO_H
#define _GYRO_H

/**
 * @file src/gyro.h
 * @author Nicholas T.
 * @brief Manages converting sensor input to some rotation
 *
 * Constantly polls the attached MPU6050 for gyro (rotational-velocity) readings
 * Integrate these readings to obtain the current rotational position
 */

/**
 * The main gyro task. Add this to the `csch` scheduled tasks to begin running
 */
void t_gyro();

#endif