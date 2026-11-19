#include "setup.h"
#include "gyro.h"

/**
 * @file gyro_mpu6050.cpp
 * @author Nicholas T. (ntoutges@gmail.com)
 * @brief Assume connected gyro is th BINO85; Query rotation directly
 * @version 0.1
 * @date 2026-11-19
 * 
 * @copyright Copyright (c) 2026
 * 
 */

// Compile IFF using BINO85 as gyro
#if GYRO_USE_BINO85

#include <7Semi_BNO08x.h>
#include <math.h>

typedef enum gyro_state_t {
    GYRO_S_INIT,        // Initializing module
    GYRO_S_IDLE,        // Poll BNO for data
} gyro_state_t;

static BnoI2CBus _gyro_bus(Wire, -1, -1, 0x4A, 100000); // Initialize BNO bus
static BNO08x_7Semi _gyro_bno(_gyro_bus);       // Initialize BNO object

gyro_state_t _gyro_state = GYRO_S_INIT;
float _gyro_x, _gyro_y, _gyro_z, _gyro_w; // Store raw quat outputs

float _gyro_rot; // Rotation from quaternion

void gyro_init() {
    csch_task_fork(&sched, t_gyro);
}

void t_gyro() {

    switch (_gyro_state) {

        case GYRO_S_INIT:
        
            // Initialize BNO
            if (!_gyro_bno.begin()) {
                csch_cqueue(csch_cms_to_ticks(1000)); // Failed to initialize, try again later
                cmd_sends(&shell, "BMO85 begin fail...\n");
                break;
            }

            // Enable relevant modules
            if (!_gyro_bno.enableRotationVector(20)) { // Request report every 20 ms
                csch_cqueue(csch_cms_to_ticks(100)); // Failed to enable report, try again later
                cmd_sends(&shell, "BMO85 report set fail...\n");
                break;
            }

            cmd_sends(&shell, "BMO85 successfully initialized!\n");
            csch_cqueue(0);
            _gyro_state = GYRO_S_IDLE;
            break;

        case GYRO_S_IDLE:
            csch_cqueue(0);
            _gyro_bno.processData(); // Attempt to process any received data packets

            if (_gyro_bno.getQuaternion(_gyro_x, _gyro_y, _gyro_z, _gyro_w)) {
                
                // Compute rotation from quat
                // Assume rotating about y axis
                // _gyro_rot = atan2f(
                //     2 * (_gyro_w * _gyro_y + _gyro_x * _gyro_z),
                //     1 - 2 * (_gyro_x * _gyro_x, _gyro_y * _gyro_y)
                // ) * (180.0 / PI); // Convert to degrees
                // float siny = 2 * (_gyro_w * _gyro_y + _gyro_x * _gyro_z);
                // siny = fmaxf(-1, fminf(1, siny)); // Clamp between -1, 1
                // _gyro_rot = asinf(siny) * (180.0 / PI);
                _gyro_rot = 2*atan2f(_gyro_y, _gyro_w) * (180.0 / PI); // Convert to degrees

                if (_gyro_rot < 0) _gyro_rot += 360; // Clamp to [0, 360)
            }

            break;
    }

}

float gyro_read() {
    return _gyro_rot;
}

// NOP for BINO85
void gyro_calibrate(uint16_t cycles) {}
void gyro_calibration(float value) {}
float gyro_get_calibration() {
    return 0;
}

#endif
