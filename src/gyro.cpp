#include "gyro.h"

uint32_t _gyro_last_tick = 0;
gyro_state_t _gyro_state = GYRO_S_INIT;

// Map from gyro fs value (index) to gyro scale (value)
const uint16_t _gyro_fs[] = { 250, 500, 1000, 2000 };
uint8_t _gyro_fs_i = 0; // Current gyro fs

MPU6050 _gyro;

float _gyro_rotation = 0; // Stores rotation in degrees

uint16_t _gyro_cal_cycles = 0; // Number of cycles of calibration to perform
uint16_t _gyro_cal_cycle_ct = 0; // Number of cycles of calibration performed
float _gyro_cal_acc = 0; // Gyro accumulator

void gyro_init() {
    Wire.begin();
    csch_task_fork(&sched, t_gyro);
}

void t_gyro() {

    switch (_gyro_state) {
        case GYRO_S_INIT:
            // Initialize gyroscope object
            _gyro.initialize();
            
            // Automagically advance to next state
            csch_cqueue(0);
            _gyro_state = GYRO_S_BEGIN;

            // nobreak;

        case GYRO_S_BEGIN:
            
            // Ensure MPU is properly connected
            if (!_gyro.testConnection()) {
                
                // Connection failed! Try again in 1s
                csch_cqueue(csch_cms_to_ticks(1000));
                break;
            }

            // Connection succeeded! Advance to idle state
            csch_cqueue(0);
            _gyro_state = GYRO_S_IDLE;
            break;

        case GYRO_S_IDLE: {
            csch_cqueue(0); // Run this task as fast as possible

            // Compute delta time since last tick
            uint32_t now = micros();
            uint32_t delta = now - _gyro_last_tick;
            _gyro_last_tick = now;

            int16_t rotation_raw = _gyro.getRotationY();
            
            // Convert from arbitrary units to deg/s
            float rotation_deg = rotation_raw * (_gyro_fs[_gyro_fs_i] / 32768.0);
            
            // Apply calibration value(s)
            rotation_deg -= _gyro_cal_acc;

            // Integrate!
            _gyro_rotation += rotation_deg * (delta / 1e6);

            // Constrain `_gyro_rotation` between 0 and 360
            while (_gyro_rotation >= 360) _gyro_rotation -= 360;
            while (_gyro_rotation < 0) _gyro_rotation += 360;

            // vvvv UNCOMMENT BELOW TO ENABLE EXPERIMENTAL AUTOSCALING vvvv

            // // ==== Run GYRO autoscaling ====

            // // Getting close to clipping upper threshold of
            // // gyro. Increase range to avoid, sacrificing
            // // precision
            // if (
            //     abs(rotation_raw) > GYRO_UPPER_THRESH &&
            //     _gyro_fs_i < sizeof(_gyro_fs) / sizeof(*_gyro_fs)
            // ) {
            //     _gyro.setFullScaleGyroRange(++_gyro_fs_i);
            //     Serial.print("++ HIGH THRESH -> ");
            //     Serial.println(_gyro_fs_i);
            // }

            // // Getting well under maximum value of current
            // // gyro scale. Decrease scale to gain back
            // // some numeric precision
            // if (
            //     abs(rotation_raw) < GYRO_LOWER_THRESH &&
            //     _gyro_fs_i > 0
            // ) {
            //     _gyro.setFullScaleAccelRange(--_gyro_fs_i);
            //     Serial.print("-- LOW THRESH -> ");
            //     Serial.println(_gyro_fs_i);
            // }

            // Move to calibration state
            if (_gyro_cal_cycles > 0) {
                _gyro_state = GYRO_S_CAL;
                _gyro.setFullScaleAccelRange(0); // Switch to most precise mode
                csch_cqueue(0);
            }
            break;
        }

        case GYRO_S_CAL: {

            // Move back to main idle state once done
            if (_gyro_cal_cycle_ct++ == _gyro_cal_cycles) {
                
                // Calibration interrupted; Don't adjust accumulator
                if (_gyro_cal_cycles != 0) {
                    // Convert accumulator to average offset
                    _gyro_cal_acc /= _gyro_cal_cycles;
                }
                
                _gyro_cal_cycles = 0; // Done calibrating

                _gyro_state = GYRO_S_IDLE;
                csch_cqueue(0);
                break;
            }

            csch_cqueue(0); // Run calibration as fast as possible

            int16_t rotation_raw = _gyro.getRotationY();

            // Get rotation in deg/s
            float rotation_deg = rotation_raw * (_gyro_fs[0] / 32768.0);

            _gyro_cal_acc += rotation_deg;
            break;
        }
    }
}

float gyro_read() {
    return _gyro_rotation;
}

void gyro_calibrate(uint16_t cycles) {
    _gyro_cal_cycles = cycles;
    _gyro_cal_acc = 0;
}

void gyro_calibration(float value) {
    _gyro_cal_acc = value;

    // Halt any current calibration
    _gyro_cal_cycle_ct = 0;
    _gyro_cal_cycles = 0;
}

float gyro_get_calibration() {
    return _gyro_cal_acc;
}
