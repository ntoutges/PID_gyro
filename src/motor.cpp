#include "motor.h"

void _motor_set(uint8_t pwmA, uint8_t pwmB); // Set raw PWM values for each channel of the motor controller

motor_state_t _motor_state = MOTOR_S_IDLE;

void motor_init() {
    csch_task_fork(&sched, t_motor);
}

void t_motor() {
    switch (_motor_state) {
        case MOTOR_S_INIT:

            // Initialize motor pins
            pinMode(MOTOR_A, OUTPUT);
            pinMode(MOTOR_B, OUTPUT);

            break;

        case MOTOR_S_IDLE:

            break;

        case MOTOR_S_PULSE:
            break;
    }
}

void motor_set(int16_t pwm) {

    // PWM value sufficiently large to not require starting pulse
    if (abs(pwm) > MOTOR_PULSE_THRESH) {
        
    }
}

void _motor_set(uint8_t pwmA, uint8_t pwmB) {
    analogWrite(MOTOR_A, pwmA);
    analogWrite(MOTOR_B, pwmB);
}
