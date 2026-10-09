#include "motor.h"

void _motor_set(int16_t pwm); // Set raw PWM values, skipping state machine
void _motor_fsm_kick(); // Run a cycle of the FSM

motor_state_t _motor_state = MOTOR_S_IDLE;
int16_t _motor_desired = 0;  // Desired motor speed
int16_t _motor_current = 0; // Current motor speed

uint8_t _p_motor; // Motor task PID

void motor_init() {

    // Initialize motor pins
    pinMode(MOTOR_A, OUTPUT);
    pinMode(MOTOR_B, OUTPUT);

    _p_motor = csch_task_fork(&sched, t_motor);
}

void t_motor() {
    switch (_motor_state) {

        // Stop pulse
        case MOTOR_S_PULSE_STOP:
            _motor_state = MOTOR_S_IDLE; // Move back to idle state
            // nobreak; // Run idle state to update motor speed

        case MOTOR_S_IDLE:
            _motor_set(_motor_desired);  // Update true motor speed to desired speed
            break;

        // Start pulse
        case MOTOR_S_PULSE_START:
            _motor_set(
                _motor_desired > 0
                    ? MOTOR_PULSE_AMP
                    : -MOTOR_PULSE_AMP
            );

            // Queue to run after some
            // time to stop pulse
            _motor_state = MOTOR_S_PULSE_STOP;
            csch_cqueue(csch_cms_to_ticks(MOTOR_PULSE_MS));
            break;
    }
}

void motor_set(int16_t pwm) {
    
    // No change; No action required...
    if (pwm == _motor_desired) return;

    // Set desired motor speed; Will be acted
    // on by the state machine later
    _motor_desired = pwm;

    // PWM value sufficiently large to not require starting pulse
    if (
        abs(pwm) >= MOTOR_PULSE_THRESH
    ) {
        _motor_state = MOTOR_S_IDLE; // Exit pulse state iff required
        _motor_fsm_kick(); // Run state machine with updated motor values
        return;
    }

    // PWM value is sufficiently small indicating a stop has been requested
    if (abs(pwm) < MOTOR_MIN_SPEED) {
        _motor_desired = 0;
        _motor_state = MOTOR_S_IDLE; // Exit pulse state iff required
        _motor_fsm_kick(); // Run state machine with updated motor values
        return;
    }

    // Motor already moving => Friction already overcome
    // => No pulse required...
    if (_motor_current != 0) {
        
        // Kick FSM iff not already running pulse
        if (_motor_state != MOTOR_S_PULSE_STOP) _motor_fsm_kick();
        return; 
    }
    
    // Start pulse
    _motor_state = MOTOR_S_PULSE_START;
    _motor_fsm_kick();
}

void _motor_set(int16_t pwm) {
    _motor_current = pwm;

    // Run motors in some direction
    if (pwm > 0) {
        analogWrite(MOTOR_A, pwm);
        analogWrite(MOTOR_B, 0);
    }

    // Run motors in opposite direction
    else {
        analogWrite(MOTOR_A, 0);
        analogWrite(MOTOR_B, -pwm);
    }
}

void _motor_fsm_kick() {
    csch_queue(&sched, _p_motor, 0);
}
