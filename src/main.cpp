#include <Arduino.h>
#include <csch.h>
#include <rval.h>

#include "gyro.h"
#include "motor.h"
#include "pid.h"
#include "debug.h"

#define MAX_OUTPUT 100

// Scheduler data
csch_t sched;
csch_proc_t proc_buf[8];

// cmd/rval data
cmd_entry_t entries[8];
uint8_t cmd_buffer[128];
cmd_t shell;

uint8_t rval_buffer[128];
rval_t rv;

// PID object
cpid_t pid;

void t_controller();
uint8_t p_controller;

void t_updater();

bool onSet(rval_key_view_t);
void a_start(void*);
void a_stop(void*);

void setup() {
    Serial.begin(115200); // Obvious debug data is obvious

    Wire.begin();
    Wire.setTimeout(50);
    Wire.setClock(10000);
    
    // Initialize scheduler
    sched = csch_create(1, millis, proc_buf, sizeof(proc_buf) / sizeof(*proc_buf));

    // Initialize cmd/rval

    shell = cmd(
        '!',
        entries,
        sizeof(entries) / sizeof(*entries),
        cmd_buffer,
        sizeof(cmd_buffer),
        debug_send
    );

    rval(
        &rv,
        &shell,
        rval_buffer,
        sizeof(rval_buffer),
        onSet,
        NULL
    );

    // Initialize submodules
    motor_init();
    gyro_init();
    debug_init();

    p_controller = csch_task_fork(&sched, t_controller);
    csch_task_fork(&sched, t_updater);

    // @TODO: Fill kp, ki, kd, with real values
    pid = pid_create(0, 0, 0);

    // Initialize rval data
    rval_decl_i16(&rv, "s", false);   // Override motor speed
    rval_decl_i16(&rv, "rot", false); // Desired rotation
    rval_decl_i16(&rv, "crot", false); // Current rotation
    
    // PID coefficients
    rval_decl_f(&rv, "p", false);
    rval_decl_f(&rv, "i", false);
    rval_decl_f(&rv, "d", false);
    rval_decl_f(&rv, "out", false);
    rval_decl_f(&rv, "err", false);

    cmd_attach(&shell, "start", a_start, NULL);
    cmd_attach(&shell, "stop", a_stop, NULL);

    // Run scheduler
    while (true) {
        csch_tick(&sched);
    }
}

void loop() {}

int16_t desired_rotation = 0;
float system_out = 0;
int16_t error;

void t_controller() {
    csch_cqueue(100); // Run as fast as possible

    error = desired_rotation - gyro_read();

    // Update error to be within range [0,360)
    while (error < 0) error += 360;
    while (error >= 360) error -= 360;

    // Update error to be minimum of both rotational directions
    if (error > 180) error -= 360;

    // Run PID controller
    system_out = pid_tick(&pid, error, micros());

    // Convert PID output to value motors can understand
    int16_t actuator_input;
    if (system_out > MAX_OUTPUT) actuator_input = MAX_OUTPUT;
    else if (system_out < -MAX_OUTPUT) actuator_input = -MAX_OUTPUT;
    else actuator_input = roundf(system_out);

    motor_set(actuator_input);
}

// Update rval occasionally
// Value rate-limited to avoid costly update each controller cycle
void t_updater() {
    csch_cqueue(csch_cms_to_ticks(100));
    rval_set_f(&rv, "out", system_out);
    rval_set_f(&rv, "err", error);
    rval_set_i(&rv, "crot", gyro_read());
}

bool onSet(rval_key_view_t key) {

    // Rotation updated
    if (rval_strcmp(key, "rot") == 0) {
        desired_rotation = rval_get_i(&rv, "rot"); // Cache updated value
    }

    // Update P/I/D
    else if (rval_strcmp(key, "p") == 0) {
        pid_p(&pid, rval_get_f(&rv, "p"));
    }
    else if (rval_strcmp(key, "i") == 0) {
        pid_i(&pid, rval_get_f(&rv, "i"));
    }
    else if (rval_strcmp(key, "d") == 0) {
        pid_d(&pid, rval_get_f(&rv, "d"));
    }

    // Override motor speed
    else if (rval_strcmp(key, "s") == 0) {
        motor_set(rval_get_i(&rv, "s"));
    }

    return true;
}

void a_start(void*) {
    
    // Restart PID controller
    pid_reset(&pid);
    csch_queue(&sched, p_controller, 0);

    const char* msg = "Started...\n";
    for (uint8_t i = 0; msg[i]; i++) {
        debug_send(msg[i]);
    }
}

void a_stop(void*) {
    csch_hibernate(&sched, p_controller);   // Stop PID controller
    motor_set(0);                           // Stop motor
    rval_set_i(&rv, "s", 0);

    const char* msg = "Stopped...\n";
    for (uint8_t i = 0; msg[i]; i++) {
        debug_send(msg[i]);
    }
}
