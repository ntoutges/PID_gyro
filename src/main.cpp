#include <Arduino.h>
#include <csch.h>
#include <rval.h>

#include "gyro.h"
#include "motor.h"
#include "pid.h"

// Scheduler data
csch_t sched;
csch_proc_t proc_buf[4];

// cmd/rval data
cmd_entry_t entries[8];
uint8_t cmd_buffer[128];
cmd_t shell;

uint8_t rval_buffer[128];
rval_t rv;

// PID object
cpid_t pid;

void t_test();
bool onSet(rval_key_view_t);

void csend(char c) {
    Serial.print(c);
}

void setup() {
    Serial.begin(115200); // Obvious debug data is obvious
    
    // Initialize scheduler
    sched = csch_create(1, millis, proc_buf, sizeof(proc_buf) / sizeof(*proc_buf));

    // Initialize cmd/rval

    shell = cmd(
        '!',
        entries,
        sizeof(entries) / sizeof(*entries),
        cmd_buffer,
        sizeof(cmd_buffer),
        csend
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

    // @TODO: Fill kp, ki, kd, with real values
    pid = pid_create(0, 0, 0);

    // Initialize rval data
    rval_decl_i16(&rv, "s", false);

    // Run scheduler
    while (true) {
        csch_tick(&sched);
    }
}

void loop() {}

void t_test() {
    while (Serial.available()) {
        cmd_recv(&shell, Serial.read());
    }
}

bool onSet(rval_key_view_t key) {

    // Assume `s` was updated...
    int16_t s = rval_get_i(&rv, "s");
    motor_set(s);
}
