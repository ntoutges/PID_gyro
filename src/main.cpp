#include <Arduino.h>
#include <csch.h>

#include "gyro.h"
// #include "motor.h"
#include "pid.h"

// Scheduler data
csch_t sched;
csch_proc_t proc_buf[4];

// PID object
cpid_t pid;

void setup() {
    Serial.begin(115200); // Obvious debug data is obvious
    
    // Initialize scheduler
    sched = csch_create(1, millis, proc_buf, sizeof(proc_buf) / sizeof(*proc_buf));

    // Initialize submodules
    // motor_init();
    gyro_init();

    // @TODO: Fill kp, ki, kd, with real values
    pid = pid_create(0, 0, 0);

    // Run scheduler
    while (true) {
        csch_tick(&sched);
    }
}

void loop() {}
