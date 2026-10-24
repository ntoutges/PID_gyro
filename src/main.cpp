#include <Arduino.h>
#include <csch.h>
#include <rval.h>

#include "gyro.h"
#include "motor.h"
#include "pid.h"

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
void t_test();

bool onSet(rval_key_view_t);

void csend(char c) {
    Serial.print(c);
}

void setup() {
    Serial.begin(115200); // Obvious debug data is obvious

    Wire.begin();
    Wire.setTimeout(50);
    // Wire.setClock(10000);
    
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

    csch_task_fork(&sched, t_controller);
    csch_task_fork(&sched, t_test);

    // @TODO: Fill kp, ki, kd, with real values
    pid = pid_create(0, 0, 0);

    // Initialize rval data
    rval_decl_i16(&rv, "s", false);   // Override motor speed
    rval_decl_i16(&rv, "rot", false); // Desired rotation
    
    // PID coefficients
    rval_decl_i16(&rv, "p", false);
    rval_decl_i16(&rv, "i", false);
    rval_decl_i16(&rv, "d", false);

    // Run scheduler
    while (true) {
        csch_tick(&sched);
    }
}

void loop() {}

int16_t desired_rotation = 0;

void t_controller() {
    csch_cqueue(0); // Run as fast as possible

    int16_t error = desired_rotation - gyro_read();

    // Update error to be within range [0,360)
    while (error < 0) error += 360;
    while (error >= 360) error -= 360;

    // Update error to be minimum of both rotational directions
    if (error > 180) error -= 360;

    // Run PID controller
    float system_out = pid_tick(&pid, error, micros());

    // Convert PID output to value motors can understand
    int16_t actuator_input;
    if (system_out > 255) actuator_input = 255;
    else if (system_out < -255) actuator_input = -255;
    else actuator_input = roundf(system_out);

    motor_set(actuator_input);
}

void t_test() {
    csch_cqueue(100);

    while (Serial.available()) {
        char c = Serial.read();
        Serial.print(c);
        cmd_recv(&shell, c);
    }
}

bool onSet(rval_key_view_t key) {

    Serial.print("RECV ");
    Serial.print(key.length);
    Serial.print(" : '");
    for (int i = 0; i < key.length; i++) {
        Serial.print(key.chars[i]);
    }
    Serial.println("'");

    // Rotation updated
    if (key.length == strlen("rot") && strncmp(key.chars, "rot", strlen("rot")) == 0) {
        desired_rotation = rval_get_i(&rv, "rot"); // Cache updated value
    }

    // Update P/I/D
    else if (key.length == strlen("p") && strncmp(key.chars, "p", strlen("p")) == 0) {
        Serial.print("Kp = ");
        Serial.println(rval_get_i(&rv, "p"));
        pid_p(&pid, rval_get_i(&rv, "p"));
    }
    else if (key.length == strlen("i") && strncmp(key.chars, "i", strlen("i")) == 0) {
        pid_i(&pid, rval_get_i(&rv, "i"));
    }
    else if (key.length == strlen("d") && strncmp(key.chars, "d", strlen("d")) == 0) {
        pid_d(&pid, rval_get_i(&rv, "d"));
    }
    
    // Override motor speed
    else if (key.length == strlen("s") && strncmp(key.chars, "s", strlen("s")) == 0) {
        motor_set(rval_get_i(&rv, "s"));
    }

    return true;
}
