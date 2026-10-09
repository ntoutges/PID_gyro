#ifndef _DEBUG_H
#define _DEBUG_H

#include <Arduino.h>
#include <csch.h>
#include <rval.h>

extern csch_t sched;
extern cmd_t cmd;

void debug_init();

void t_debug();

#endif