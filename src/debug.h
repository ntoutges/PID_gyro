#ifndef _DEBUG_H
#define _DEBUG_H

#include <Arduino.h>
#include <stdint.h>
#include <csch.h>
#include <rval.h>

#include <AsyncTCP.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>

#include "wifi_config.h"
#include "pages/picom_html.h"

extern csch_t sched;
extern cmd_t shell;

#define DEBUG_CLEANUP_MS 5000 // Period between cleanup cycles
#define DEBUG_SEND_BUF 64     // Send buffer size; Max number of chars to hold onto before sending
#define DEBUG_SEND_MS 100     // Send buffer timeout; Max time to wait before sending

/**
 * Initialize the debug task
 * Expects global `csch_t sched` to already be initialized and ready-to-use
 */
void debug_init();

/**
 * Main debug task, used to read input from the user via the Serial terminal
 * Automagically scheduled by `debug_init`. Do _NOT_ create this task a second time externally
 */
void t_debug();

/**
 * Task periodically run, used to cleanup websocket data
 * Automagically scheduled by `debug_init`. Do _NOT_ create this task a second time externally
 */
void t_debug_cleanup();

/**
 * @brief Send some message to the debug interface
 * @param c The character to send
 * 
 * `extern "C"` added to make fn available to raw C code
 */
extern "C" void debug_send(char c);

#endif