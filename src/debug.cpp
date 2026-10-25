#include "debug.h"

void _debug_index_get(AsyncWebServerRequest* request);
void _debug_ws_event(AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t* data, size_t len);
void _debug_send_buf(); // Send current send buffer to debug interfaces (WiFi + Serial)

// WiFi data
AsyncWebServer _debug_server(80);
AsyncWebSocket _debug_ws("/");

char _debug_buf[DEBUG_SEND_BUF]; // Send buffer
uint8_t _debug_buf_len = 0;      // Send buffer length

uint8_t _p_debug_send;

void debug_init() {
    
    // Schedule debug tasks
    csch_task_fork(&sched, t_debug_cleanup); // Periodic cleanup task
    csch_task_fork(&sched, t_debug);
    _p_debug_send = csch_task_fork(&sched, _debug_send_buf); // One-shot send-buffer (used for timeouts)
    csch_hibernate(&sched, _p_debug_send);

    // Setup esp32 as AP
    WiFi.mode(WIFI_STA);
    WiFi.softAP(WIFI_SSID, WIFI_PASS);

    // Setup webserver, accessed via AP
    _debug_ws.onEvent(_debug_ws_event);
    _debug_server.addHandler(&_debug_ws);
    _debug_server.on("/", HTTP_GET, _debug_index_get); // Listen for index path

    _debug_server.begin();
}

void t_debug() {
    csch_cqueue(csch_cms_to_ticks(100)); // Periodically poll for new data

    while (Serial.available()) {
        char c = Serial.read();
        cmd_recv(&shell, c);
    }
}

void t_debug_cleanup() {
    csch_cqueue(csch_cms_to_ticks(DEBUG_CLEANUP_MS)); // Periodically run cleanup
    _debug_ws.cleanupClients();
}

void _debug_index_get(AsyncWebServerRequest* request) {
    request->send(200, "text/html", picom_html);
}

void _debug_ws_event(AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t* data, size_t len) {

    switch (type) {
        // Logging
        case WS_EVT_CONNECT:
            Serial.print("[WebSocket] Connected: ");
            Serial.println(client->id());
            break;

        case WS_EVT_DISCONNECT:
            Serial.print("[WebSocket] Disconnected: ");
            Serial.println(client->id());
            break;

        // Handle messages
        case WS_EVT_DATA: {
            
            // Ingest all data
            for (size_t i = 0; i < len; i++) {
                cmd_recv(&shell, data[i]);
            }
            break;
        }
    }
}

void debug_send(char c) {
    _debug_buf[_debug_buf_len++] = c;

    // Buffer at max length; Send!
    if (_debug_buf_len >= DEBUG_SEND_BUF) {
        _debug_send_buf();
        csch_hibernate(&sched, _p_debug_send); // Kill current send timer
        return;
    }

    // Start new timer
    // Once expires, data will be sent
    csch_queue(&sched, _p_debug_send, csch_ms_to_ticks(&sched, DEBUG_SEND_MS));
}

void _debug_send_buf() {
    Serial.write(_debug_buf, _debug_buf_len); // Send via Serial
    _debug_ws.textAll(_debug_buf, _debug_buf_len);

    _debug_buf_len = 0; // Reset send buffer; All data processed!
}
