#include "comms_task.h"

#include "config.h"
#include "globals.h"

namespace tasks {

// Simple line-based serial parser.
// Supported commands:
//   s <angle>      → set steering angle in degrees (from -30 to 30) (example: > s 15.0)
//   ?              → print current servo degrees
static void process_line(const String& line) {
    String trimmed = line;
    trimmed.trim();
    if (trimmed.length() == 0) return;

    const char cmd = trimmed[0];
    String args = trimmed.substring(1);
    args.trim();

    switch (cmd) {
        case 's': {
            const float requested = args.toFloat();
            const float actual    = g_servo.setAngle(requested);
            if (fabsf(actual - requested) > 0.01f) {
                Serial.printf("[comms] steering CLAMPED %.2f -> %.2f deg\n",
                              requested, actual);
            } else {
                Serial.printf("[comms] steering set to %.2f deg\n", actual);
            }
            break;
        }
        case '?': {
            Serial.printf("[comms] servo angle = %.2f deg\n", g_servo.getAngle());
            break;
        }
        default:
            Serial.printf("[comms] unknown command: '%c'\n", cmd);
            break;
    }
}

void comms_task(void* params) {
    (void)params;

    const TickType_t period_ticks = pdMS_TO_TICKS(PERIOD_COMMS_MS);
    TickType_t last_wake = xTaskGetTickCount();

    String line_buffer;
    uint32_t heartbeat_counter = 0;

    Serial.println("[comms] ready. Commands: 's <angle>' to steer, '?' to query.");

    while (true) {
        while (Serial.available()) {
            const char c = static_cast<char>(Serial.read());
            if (c == '\n' || c == '\r') {
                if (line_buffer.length() > 0) {
                    process_line(line_buffer);
                    line_buffer = "";
                }
            } else {
                line_buffer += c;
                if (line_buffer.length() > 64) {
                    line_buffer = "";
                }
            }
        }

        if (++heartbeat_counter >= 50) { 
            heartbeat_counter = 0;
        }

        vTaskDelayUntil(&last_wake, period_ticks);
    }
}

}