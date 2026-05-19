#include "comms_task.h"

#include "config.h"
#include "globals.h"

namespace tasks {

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
        case 'm': {
            int left  = 0;
            int right = 0;
            const int parsed = sscanf(args.c_str(), "%d %d", &left, &right);
            if (parsed != 2) {
                Serial.println("[comms] usage: m <left_duty> <right_duty>");
                break;
            }
            const int16_t actual_l = g_motor_left.setDuty(static_cast<int16_t>(left));
            const int16_t actual_r = g_motor_right.setDuty(static_cast<int16_t>(right));
            Serial.printf("[comms] motors set L=%d R=%d\n", actual_l, actual_r);
            break;
        }
        case 'x': {
            // emergency coast!
            g_motor_left.coast();
            g_motor_right.coast();
            Serial.println("[comms] EMERGENCY COAST: motors stopped");
            break;
        }
        case '?': {
            Serial.printf("[comms] servo=%.2f deg, motor L=%d, motor R=%d\n",
                          g_servo.getAngle(),
                          g_motor_left.getDuty(),
                          g_motor_right.getDuty());
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

    Serial.println("[comms] ready. Commands:");
    Serial.println("[comms]   s <angle>          → steering angle (deg)");
    Serial.println("[comms]   m <left> <right>   → motor duty (-1023..+1023)");
    Serial.println("[comms]   x                  → emergency coast");
    Serial.println("[comms]   ?                  → query state");

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