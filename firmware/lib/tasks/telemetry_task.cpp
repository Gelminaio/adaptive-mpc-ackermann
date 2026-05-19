#include "telemetry_task.h"

#include "config.h"

namespace tasks {

void telemetry_task(void* params) {
    (void)params;

    const TickType_t period_ticks = pdMS_TO_TICKS(PERIOD_TELEMETRY_MS);
    TickType_t last_wake = xTaskGetTickCount();

    uint32_t heartbeat_counter = 0;

    while (true) {
        // TODO: structured telemetry output for plotting/logging

        if (++heartbeat_counter >= 10) {  
            heartbeat_counter = 0;
            Serial.printf("[telemetry] heartbeat @ %lu ms\n", millis());
        }

        vTaskDelayUntil(&last_wake, period_ticks);
    }
}

}  