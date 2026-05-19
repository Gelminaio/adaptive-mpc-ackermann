#include "safety_task.h"

#include "config.h"

namespace tasks {

void safety_task(void* params) {
    (void)params;

    const TickType_t period_ticks = pdMS_TO_TICKS(PERIOD_SAFETY_MS);
    TickType_t last_wake = xTaskGetTickCount();

    uint32_t heartbeat_counter = 0;

    while (true) {
        // TODO (step 3.7): watchdog feed, cmd_vel timeout check, emergency stop

        if (++heartbeat_counter >= 50) {  
            heartbeat_counter = 0;
            Serial.printf("[safety] heartbeat @ %lu ms\n", millis());
        }

        vTaskDelayUntil(&last_wake, period_ticks);
    }
}

} 