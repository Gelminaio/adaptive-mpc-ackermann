#include "comms_task.h"

#include "config.h"

namespace tasks {

void comms_task(void* params) {
    (void)params;

    const TickType_t period_ticks = pdMS_TO_TICKS(PERIOD_COMMS_MS);
    TickType_t last_wake = xTaskGetTickCount();

    uint32_t heartbeat_counter = 0;

    while (true) {
        // TODO (step 3.8): micro-ROS publish/subscribe

        if (++heartbeat_counter >= 50) {  
            heartbeat_counter = 0;
            Serial.printf("[comms] heartbeat @ %lu ms\n", millis());
        }

        vTaskDelayUntil(&last_wake, period_ticks);
    }
}

}