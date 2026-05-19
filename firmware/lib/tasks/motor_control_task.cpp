#include "motor_control_task.h"

#include "config.h"

namespace tasks {

void motor_control_task(void* params) {
    (void)params;

    const TickType_t period_ticks = pdMS_TO_TICKS(PERIOD_MOTOR_CONTROL_MS);
    TickType_t last_wake = xTaskGetTickCount();

    uint32_t heartbeat_counter = 0;

    while (true) {
        // TODO (step 3.6): read setpoint, run PID, write PWM

        if (++heartbeat_counter >= 100) {  
            heartbeat_counter = 0;
            Serial.printf("[motor_control] heartbeat @ %lu ms\n", millis());
        }

        vTaskDelayUntil(&last_wake, period_ticks);
    }
}

}  