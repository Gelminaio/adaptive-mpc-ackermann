#include "imu_task.h"

#include "config.h"

namespace tasks {

void imu_task(void* params) {
    (void)params;

    const TickType_t period_ticks = pdMS_TO_TICKS(PERIOD_IMU_MS);
    TickType_t last_wake = xTaskGetTickCount();

    uint32_t heartbeat_counter = 0;

    while (true) {
        // TODO (step 3.5): read BNO085 over I2C

        if (++heartbeat_counter >= 200) { 
            heartbeat_counter = 0;
            Serial.printf("[imu] heartbeat @ %lu ms\n", millis());
        }

        vTaskDelayUntil(&last_wake, period_ticks);
    }
}

}  