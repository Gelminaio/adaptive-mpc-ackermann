#include "safety_task.h"

#include "config.h"
#include "globals.h"

#include <math.h>
#include "esp_task_wdt.h"

namespace tasks
{

    void safety_task(void *params)
    {
        (void)params;

        const TickType_t period_ticks = pdMS_TO_TICKS(PERIOD_SAFETY_MS);
        TickType_t last_wake = xTaskGetTickCount();

        // subscribe this task to the Task Watchdog Timer.
        esp_task_wdt_add(NULL);

        while (true)
        {
            // feed the hardware watchdog: proves this task is alive.
            esp_task_wdt_reset();

            const uint32_t now_ms = millis();

            g_safety.update(now_ms, g_vehicle_state.wheel_left.velocity_mps, g_vehicle_state.wheel_left.velocity_setpoint_mps, g_vehicle_state.wheel_right.velocity_mps, g_vehicle_state.wheel_right.velocity_setpoint_mps);
            g_vehicle_state.safety_state = g_safety.state();

            vTaskDelayUntil(&last_wake, period_ticks);
        }
    }
}