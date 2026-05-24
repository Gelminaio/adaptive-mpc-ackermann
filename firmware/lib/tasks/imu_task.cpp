#include "imu_task.h"

#include "config.h"
#include "globals.h"

namespace tasks
{

    void imu_task(void *params)
    {
        (void)params;

        const TickType_t period_ticks = pdMS_TO_TICKS(PERIOD_IMU_MS);
        TickType_t last_wake = xTaskGetTickCount();

        while (true)
        {
            // poll the IMU, read() is non-blocking, returns true if new data arrived.
            if (g_imu.read())
            {
                // atomically update the shared state.
                g_vehicle_state.imu = g_imu.getData();
            }

            vTaskDelayUntil(&last_wake, period_ticks);
        }
    }

}