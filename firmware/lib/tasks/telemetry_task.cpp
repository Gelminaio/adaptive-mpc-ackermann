#include "telemetry_task.h"

#include "config.h"
#include "globals.h"

namespace tasks
{

    void telemetry_task(void *params)
    {
        (void)params;

        const TickType_t period_ticks = pdMS_TO_TICKS(PERIOD_TELEMETRY_MS);
        TickType_t last_wake = xTaskGetTickCount();

        while (true)
        {
            const uint32_t t_ms = millis();
            const int32_t cl = g_encoder_left.getCount();
            const int32_t cr = g_encoder_right.getCount();
            const float vl = g_encoder_left.getVelocityMps();
            const float vr = g_encoder_right.getVelocityMps();

            Serial.printf("TELEM,%lu,%ld,%ld,%.3f,%.3f\n",
                          t_ms, cl, cr, vl, vr);

            vTaskDelayUntil(&last_wake, period_ticks);
        }
    }

}