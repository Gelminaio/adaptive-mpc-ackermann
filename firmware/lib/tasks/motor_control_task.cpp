#include "motor_control_task.h"

#include "config.h"
#include "globals.h"

#include <math.h>

namespace tasks
{

    void motor_control_task(void *params)
    {
        (void)params;

        const TickType_t period_ticks = pdMS_TO_TICKS(PERIOD_MOTOR_CONTROL_MS);
        TickType_t last_wake = xTaskGetTickCount();

        while (true)
        {
            // (no Ackermann differential yet, I will add that with the kinematics in 3.8).
            const float sp_left = g_vehicle_state.wheel_left.velocity_setpoint_mps;
            const float sp_right = g_vehicle_state.wheel_right.velocity_setpoint_mps;

            constexpr float VEL_EMA_ALPHA = 0.2f;
            const float meas_left = g_encoder_left.getVelocityMpsFiltered(VEL_EMA_ALPHA);
            const float meas_right = g_encoder_right.getVelocityMpsFiltered(VEL_EMA_ALPHA);

            // diagnostic mode: skip PID entirely, let 'm' command drive motors directly.
            if (g_vehicle_state.manual_duty_override)
            {
                g_vehicle_state.wheel_left.velocity_mps = g_encoder_left.getVelocityMps();
                g_vehicle_state.wheel_right.velocity_mps = g_encoder_right.getVelocityMps();
                vTaskDelayUntil(&last_wake, period_ticks);
                continue;
            }

            // if the setpoint is essentially zero, force PWM to be equals to 0 and reset PID, this prevents low-speed creep due to static friction.
            int16_t duty_left, duty_right;

            if (fabsf(sp_left) < PID_DEADBAND_MPS)
            {
                g_pid_left.reset();
                duty_left = 0;
            }
            else
            {
                const float u = g_pid_left.compute(sp_left, meas_left);
                duty_left = static_cast<int16_t>(u);
            }

            if (fabsf(sp_right) < PID_DEADBAND_MPS)
            {
                g_pid_right.reset();
                duty_right = 0;
            }
            else
            {
                const float u = g_pid_right.compute(sp_right, meas_right);
                duty_right = static_cast<int16_t>(u);
            }

            g_motor_left.setDuty(duty_left);
            g_motor_right.setDuty(duty_right);

            // cache for telemetry
            g_vehicle_state.wheel_left.velocity_mps = meas_left;
            g_vehicle_state.wheel_right.velocity_mps = meas_right;
            g_vehicle_state.wheel_left.pwm_duty = duty_left;
            g_vehicle_state.wheel_right.pwm_duty = duty_right;

            vTaskDelayUntil(&last_wake, period_ticks);
        }
    }

}