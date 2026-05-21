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

        constexpr float VEL_EMA_ALPHA = 0.2f;

        while (true)
        {
            // diagnostic mode: skip PID entirely, let 'm' command drive motors directly.
            if (g_vehicle_state.manual_duty_override)
            {
                g_vehicle_state.wheel_left.velocity_mps =
                    g_encoder_left.getVelocityMpsFiltered(VEL_EMA_ALPHA);
                g_vehicle_state.wheel_right.velocity_mps =
                    g_encoder_right.getVelocityMpsFiltered(VEL_EMA_ALPHA);
                vTaskDelayUntil(&last_wake, period_ticks);
                continue;
            }

            // (no Ackermann differential yet, I will add that with the kinematics in 3.8).
            float sp_left = g_vehicle_state.wheel_left.velocity_setpoint_mps;
            float sp_right = g_vehicle_state.wheel_right.velocity_setpoint_mps;

            const float meas_left = g_encoder_left.getVelocityMpsFiltered(VEL_EMA_ALPHA);
            const float meas_right = g_encoder_right.getVelocityMpsFiltered(VEL_EMA_ALPHA);

            // ---- SAFETY GATE ----
            const SafetyState st = g_safety.state();

            if (st == SafetyState::DISARMED || st == SafetyState::EMERGENCY)
            {
                // hard off: motors coast, PID reset, no movement whatsoever.
                g_pid_left.reset();
                g_pid_right.reset();
                g_motor_left.coast();
                g_motor_right.coast();
                g_vehicle_state.wheel_left.velocity_mps = meas_left;
                g_vehicle_state.wheel_right.velocity_mps = meas_right;
                g_vehicle_state.wheel_left.pwm_duty = 0;
                g_vehicle_state.wheel_right.pwm_duty = 0;
                vTaskDelayUntil(&last_wake, period_ticks);
                continue;
            }

            if (st == SafetyState::SOFT_STOP)
            {
                // ramp setpoints toward zero instead of cutting abruptly.
                const float ramp_step =
                    SAFETY_SOFTSTOP_RAMP_MPS_PER_S * (PERIOD_MOTOR_CONTROL_MS / 1000.0f);
                auto ramp_to_zero = [&](float v)
                {
                    if (v > ramp_step)
                        return v - ramp_step;
                    if (v < -ramp_step)
                        return v + ramp_step;
                    return 0.0f;
                };
                sp_left = ramp_to_zero(sp_left);
                sp_right = ramp_to_zero(sp_right);
                // write back the ramped setpoints so they persist across cycles.
                g_vehicle_state.wheel_left.velocity_setpoint_mps = sp_left;
                g_vehicle_state.wheel_right.velocity_setpoint_mps = sp_right;
            }

            // ---- PID (ARMED, or ramping during SOFT_STOP) ----
            // if the setpoint is essentially zero, force PWM to 0 and reset PID, this prevents low-speed creep due to static friction.
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
            
            constexpr int16_t STALL_DUTY_LIMIT = 350;
            auto limit_if_stalled = [&](int16_t duty, float sp, float meas) -> int16_t
            {
                if (fabsf(sp) > SAFETY_STALL_SETPOINT_MPS &&
                    fabsf(meas) < SAFETY_STALL_ACTUAL_MPS)
                {
                    if (duty > STALL_DUTY_LIMIT)
                        return STALL_DUTY_LIMIT;
                    if (duty < -STALL_DUTY_LIMIT)
                        return -STALL_DUTY_LIMIT;
                }
                return duty;
            };
            duty_left = limit_if_stalled(duty_left, sp_left, meas_left);
            duty_right = limit_if_stalled(duty_right, sp_right, meas_right);
            
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