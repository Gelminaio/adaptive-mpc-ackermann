#pragma once

#include <Arduino.h>

namespace control
{
    // Discrete-time PID controller with anti-windup and derivative-on-measurement.
    class PIDController
    {
    public:
        PIDController(float kp,
                      float ki,
                      float kd,
                      float dt_seconds,
                      float integral_clamp,
                      float output_min,
                      float output_max);

        // Compute one PID step. Returns the (clamped) control output.
        float compute(float setpoint, float measurement);

        // Reset internal state (integral and last_measurement).
        void reset();

        // Live tuning: update gains at runtime.
        void setGains(float kp, float ki, float kd);

        // Inspect current gains and internal state (for telemetry/debug).
        float getKp() const { return kp_; }
        float getKi() const { return ki_; }
        float getKd() const { return kd_; }
        float getIntegral() const { return integral_; }

    private:
        float kp_;
        float ki_;
        float kd_;
        const float dt_;
        const float integral_clamp_;
        const float output_min_;
        const float output_max_;

        float integral_ = 0.0f;
        float last_measurement_ = 0.0f;
        bool has_last_measurement_ = false;
    };

}