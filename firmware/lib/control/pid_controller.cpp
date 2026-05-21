#include "pid_controller.h"

namespace control
{

    PIDController::PIDController(float kp,
                                 float ki,
                                 float kd,
                                 float dt_seconds,
                                 float integral_clamp,
                                 float output_min,
                                 float output_max)
        : kp_(kp),
          ki_(ki),
          kd_(kd),
          dt_(dt_seconds),
          integral_clamp_(integral_clamp),
          output_min_(output_min),
          output_max_(output_max) {}

    float PIDController::compute(float setpoint, float measurement)
    {
        const float error = setpoint - measurement;

        // Proportional term (Kp*e(t))
        const float p_term = kp_ * error;

        // Integral term (Ki*∫e(t)dt)with anti-windup clamping)
        integral_ += error * dt_;
        if (integral_ > integral_clamp_)
            integral_ = integral_clamp_;
        if (integral_ < -integral_clamp_)
            integral_ = -integral_clamp_;
        const float i_term = ki_ * integral_;

        // Derivative term (Kd*de(t)/dt) — on measurement, not on error.
        // I use d(measurement)/dt with a negative sign because:
        // d(error)/dt = -d(measurement)/dt  (setpoint considered constant)
        float d_term = 0.0f;
        if (has_last_measurement_)
        {
            const float d_meas = (measurement - last_measurement_) / dt_;
            d_term = -kd_ * d_meas;
        }
        last_measurement_ = measurement;
        has_last_measurement_ = true;

        // Sum and clamp
        float output = p_term + i_term + d_term;
        if (output > output_max_)
            output = output_max_;
        if (output < output_min_)
            output = output_min_;

        return output;
    }

    void PIDController::reset()
    {
        integral_ = 0.0f;
        last_measurement_ = 0.0f;
        has_last_measurement_ = false;
    }

    void PIDController::setGains(float kp, float ki, float kd)
    {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

}