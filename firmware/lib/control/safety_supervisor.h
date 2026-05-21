#pragma once

#include <Arduino.h>

#include "vehicle_state.h"

namespace control
{
    class SafetySupervisor
    {
    public:
        void arm();
        void disarm();
        void clearEmergency();

        void notifyCommand(uint32_t now_ms);

        void update(uint32_t now_ms, float measured_left, float setpoint_left, float measured_right, float setpoint_right);
        SafetyState state() const { return state_; }
        const char *reason() const { return reason_; }

        // True if motors are allowed to move (only in ARMED).
        bool motionAllowed() const { return state_ == SafetyState::ARMED; }

    private:
        SafetyState state_ = SafetyState::DISARMED;
        const char *reason_ = "";

        uint32_t last_command_ms_ = 0;
        uint32_t stall_start_ms_ = 0;
        bool stall_active_ = false;
    };
}