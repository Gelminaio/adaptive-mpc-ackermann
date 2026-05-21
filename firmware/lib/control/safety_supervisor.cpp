#include "safety_supervisor.h"

#include "config.h"

namespace control
{
    void SafetySupervisor::arm()
    {
        if (state_ == SafetyState::EMERGENCY)
        {
            Serial.println("[safety] cannot ARM: clear emergency first ('clear')");
            return;
        }
        state_ = SafetyState::ARMED;
        reason_ = "";
        Serial.println("[safety] state -> ARMED");
    }

    void SafetySupervisor::disarm()
    {
        state_ = SafetyState::DISARMED;
        reason_ = "";
        Serial.println("[safety] state -> DISARMED");
    }

    void SafetySupervisor::clearEmergency()
    {
        if (state_ == SafetyState::EMERGENCY)
        {
            state_ = SafetyState::DISARMED;
            reason_ = "";
            stall_active_ = false;
            Serial.println("[safety] emergency cleared -> DISARMED (send 'arm' to enable)");
        }
    }

    void SafetySupervisor::notifyCommand(uint32_t now_ms)
    {
        last_command_ms_ = now_ms;
    }

    void SafetySupervisor::update(uint32_t now_ms, float measured_left, float setpoint_left, float measured_right, float setpoint_right)
    {
        switch (state_)
        {
        case SafetyState::DISARMED:
        case SafetyState::EMERGENCY:
            stall_active_ = false;
            break;

        case SafetyState::ARMED:
        {
            // 1) Command timeout → SOFT_STOP
            if (last_command_ms_ != 0 &&
                (now_ms - last_command_ms_) > SAFETY_CMD_TIMEOUT_MS)
            {
                state_ = SafetyState::SOFT_STOP;
                reason_ = "command timeout";
                Serial.println("[safety] command timeout -> SOFT_STOP");
                break;
            }

            // 2) Per-wheel stall: ANY wheel commanded to move but not moving.
            const bool left_stalled =
                (fabsf(setpoint_left) > SAFETY_STALL_SETPOINT_MPS) &&
                (fabsf(measured_left) < SAFETY_STALL_ACTUAL_MPS);
            const bool right_stalled =
                (fabsf(setpoint_right) > SAFETY_STALL_SETPOINT_MPS) &&
                (fabsf(measured_right) < SAFETY_STALL_ACTUAL_MPS);

            if (left_stalled || right_stalled)
            {
                if (!stall_active_)
                {
                    stall_active_ = true;
                    stall_start_ms_ = now_ms;
                }
                else if ((now_ms - stall_start_ms_) > SAFETY_STALL_TIME_MS)
                {
                    state_ = SafetyState::EMERGENCY;
                    reason_ = left_stalled ? "stall: left wheel" : "stall: right wheel";
                    stall_active_ = false;
                    Serial.printf("[safety] STALL (%s) -> EMERGENCY (motors locked)\n",
                                  reason_);
                }
            }
            else
            {
                stall_active_ = false;
            }
            break;
        }

        case SafetyState::SOFT_STOP:
            if ((now_ms - last_command_ms_) <= SAFETY_CMD_TIMEOUT_MS)
            {
                state_ = SafetyState::ARMED;
                reason_ = "";
                Serial.println("[safety] command resumed -> ARMED");
            }
            break;
        }
    }
}