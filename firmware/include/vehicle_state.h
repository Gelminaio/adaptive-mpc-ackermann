#pragma once

#include <Arduino.h>

// Safety state machine
enum class SafetyState : uint8_t
{
    DISARMED = 0,  // boot state: motors hard-off, commands ignored
    ARMED = 1,     // normal operation: commands accepted
    SOFT_STOP = 2, // command timeout: ramping down to zero
    EMERGENCY = 3  // stall or fault: motors hard-off until re-arm
};

// Shared state structures, protected by mutex when accessed.

// Vehicle command (received from ROS via /cmd_vel)
struct VehicleCommand
{
    float linear_vel_mps = 0.0f;
    float angular_vel_rps = 0.0f;
    float steering_angle_deg = 0.0f;
    uint32_t timestamp_ms = 0;
    bool armed = false;
};

// Per-wheel measured + commanded state
struct WheelState
{
    int32_t encoder_ticks = 0;
    float velocity_mps = 0.0f;
    float velocity_setpoint_mps = 0.0f;
    int16_t pwm_duty = 0;
};

// IMU measurement (BNO085 already fused)
struct ImuData
{
    // Quaternion (orientation in world frame)
    float qw = 1.0f;
    float qx = 0.0f;
    float qy = 0.0f;
    float qz = 0.0f;

    // Linear acceleration (gravity removed, body frame, m/s^2)
    float lin_acc_x = 0.0f;
    float lin_acc_y = 0.0f;
    float lin_acc_z = 0.0f;

    // Angular velocity (body frame, rad/s)
    float gyro_x = 0.0f;
    float gyro_y = 0.0f;
    float gyro_z = 0.0f;

    // Quality / freshness
    uint32_t timestamp_ms = 0;
    uint8_t accuracy = 0; // 0=unreliable, 1=low, 2=med, 3=high
    bool valid = false;
};

// Complete vehicle state, owned by the system
struct VehicleState
{
    ImuData imu;
    WheelState wheel_left;
    WheelState wheel_right;
    float steering_angle_deg = 0.0f;
    bool emergency_stop = false;
    SafetyState safety_state = SafetyState::DISARMED;
    uint32_t last_command_ms = 0;
    const char *emergency_reason = ""; // why we entered EMERGENCY (for logging)
    bool manual_duty_override = false; // just for diagnostics
};