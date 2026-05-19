#pragma once

#include <Arduino.h>

// Shared state structures, protected by mutex when accessed.

// Vehicle command (received from ROS via /cmd_vel)
struct VehicleCommand {
    float linear_vel_mps = 0.0f;  
    float angular_vel_rps = 0.0f; 
    uint32_t timestamp_ms = 0;   
    bool armed = false;          
};

// Per-wheel measured state
struct WheelState {
    int32_t encoder_ticks = 0;
    float velocity_mps = 0.0f;     
};

// IMU measurement (BNO085 already fused)
struct ImuData {
    float quat_w = 1.0f, quat_x = 0.0f, quat_y = 0.0f, quat_z = 0.0f;
    float linear_accel_x = 0.0f, linear_accel_y = 0.0f, linear_accel_z = 0.0f;
    float angular_vel_x = 0.0f, angular_vel_y = 0.0f, angular_vel_z = 0.0f;
    uint32_t timestamp_ms = 0;
};

// Complete vehicle state, owned by the system
struct VehicleState {
    WheelState wheel_left;
    WheelState wheel_right;
    ImuData imu;
    float steering_angle_deg = 0.0f;  
    bool emergency_stop = false;
};