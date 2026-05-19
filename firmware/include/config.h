#pragma once

// config.h — pinout, hardware constants, timings.

// I2C (BNO085 IMU)
constexpr int PIN_I2C_SDA = 21;
constexpr int PIN_I2C_SCL = 22;
constexpr uint32_t I2C_FREQUENCY_HZ = 400000;
constexpr uint8_t BNO085_I2C_ADDRESS = 0x4A;

// Right BTS7960
constexpr int PIN_MOTOR_R_PWM_FWD = 25;      
constexpr int PIN_MOTOR_R_PWM_REV = 26;        

// Left BTS7960
constexpr int PIN_MOTOR_L_PWM_FWD = 27;      
constexpr int PIN_MOTOR_L_PWM_REV = 14;        

// Encoders
constexpr int PIN_ENCODER_R_A = 34;
constexpr int PIN_ENCODER_R_B = 35;
constexpr int PIN_ENCODER_L_A = 36;
constexpr int PIN_ENCODER_L_B = 39;

// Steering servo
constexpr int PIN_SERVO_STEERING = 33;
constexpr float SERVO_ANGLE_MAX_DEG = 30.0f;
constexpr float SERVO_ANGLE_MIN_DEG = -30.0f;

// PWM pulse-width calibration in microseconds (established through some tests)
constexpr uint32_t SERVO_PULSE_CENTER_US = 1460;   // straight wheels (0° degrees)
constexpr uint32_t SERVO_PULSE_LEFT_US = 960;   // full left  (-30 degrees)
constexpr uint32_t SERVO_PULSE_RIGHT_US = 1960;   // full right (+30 degrees)

// LEDC channel assignment
constexpr int SERVO_LEDC_CHANNEL = 0;
constexpr int MOTOR_R_FWD_LEDC_CHANNEL = 2;
constexpr int MOTOR_R_REV_LEDC_CHANNEL = 3;
constexpr int MOTOR_L_FWD_LEDC_CHANNEL = 4;
constexpr int MOTOR_L_REV_LEDC_CHANNEL = 5;

// Motor control constants
constexpr int16_t  MOTOR_DUTY_MAX = 1023;
constexpr int16_t  MOTOR_DUTY_MIN = -1023;
constexpr uint32_t MOTOR_DEAD_TIME_MS = 50; 

// LED
constexpr int PIN_LED = 2;          

// PWM (LEDC) settings
constexpr uint32_t MOTOR_PWM_FREQ_HZ = 20000;   
constexpr uint8_t  MOTOR_PWM_RESOLUTION = 10;    
constexpr uint32_t SERVO_PWM_FREQ_HZ = 50;      
constexpr uint8_t  SERVO_PWM_RESOLUTION = 16;    

// Vehicle physical parameters
// TODO: refine these values with precise measurements in Phase 5 (System ID)
constexpr float WHEEL_RADIUS_M = 0.0325f; 
constexpr float WHEELBASE_M = 0.165f;   
constexpr float TRACK_WIDTH_M = 0.145f;    
constexpr int   ENCODER_TICKS_PER_REV = 1440;  
constexpr float GEAR_RATIO = 1.0f;    

// Control & safety limits
constexpr float VEHICLE_MAX_LINEAR_VEL_MPS = 2.0f;
constexpr float VEHICLE_MAX_ANGULAR_VEL_RPS = 3.0f;
constexpr uint32_t CMD_TIMEOUT_MS = 500;       

// Task timing (periods in ms)
constexpr uint32_t PERIOD_MOTOR_CONTROL_MS = 10;   
constexpr uint32_t PERIOD_IMU_MS = 5;    
constexpr uint32_t PERIOD_COMMS_MS = 20;   
constexpr uint32_t PERIOD_SAFETY_MS = 20;  
constexpr uint32_t PERIOD_TELEMETRY_MS = 100;  

// Task stack sizes (bytes) and priorities (0 = lowest, 24 = highest)
constexpr uint32_t STACK_MOTOR_CONTROL = 4096;
constexpr uint32_t STACK_IMU = 4096;
constexpr uint32_t STACK_COMMS = 8192;   
constexpr uint32_t STACK_SAFETY = 2048;
constexpr uint32_t STACK_TELEMETRY = 4096;

constexpr UBaseType_t PRIO_SAFETY = 5;
constexpr UBaseType_t PRIO_MOTOR_CONTROL = 4;
constexpr UBaseType_t PRIO_IMU = 3;
constexpr UBaseType_t PRIO_COMMS = 2;
constexpr UBaseType_t PRIO_TELEMETRY = 1;

// Core assignment, ESP32 has 2 cores: 0 and 1
constexpr BaseType_t CORE_CONTROL = 1;   // real-time (critical)
constexpr BaseType_t CORE_COMMS = 0;   // communications and telemetry