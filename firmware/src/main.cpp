#include <Arduino.h>

#include "globals.h"
#include "servo_driver.h"
#include "motor_driver.h"
#include "config.h"
#include "vehicle_state.h"

#include "motor_control_task.h"
#include "imu_task.h"
#include "comms_task.h"
#include "safety_task.h"
#include "telemetry_task.h"


// global shared state, in future steps will be protected with mutex.
drivers::ServoDriver g_servo;
drivers::MotorDriver g_motor_left(PIN_MOTOR_L_PWM_FWD, PIN_MOTOR_L_PWM_REV, MOTOR_L_FWD_LEDC_CHANNEL, MOTOR_L_REV_LEDC_CHANNEL, "left", true);
drivers::MotorDriver g_motor_right(PIN_MOTOR_R_PWM_FWD, PIN_MOTOR_R_PWM_REV, MOTOR_R_FWD_LEDC_CHANNEL, MOTOR_R_REV_LEDC_CHANNEL, "right");
VehicleState         g_vehicle_state;
VehicleCommand       g_vehicle_command;

// Setup: hardware bring-up + task creation, then return forever.
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== Adaptive MPC Ackermann — Firmware Boot ===");
    Serial.printf("ESP-IDF: %s\n", esp_get_idf_version());
    Serial.printf("Free heap: %u bytes\n", ESP.getFreeHeap());

    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);

    // Initialize hardware drivers
    //servo 
    if (!g_servo.begin()) {
        Serial.println("FATAL: servo driver init failed");
    }
    //motors left and right
    if (!g_motor_left.begin()) {
        Serial.println("FATAL: left motor driver init failed");
    }
    if (!g_motor_right.begin()) {
        Serial.println("FATAL: right motor driver init failed");
    }
    // Create FreeRTOS tasks
    xTaskCreatePinnedToCore(
        tasks::safety_task,         "safety",
        STACK_SAFETY,               nullptr,
        PRIO_SAFETY,                nullptr,
        CORE_CONTROL);

    xTaskCreatePinnedToCore(
        tasks::motor_control_task,  "motor_ctrl",
        STACK_MOTOR_CONTROL,        nullptr,
        PRIO_MOTOR_CONTROL,         nullptr,
        CORE_CONTROL);

    xTaskCreatePinnedToCore(
        tasks::imu_task,            "imu",
        STACK_IMU,                  nullptr,
        PRIO_IMU,                   nullptr,
        CORE_CONTROL);

    xTaskCreatePinnedToCore(
        tasks::comms_task,          "comms",
        STACK_COMMS,                nullptr,
        PRIO_COMMS,                 nullptr,
        CORE_COMMS);

    xTaskCreatePinnedToCore(
        tasks::telemetry_task,      "telemetry",
        STACK_TELEMETRY,            nullptr,
        PRIO_TELEMETRY,             nullptr,
        CORE_COMMS);

    Serial.println("All tasks created. Entering idle.\n");
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}