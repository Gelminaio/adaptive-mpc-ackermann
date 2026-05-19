#pragma once

#include "servo_driver.h"
#include "motor_driver.h"
#include "encoder_driver.h"
#include "vehicle_state.h"

extern drivers::ServoDriver g_servo;
extern drivers::MotorDriver g_motor_left;
extern drivers::MotorDriver g_motor_right;
extern drivers::EncoderDriver g_encoder_left;
extern drivers::EncoderDriver g_encoder_right;

extern VehicleCommand g_vehicle_command;
extern VehicleState   g_vehicle_state;