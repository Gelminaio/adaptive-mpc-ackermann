#pragma once

#include "servo_driver.h"
#include "vehicle_state.h"

// singletons, accessed by multiple tasks.
extern drivers::ServoDriver g_servo;
extern VehicleCommand       g_vehicle_command;
extern VehicleState         g_vehicle_state;