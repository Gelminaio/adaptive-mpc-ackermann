#include "telemetry_task.h"

#include "config.h"
#include "globals.h"

#include <math.h>

namespace tasks
{

    // Convert quaternion to Euler angles (yaw, pitch, roll in radiants).
    // Standard ZYX intrinsic convention.
    static void quaternionToEuler(float qw, float qx, float qy, float qz,
                                  float &yaw, float &pitch, float &roll)
    {
        // ROLL (X-axis rotation)
        const float sinr_cosp = 2.0f * (qw * qx + qy * qz);
        const float cosr_cosp = 1.0f - 2.0f * (qx * qx + qy * qy);
        roll = atan2f(sinr_cosp, cosr_cosp);

        // PITCH (Y-axis rotation)
        const float sinp = 2.0f * (qw * qy - qz * qx);
        if (fabsf(sinp) >= 1.0f)
        {
            pitch = copysignf(static_cast<float>(M_PI) / 2.0f, sinp); // gimbal lock
        }
        else
        {
            pitch = asinf(sinp);
        }

        // YAW (Z-axis rotation)
        const float siny_cosp = 2.0f * (qw * qz + qx * qy);
        const float cosy_cosp = 1.0f - 2.0f * (qy * qy + qz * qz);
        yaw = atan2f(siny_cosp, cosy_cosp);
    }

    void telemetry_task(void *params)
    {
        (void)params;

        const TickType_t period_ticks = pdMS_TO_TICKS(PERIOD_TELEMETRY_MS);
        TickType_t last_wake = xTaskGetTickCount();

        while (true)
        {
            const uint32_t t_ms = millis();

            // Encoders
            const int32_t cl = g_encoder_left.getCount();
            const int32_t cr = g_encoder_right.getCount();
            const float vl = g_vehicle_state.wheel_left.velocity_mps;
            const float vr = g_vehicle_state.wheel_right.velocity_mps;

            // IMU
            const ImuData &imu = g_vehicle_state.imu;
            float yaw_rad, pitch_rad, roll_rad;
            quaternionToEuler(imu.qw, imu.qx, imu.qy, imu.qz, yaw_rad, pitch_rad, roll_rad);
            const float yaw_deg = yaw_rad * 180.0f / static_cast<float>(M_PI);
            const float pitch_deg = pitch_rad * 180.0f / static_cast<float>(M_PI);
            const float roll_deg = roll_rad * 180.0f / static_cast<float>(M_PI);

            Serial.printf("TELEM,%lu,%ld,%ld,%.3f,%.3f,"
                          "%.2f,%.2f,%.2f,"
                          "%.3f,%.3f,%.3f,"
                          "%.3f,%.3f,%.3f,%u\n",
                          t_ms, cl, cr, vl, vr,
                          yaw_deg, pitch_deg, roll_deg,
                          imu.lin_acc_x, imu.lin_acc_y, imu.lin_acc_z,
                          imu.gyro_x, imu.gyro_y, imu.gyro_z,
                          imu.accuracy);

            vTaskDelayUntil(&last_wake, period_ticks);
        }
    }

}