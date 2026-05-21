#include "comms_task.h"

#include "config.h"
#include "globals.h"

namespace tasks
{

    static void process_line(const String &line)
    {
        String trimmed = line;
        trimmed.trim();
        if (trimmed.length() == 0)
            return;

        const char cmd = trimmed[0];
        String args = trimmed.substring(1);
        args.trim();

        switch (cmd)
        {
        case 's':
        {
            const float requested = args.toFloat();
            const float actual = g_servo.setAngle(requested);
            if (fabsf(actual - requested) > 0.01f)
            {
                Serial.printf("[comms] steering CLAMPED %.2f -> %.2f deg\n",
                              requested, actual);
            }
            else
            {
                Serial.printf("[comms] steering set to %.2f deg\n", actual);
            }
            break;
        }
        case 'm':
        {
            int left = 0;
            int right = 0;
            const int parsed = sscanf(args.c_str(), "%d %d", &left, &right);
            if (parsed != 2)
            {
                Serial.println("[comms] usage: m <left_duty> <right_duty>");
                break;
            }
            g_vehicle_state.wheel_left.velocity_setpoint_mps = 0.0f;
            g_vehicle_state.wheel_right.velocity_setpoint_mps = 0.0f;
            const int16_t actual_l = g_motor_left.setDuty(static_cast<int16_t>(left));
            const int16_t actual_r = g_motor_right.setDuty(static_cast<int16_t>(right));
            Serial.printf("[comms] motors set L=%d R=%d\n", actual_l, actual_r);
            break;
        }
        case 'e':
        {
            const int32_t cl = g_encoder_left.getCount();
            const int32_t cr = g_encoder_right.getCount();
            const float vl = g_vehicle_state.wheel_left.velocity_mps;
            const float vr = g_vehicle_state.wheel_right.velocity_mps;
            Serial.printf("[comms] enc L: %ld ticks, %.3f m/s | R: %ld ticks, %.3f m/s\n",
                          cl, vl, cr, vr);
            break;
        }
        case 'r':
        {
            // Reset both encoder counts
            g_encoder_left.resetCount();
            g_encoder_right.resetCount();
            Serial.println("[comms] encoder counts reset to zero");
            break;
        }
        case 'i':
        {
            // prints IMU state
            const ImuData &imu = g_vehicle_state.imu;
            if (!imu.valid)
            {
                Serial.println("[comms] IMU: no valid data yet");
                break;
            }
            Serial.printf("[comms] IMU quat: w=%.3f x=%.3f y=%.3f z=%.3f (accuracy=%u)\n", imu.qw, imu.qx, imu.qy, imu.qz, imu.accuracy);
            Serial.printf("[comms] IMU lin_acc: %.3f %.3f %.3f m/s^2\n", imu.lin_acc_x, imu.lin_acc_y, imu.lin_acc_z);
            Serial.printf("[comms] IMU gyro:    %.3f %.3f %.3f rad/s\n", imu.gyro_x, imu.gyro_y, imu.gyro_z);
            break;
        }
        case 'v':
        {
            // set velocity setpoint for both wheels
            float vl = 0.0f, vr = 0.0f;
            const int parsed = sscanf(args.c_str(), "%f %f", &vl, &vr);
            if (parsed != 2)
            {
                Serial.println("[comms] usage: v <left_mps> <right_mps>");
                break;
            }
            // clamp to safety limits
            if (vl > VEHICLE_MAX_LINEAR_VEL_MPS)
                vl = VEHICLE_MAX_LINEAR_VEL_MPS;
            if (vl < -VEHICLE_MAX_LINEAR_VEL_MPS)
                vl = -VEHICLE_MAX_LINEAR_VEL_MPS;
            if (vr > VEHICLE_MAX_LINEAR_VEL_MPS)
                vr = VEHICLE_MAX_LINEAR_VEL_MPS;
            if (vr < -VEHICLE_MAX_LINEAR_VEL_MPS)
                vr = -VEHICLE_MAX_LINEAR_VEL_MPS;
            g_vehicle_state.wheel_left.velocity_setpoint_mps = vl;
            g_vehicle_state.wheel_right.velocity_setpoint_mps = vr;
            Serial.printf("[comms] velocity setpoints: L=%.3f R=%.3f m/s\n", vl, vr);
            break;
        }
        case 'p':
        {
            // tune PID gains live
            float kp = 0, ki = 0, kd = 0;
            const int parsed = sscanf(args.c_str(), "%f %f %f", &kp, &ki, &kd);
            if (parsed != 3)
            {
                Serial.printf("[comms] current gains: Kp=%.1f Ki=%.1f Kd=%.3f\n",
                              g_pid_left.getKp(), g_pid_left.getKi(), g_pid_left.getKd());
                Serial.println("[comms] usage: p <kp> <ki> <kd>");
                break;
            }
            g_pid_left.setGains(kp, ki, kd);
            g_pid_right.setGains(kp, ki, kd);
            g_pid_left.reset();
            g_pid_right.reset();
            Serial.printf("[comms] PID gains updated: Kp=%.2f Ki=%.2f Kd=%.4f\n", kp, ki, kd);
            break;
        }
        case 'o':
        {
            // toggle manual duty override (diagnostics: disables PID)
            g_vehicle_state.manual_duty_override = !g_vehicle_state.manual_duty_override;
            // safety: stop motors on toggle
            g_motor_left.coast();
            g_motor_right.coast();
            Serial.printf("[comms] manual override = %s (motors coasted)\n", g_vehicle_state.manual_duty_override ? "ON" : "OFF");
            break;
        }
        case 'x':
        {
            // emergency coast!
            g_motor_left.coast();
            g_motor_right.coast();
            Serial.println("[comms] EMERGENCY COAST: motors stopped");
            break;
        }
        case '?':
        {
            Serial.printf("[comms] servo=%.2f deg, motor L=%d, motor R=%d\n",
                          g_servo.getAngle(),
                          g_motor_left.getDuty(),
                          g_motor_right.getDuty());
            break;
        }
        default:
            Serial.printf("[comms] unknown command: '%c'\n", cmd);
            break;
        }
    }

    void comms_task(void *params)
    {
        (void)params;

        const TickType_t period_ticks = pdMS_TO_TICKS(PERIOD_COMMS_MS);
        TickType_t last_wake = xTaskGetTickCount();

        String line_buffer;
        uint32_t heartbeat_counter = 0;

        Serial.println("[comms] ready. Commands:");
        Serial.println("[comms]   s <angle>          → steering angle (deg)");
        Serial.println("[comms]   m <left> <right>   → motor duty (-1023..+1023)");
        Serial.println("[comms]   e                  → print encoder state");
        Serial.println("[comms]   r                  → reset encoder counts");
        Serial.println("[comms]   i                  → print IMU state");
        Serial.println("[comms]   x                  → emergency coast");
        Serial.println("[comms]   ?                  → query state");
        Serial.println("[comms]   v <vL> <vR>        → velocity setpoints (m/s)");
        Serial.println("[comms]   p <kp> <ki> <kd>   → tune PID gains live (both motors)");

        while (true)
        {
            while (Serial.available())
            {
                const char c = static_cast<char>(Serial.read());
                if (c == '\n' || c == '\r')
                {
                    if (line_buffer.length() > 0)
                    {
                        process_line(line_buffer);
                        line_buffer = "";
                    }
                }
                else
                {
                    line_buffer += c;
                    if (line_buffer.length() > 64)
                    {
                        line_buffer = "";
                    }
                }
            }

            if (++heartbeat_counter >= 50)
            {
                heartbeat_counter = 0;
            }

            vTaskDelayUntil(&last_wake, period_ticks);
        }
    }

}