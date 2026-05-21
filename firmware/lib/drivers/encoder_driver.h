#pragma once

#include <Arduino.h>
#include <driver/pcnt.h>

namespace drivers
{

    class EncoderDriver
    {
    public:
        EncoderDriver(int pin_a,
                      int pin_b,
                      int pcnt_unit,
                      const char *label,
                      bool invert = false);

        bool begin();

        int32_t getCount();

        float getVelocityTicksPerSec();
        float getVelocityMps();
        float getVelocityMpsFiltered(float ema_alpha);
        void resetCount();

    private:
        static void IRAM_ATTR pcntOverflowHandler(void *arg);

        const int pin_a_;
        const int pin_b_;
        const pcnt_unit_t unit_;
        const char *label_;
        const bool invert_;
        float ema_velocity_tps_ = 0.0f;
        bool ema_initialized_ = false;
        portMUX_TYPE mux_ = portMUX_INITIALIZER_UNLOCKED;

        volatile int32_t overflow_count_ = 0;

        int32_t last_count_for_vel_ = 0;
        uint32_t last_time_for_vel_ms_ = 0;
        float cached_vel_tps_ = 0.0f;

        bool initialized_ = false;
    };

}