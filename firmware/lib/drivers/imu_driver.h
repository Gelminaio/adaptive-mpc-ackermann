#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_BNO080_Arduino_Library.h>

#include "vehicle_state.h"

namespace drivers
{
    class ImuDriver
    {
    public:
        ImuDriver(uint8_t i2c_address,
                  uint32_t i2c_frequency_hz);

        bool begin();

        bool read();

        ImuData getData() const;

    private:
        const uint8_t i2c_address_;
        const uint32_t i2c_frequency_hz_;

        BNO080 imu_;
        ImuData latest_;
        bool initialized_ = false;
    };

}