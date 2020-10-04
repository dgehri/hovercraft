#pragma once

#include "eeprom_util.h"
#include <estd/algorithm.h>
#include <Arduino.h>
#include <MPU6050.h>

static constexpr auto IS_INIT_VALUE = 42;

class Gyro
{
public:
    void setup()
    {
        _device.initialize();
        _device.setDLPFMode(2);

        // 0 = +/- 250 degrees/sec | 1 = +/- 500 degrees/sec | 2 = +/- 1000 degrees/sec | 3 =  +/- 2000 degrees/sec
        _device.setFullScaleGyroRange(1);

        if (EEPROM.read(EEPROM_IS_INIT_ADDR) == IS_INIT_VALUE)
        {
            _baseline = eeprom_read_int(EEPROM_GYRO_BASELINE_ADDR);
        }
    }

    int16_t read() const
    {
        // remove baseline
        int16_t gz = _device.getRotationZ() - _baseline;

        // write_rc_outputs ~degrees per second (assuming gyro mode 2)
        return gz / 16;
    }

    int16_t baseline() const { return _baseline; }

    void calibrate()
    {
        const int16_t N = 32;
        int32_t sum = 0;

        for (int16_t i = 0; i < N; ++i)
        {
            sum += _device.getRotationZ();
        }
        _baseline = static_cast<int16_t>(sum / N);

        eeprom_write(EEPROM_GYRO_BASELINE_ADDR, _baseline);
        EEPROM.write(EEPROM_IS_INIT_ADDR, IS_INIT_VALUE);
    } 

private:
    int16_t _baseline;
    mutable MPU6050 _device;
};
