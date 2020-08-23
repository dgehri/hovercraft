#pragma once

#include <estd/algorithm.h>
#include <Arduino.h>
#include <MPU6050.h>

class Gyro
{
public:
    void setup()
    {
        _device.initialize();
        _device.setDLPFMode(2);

        // 0 = +/- 250 degrees/sec | 1 = +/- 500 degrees/sec | 2 = +/- 1000 degrees/sec | 3 =  +/- 2000 degrees/sec
        _device.setFullScaleGyroRange(1);

        calibrate();
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
        const int N = 32;
        int sum = 0;

        for (int i = 0; i < N; ++i)
        {
            sum += _device.getRotationZ();
        }
        _baseline = sum / N;
    } 

private:
    int16_t _baseline;
    mutable MPU6050 _device;
};
