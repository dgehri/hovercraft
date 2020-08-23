#pragma once

#define USE_SERVO_LIBRARY

#include <Arduino.h>
#ifdef USE_SERVO_LIBRARY
#   include <Servo.h>
#endif
#include <assert.h>
#include <estd/algorithm.h>

struct Range
{
    int min_us;
    int max_us;
    int start_us;   // value at which motor starts
    int stop_us;    // value at which motor stopps
};

class Motor
{
public:
    Motor(int pin, const Range& range, bool disable = false)
        : _disabled(disable)
        , _pin(pin)
        , _range(range)
        , _value_us(range.min_us)
        , _start_time(0)
    {
        _pwm.writeMicroseconds(range.min_us);
    }

    void setup()
    {
        _pwm.attach(_pin);
    }

    int value() const 
    { 
        return _value_us;
    }

    void set(int value_us, bool clamp = true)
    {
        if (clamp)
        {
            _value_us = estd::clamp(value_us, _range.min_us, _range.max_us);
        }
        else
        {
            _value_us = value_us;
        }

        if (!_disabled)
        {
            _pwm.writeMicroseconds(value_us);
        }
    }

private:
    const int _disabled;
    const int _pin;
    const Range _range;
    Servo _pwm;
    int _value_us;
    uint32_t _start_time;
};
