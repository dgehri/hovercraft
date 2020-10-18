#pragma once

#include "RcPwm.h"
#include <Arduino.h>
#include <assert.h>
#include <estd/algorithm.h>

struct Range
{
    uint16_t min_us;
    uint16_t max_us;
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

    void setup() { _pwm.attach(_pin); }

    int value() const { return _value_us; }

    void disable() { _disabled = true; }
    void enable() { _disabled = false; }

    void setFiltered(uint16_t value_us, uint16_t midValue, uint16_t maxStep, bool clamp = true)
    {
        if (value_us > midValue && value_us > _value_us && (value_us - _value_us) > maxStep)
        {
            value_us = _value_us + maxStep;
        }
        else if (value_us < midValue && value_us < _value_us && (_value_us - value_us) > maxStep)
        {
            value_us = _value_us - maxStep;
        }

        set(value_us, clamp);
    }

    void set(uint16_t value_us, bool clamp = true)
    {
        if (clamp)
        {
            value_us = estd::clamp(value_us, _range.min_us, _range.max_us);
        }

        _value_us = value_us;

        if (!_disabled)
        {
            _pwm.writeMicroseconds(value_us);
        }
    }

private:
    uint16_t _disabled;
    const int _pin;
    const Range _range;
    RcPwm _pwm;
    uint16_t _value_us;
    uint32_t _start_time;
};
