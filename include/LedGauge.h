#pragma once

#include "Timer.h"
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif

constexpr uint8_t NUM_PIXEL = 5;

class LedGauge
{
public:
    LedGauge(uint8_t pin) :
        _pixels(NUM_PIXEL, pin, NEO_GRB + NEO_KHZ800)
    {}

    void setup()
    {
        _pixels.begin();
    }

    void rainbowCycle(int speedDelayMs)
    {
        if (Timer::instance().get_count() - _rbState.prevTimer > static_cast<uint16_t>(COUNT_PER_MICROS * speedDelayMs))
        {
            auto c = wheelColor(((_rbState.pixel * 256 / NUM_PIXEL) + _rbState.phase) & 255);
            _pixels.setPixelColor(_rbState.pixel, c);
            
            if (++_rbState.pixel == NUM_PIXEL)
            {
                _pixels.show();

                _rbState.prevTimer = Timer::instance().get_count();

                _rbState.pixel = 0;

                if (++_rbState.phase == 256 * 5)
                {
                    _rbState.phase = 0;
                }
            }
        }
    }

private:
    struct RainBowState
    {
        uint32_t prevTimer = 0;
        uint16_t pixel = 0;
        uint16_t phase = 0;
    };

    uint32_t wheelColor(byte wheelPos)
    {
        if (wheelPos < 85)
            return Adafruit_NeoPixel::Color(wheelPos * 3, 255 - wheelPos * 3, 0);
        else if (wheelPos < 170)
        {
            wheelPos -= 85;
            return Adafruit_NeoPixel::Color(255 - wheelPos * 3, 0, wheelPos * 3);
        }
        else
        {
            wheelPos -= 170;
            return Adafruit_NeoPixel::Color(0, wheelPos * 3, 255 - wheelPos * 3);
        }
    }

private:
    Adafruit_NeoPixel _pixels;
    RainBowState _rbState;
};
