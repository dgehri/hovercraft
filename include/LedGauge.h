#pragma once

#include "Timer.h"
#include <Adafruit_NeoPixel.h>
#include <estd/array.h>
#ifdef __AVR__
#include <avr/power.h>
#endif

constexpr uint8_t NUM_PIXEL = 5;

class LedGauge
{
public:
    LedGauge(uint8_t pin)
        : _pixels(NUM_PIXEL, pin, NEO_GRB + NEO_KHZ800)
    {}

    void setup() { _pixels.begin(); }

    bool canShow() { return _pixels.canShow(); }

    void rainbowCycle(int speedDelayMs)
    {
        if (!canShow())
            return;

        if (Timer::instance().get_count() - prevTimer > static_cast<uint16_t>(COUNT_PER_MICROS * speedDelayMs))
        {
            auto c = wheelColor(((_rbState.pixel * 256 / NUM_PIXEL) + _rbState.phase) & 255);
            _pixels.setPixelColor(_rbState.pixel, c);

            if (++_rbState.pixel == NUM_PIXEL)
            {
                show();
                _rbState.pixel = 0;

                if (++_rbState.phase == 256 * 5)
                {
                    _rbState.phase = 0;
                }
            }
        }
    }

    void showBars(int bars, int speedDelayMs = 1)
    {
        if (!canShow())
            return;

        if (Timer::instance().get_count() - prevTimer > static_cast<uint16_t>(COUNT_PER_MICROS * speedDelayMs))
        {
            _lastBars = bars;

            uint32_t colors[5];
            for (int i = 0; i < 5; ++i)
            {
                colors[i] = Adafruit_NeoPixel::Color(0x00, 0x00, 0xff);
            }

            for (int i = 0; i < bars; ++i)
            {
                colors[i] = Adafruit_NeoPixel::Color(0xff, 0x00, 0x00);
            }

            for (int i = 0; i < 5; ++i)
            {
                setPixelColor(i, colors[i]);
            }

            show();
        }
    }

    void showVoltage(float voltage, int speedDelayMs = 1)
    {
        static const estd::array<float, 6> VOLTAGE_LEVELS = { 0.0f, 7.45f, 7.59f, 7.75f, 8.16f, 999.0f };
        static const estd::array<uint32_t, 5> COLORS = {
            Adafruit_NeoPixel::Color(255, 0, 51), Adafruit_NeoPixel::Color(0xc7, 0x5f, 0x00),
            Adafruit_NeoPixel::Color(0x7b, 0x7e, 0x00), Adafruit_NeoPixel::Color(0x00, 0x8b, 0x00), 
            Adafruit_NeoPixel::Color(0x00, 0x8b, 0x00)
        };
        static constexpr float MARGIN = 0.1f;

        if (!canShow())
            return;

        if (Timer::instance().get_count() - prevTimer < static_cast<uint16_t>(COUNT_PER_MICROS * speedDelayMs))
            return;

        // get bounds of current level
        auto lower_bound = VOLTAGE_LEVELS[estd::clamp(_lastBars - 1, 0, VOLTAGE_LEVELS.size()-1)];
        auto upper_bound = VOLTAGE_LEVELS[estd::clamp(_lastBars, 0, VOLTAGE_LEVELS.size()-1)] + MARGIN;

        int bars = _lastBars;
        if (voltage < lower_bound || voltage > upper_bound)
        {
            bars = 0;
            for (auto level : VOLTAGE_LEVELS)
            {
                if (voltage < level)
                    break;

                ++bars;
            }
        }

        if (bars != _lastBars)
        {
            auto color = COLORS[bars-1];
            _lastBars = bars;

            uint32_t colors[5];
            for (int i = 0; i < 5; ++i)
            {
                colors[i] = 0;
            }

            for (int i = 0; i < bars; ++i)
            {
                colors[i] = color;
            }

            for (int i = 0; i < 5; ++i)
            {
                setPixelColor(i, colors[i]);
            }

            show();
        }
    }

private:
    void setPixelColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b)
    {
        _pixels.setPixelColor(n, 
            Adafruit_NeoPixel::gamma8(r), 
            Adafruit_NeoPixel::gamma8(g),
            Adafruit_NeoPixel::gamma8(b));
    }

    void setPixelColor(uint16_t n, uint32_t color)
    {
        _pixels.setPixelColor(n, Adafruit_NeoPixel::gamma32(color));
    }

    void show()
    {
        prevTimer = Timer::instance().get_count();
        _pixels.show();
        Timer::instance().bump(NUM_PIXEL * 30 * COUNT_PER_MICROS);
    }

    struct RainBowState
    {
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
    uint32_t prevTimer = 0;
    float _lastVoltage = -1.0;
    int _lastBars = 1;
    uint8_t _glow = 0;
};
