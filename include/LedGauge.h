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
                colors[i] = Adafruit_NeoPixel::Color(0x00, 0x00, 0xff);
            }

            for (int i = 0; i < 5; ++i)
            {
                setPixelColor(i, colors[i]);
            }

            show();
        }
    }

    void showVoltage(float voltage, bool glow, int speedDelayMs = 1)
    {
        if (!canShow())
            return;

        if (Timer::instance().get_count() - prevTimer > static_cast<uint16_t>(COUNT_PER_MICROS * speedDelayMs))
        {
            int bars;
            uint32_t color;

            float m = 1.0;
            if (glow)
            {
                _glow += 16;
                m *= Adafruit_NeoPixel::sine8(_glow);
            }

            // hysteresis
            auto v = voltage;
            if (v > _lastVoltage)
            {
                v -= 0.1;
            }
            _lastVoltage = voltage;

            if (v > 8.16f)
            {
                bars = 5;
                color = Adafruit_NeoPixel::Color(m*0x00, m*0x8b, m*0x00);
            }
            else if (v > 7.75f)
            {
                bars = 4;
                color = Adafruit_NeoPixel::Color(m*0x00, m*0x8b, m*0x00);
            }
            else if (v > 7.59)
            {
                bars = 3;
                color = Adafruit_NeoPixel::Color(m*0x7b, m*0x7e, m*0x00);
            }
            else if (v > 7.45)
            {
                bars = 2;
                color = Adafruit_NeoPixel::Color(m*0xc7, m*0x5f, m*0x00);
            }
            else
            {
                bars = 1;
                color = Adafruit_NeoPixel::Color(m*255, m*0, m*51);
            }

            if (bars != _lastBars)
            {
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
    int _lastBars = -1;
    uint8_t _glow = 0;
};
