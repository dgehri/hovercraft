#pragma once

#include "Timer.h"
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>
#endif

constexpr uint8_t PIN_NEOPIXEL = 6;
constexpr uint8_t NUM_PIXEL = 5;

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUM_PIXEL, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

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

struct RainBowState
{
    uint32_t prevTimer = 0;
    uint16_t pixel = 0;
    uint16_t phase = 0;
} cycle;

void rainbowCycle(RainBowState& state, int speedDelayMs)
{
    if (Timer::instance().get_count() - state.prevTimer > COUNT_PER_MICROS * speedDelayMs)
    {
        auto c = wheelColor(((state.pixel * 256 / NUM_PIXEL) + state.phase) & 255);
        pixels.setPixelColor(state.pixel, c);
        
        if (++state.pixel == NUM_PIXEL)
        {
            //pixels.show();

            state.prevTimer = Timer::instance().get_count();

            state.pixel = 0;

            if (++state.phase == 256 * 5)
            {
                state.phase = 0;
            }
        }
    }
}
