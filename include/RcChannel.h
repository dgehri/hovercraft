#pragma once
#include <Arduino.h>

class RcChannel
{
public:
    static constexpr auto AVG_CNT = 4;

    RcChannel(uint8_t pin, uint32_t init_pulse_length)
        : _pin(pin)
        , _mask(bit(pin))
        , _start(0)
        , _pulse_length(init_pulse_length)
        , _prev_pind(PIND)
    {
    }

    void setup() { pci_setup(_pin); }

    // pulse length in us
    uint32_t pulse_length() const { return _pulse_length; }

    bool rx(uint8_t pind, uint32_t us) __attribute__((always_inline))
    {
        bool falling = false;

        if ((pind ^ _prev_pind) & _mask)
        {
            if ((pind & _mask) != 0)
            {
                // rising
                _start = us;
            }
            else if (_start != 0)
            {
                // falling
                _pulse_length = us - _start;
                _start = 0;
                falling = true;
            }

            _prev_pind = pind;
        }

        return falling;
    }

private:
    static void pci_setup(byte pin)
    {
        *digitalPinToPCMSK(pin) |= bit(digitalPinToPCMSKbit(pin));  // enable pin
        PCIFR |= bit(digitalPinToPCICRbit(pin));  // clear any outstanding interrupt
        PCICR |= bit(digitalPinToPCICRbit(pin));  // enable interrupt for the group
    }

private:
    const uint8_t _pin;
    const uint8_t _mask;
    volatile uint32_t _start;
    volatile uint32_t _pulse_length;
    volatile uint8_t _prev_pind;
};
