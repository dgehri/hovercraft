#pragma once

#include <avr/interrupt.h>
#include <Arduino.h>

#define MAX_PWM_COUNT 12

class RcPwm
{
public:
    RcPwm();

    /**
     * Attach the given pin to the next free channel, sets pinMode, returns channel number or 0 if failure
     */
    uint8_t attach(int pin);

    void detach();
    void writeMicroseconds(uint16_t value);  // Write pulse width in microseconds
    uint16_t readMicroseconds() const;  // returns current pulse width in microseconds for this pwm (was read_us() in first release)
    bool attached() const;  // return true if this pwm is attached, otherwise false

    static void isr();
private:
    static boolean isTimerActive();

    struct PwmPin
    {
        uint8_t nbr : 6;  // a pin number from 0 to 63
        uint8_t isActive : 1;  // true if this channel is enabled, pin not pulsed if false
    };

    struct Pwm
    {
        PwmPin pin;
        volatile unsigned int ticks;
    } ;

    uint8_t _index;  // index into the channel data for this pwm
    static volatile int8_t _counter; // counter for the pwm being pulsed for each timer (or -1 if refresh interval)
    static Pwm _pwms[MAX_PWM_COUNT]; // static array of pwm structures
    static uint8_t _pwmCount; // the total number of attached _pwms
};
