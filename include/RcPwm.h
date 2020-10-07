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

    /**
     * Detach pin
     */
    void detach();

    /**
     * Set PWM duty-cycle to \p value
     * 
     * @param us Duty-cycle [us]
     */
    void writeMicroseconds(uint16_t us);

    /**
     * Current PWM duty-cycle [us]
     * 
     * @return uint16_t Duty-cycle [us]
     */
    uint16_t readMicroseconds() const;

    /**
     * @return \c true If pin attached; \c false otherwise.
     */
    bool attached() const;

    /**
     * Run PWM now
     */
    static void runNow();

    static bool needsToRun();

    static void runImpl(bool start);
private:
    static void initISR();
    static boolean isTimerActive();

    struct PwmPin
    {
        uint8_t pin_index : 6;  // a pin number from 0 to 63
        uint8_t is_active : 1;  // true if this channel is enabled, pin not pulsed if false
    };

    struct Pwm
    {
        PwmPin pin;
        volatile uint16_t ticks;
    } ;

    uint8_t _index;  // index into the channel data for this pwm
    static volatile bool _needs_run;
    static volatile int8_t _counter; // counter for the pwm being pulsed for each timer (or -1 if refresh interval)
    static Pwm _pwms[MAX_PWM_COUNT]; // static array of pwm structures
    static uint8_t _pwm_count; // the total number of attached _pwms
};
