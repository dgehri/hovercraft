#pragma once

#include <Arduino.h>
#include <stdint.h>

constexpr uint16_t COUNT_PER_MICROS = 2;

// high-precision timer
class Timer
{
public:
    static Timer& instance()
    {
        static Timer timer;
        return timer;
    }

    void setup()
    {
        // backup variables
        _tccr2a_save = TCCR2A;  // first, backup some values
        _tccr2b_save = TCCR2B;  // backup some more values

        // increase the speed of timer2; see below link, as well as the datasheet pg 158-159.
        TCCR2B = (TCCR2B & 0b11111000) | 0x02;  // Timer2 is now faster than default; see here for more info:
                                              // http://playground.arduino.cc/Main/TimerPWMCheatsheet
        // Note: don't forget that when you speed up Timer2 like this you are also affecting any PWM output (using
        // analogWrite) on Pins 3 & 11. Refer to the link just above, as well as to this source here:
        // http://www.oreilly.de/catalog/arduinockbkger/Arduino_Kochbuch_englKap_18.pdf

        // Enable Timer2 overflow interrupt; see datasheet pg. 159-160
        TIMSK2 |= 0b00000001;  // enable Timer2 overflow interrupt. (by making the right-most bit in TIMSK2 a 1)
        // TIMSK2 &= 0b11111110; //use this code to DISABLE the Timer2 overflow interrupt, if you ever wish to do so
        // later. (see datasheet pg. 159-160)

        // set timer2 to "normal" operation mode.  See datasheet pg. 147, 155, & 157-158 (incl. Table 18-8).
        //-This is important so that the timer2 counter, TCNT2, counts only UP and not also down.
        //-To do this we must make WGM22, WGM21, & WGM20, within TCCR2A & TCCR2B, all have values of 0.
        TCCR2A &= 0b11111100;  // set WGM21 & WGM20 to 0 (see datasheet pg. 155).
        TCCR2B &= 0b11110111;  // set WGM22 to 0 (see pg. 158).
    }

    // Get total count (0.5 us increments)
    uint32_t get_count() const
    {
        uint8_t SREG_old = SREG;  // back up the AVR Status Register; see example in datasheet on pg. 14, as well as
                                  // Nick Gammon's "Interrupts" article - http://www.gammon.com.au/forum/?id=11488
        noInterrupts();  // prepare for critical section of code
        uint8_t tcnt2_save = TCNT2;  // grab the counter value from Timer2
        boolean flag_save = bitRead(TIFR2, 0);  // grab the timer2 overflow flag value; see datasheet pg. 160

        if (flag_save)
        {
            // if the overflow flag is set
            tcnt2_save = TCNT2;  // update variable just saved since the overflow flag could have just tripped between
                                 // previously saving the TCNT2 value and reading bit 0 of TIFR2. If this is the case,
                                 // TCNT2 might have just changed from 255 to 0, and so we need to grab the new value of
                                 // TCNT2 to prevent an error of up to 127.5us in any time obtained using the T2 counter
                                 // (ex: T2_micros). (Note: 255 counts / 2 counts/us = 127.5us) Note: this line of code
                                 // DID in fact fix the error just described, in which I periodically saw an error of
                                 // ~127.5us in some values read in by some PWM read code I wrote.
            increment_overflow_count();  // force the overflow count to increment
            TIFR2 |= 0b00000001;  // reset Timer2 overflow flag since we just manually incremented above; see datasheet
                                  // pg. 160; this prevents execution of Timer2's overflow ISR
        }
        uint32_t total_count = _overflow_count * 256 + tcnt2_save;  // get total Timer2 count

        SREG = SREG_old;  // use this method instead, to re-enable interrupts if they were enabled before, or to leave
                          // them disabled if they were disabled before
        return total_count;
    }

    // Reset counters
    void reset()
    {
        _overflow_count = 0;  // reset overflow counter
        TCNT2 = 0;  // reset Timer2 counter
        TIFR2 |= 0b00000001;  // reset Timer2 overflow flag; see datasheet pg. 160; this prevents an immediate execution
                              // of Timer2's overflow ISR
    }

    void increment_overflow_count() const
    {
        ++_overflow_count;
    }

private:
    Timer()
        : _overflow_count(0)
    {}

private:
    mutable volatile uint32_t _overflow_count;
    uint8_t _tccr2a_save;
    uint8_t _tccr2b_save;
};

// Interrupt Service Routine (ISR) for when Timer2's counter overflows; this will occur every 128us
ISR(TIMER2_OVF_vect)  // Timer2's counter has overflowed
{
    Timer::instance().increment_overflow_count();  // increment the timer2 overflow counter
}
