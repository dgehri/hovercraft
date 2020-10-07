#include "RcPwm.h"
#include <assert.h>
#include <estd/algorithm.h>

/**
 * Converts ticks to microseconds (pre-scaler: 8)
 * 
 * @param ticks Ticks
 * @return uint16_t Duration [us]
 */
static inline uint16_t ticksToUs(uint16_t ticks)
{
    return (ticks * 8) / clockCyclesPerMicrosecond();
}

/**
 * Converts microseconds to ticks (pre-scaler: 8)
 * 
 * @param us Duration to convert [us]
 * @return uint16_t Converted ticks
 */
static inline uint16_t usToTicks(uint16_t us)
{
    return (clockCyclesPerMicrosecond() * us) / 8;
}

static constexpr uint16_t TRIM_DURATION = 2;  // compensation ticks to trim adjust for digitalWrite delays // 12 August 2009
static constexpr uint16_t MIN_PULSE_WIDTH = 544;  // the shortest pulse sent to a pwm
static constexpr uint16_t MAX_PULSE_WIDTH = 2400;  // the longest pulse sent to a pwm
static constexpr uint16_t DEFAULT_PULSE_WIDTH = 1500;  // default pulse width when pwm is attached
static constexpr uint16_t REFRESH_INTERVAL = 25000;  // minumim time to refresh PWM train in microseconds
static constexpr uint16_t INVALID_SERVO = 255;  // flag indicating an invalid pwm index

uint8_t RcPwm::_pwm_count = 0;
volatile int8_t RcPwm::_counter = 0;
volatile bool RcPwm::_needs_run = false;
RcPwm::Pwm RcPwm::_pwms[MAX_PWM_COUNT];


RcPwm::RcPwm()
{
    if (_pwm_count < MAX_PWM_COUNT)
    {
        // assign a pwm index to this instance
        _index = _pwm_count++;

        // store default values
        _pwms[_index].ticks = usToTicks(DEFAULT_PULSE_WIDTH);
    }
    else
    {
        _index = INVALID_SERVO; // too many _pwms
    }
}

uint8_t RcPwm::attach(int pin)
{
    if (_index < MAX_PWM_COUNT)
    {
        pinMode(pin, OUTPUT); // set pwm pin to output
        _pwms[_index].pin.pin_index = pin;

        // initialize the timer if it has not already been initialized
        if (!isTimerActive())
        {
            initISR();
        }

        _pwms[_index].pin.is_active = true; // this must be set after the check for isTimerActive
    }
    return _index;
}

void RcPwm::detach()
{
    _pwms[_index].pin.is_active = false;
}

void RcPwm::writeMicroseconds(uint16_t us)
{
    // calculate and store the values for the given channel
    byte channel = _index;
    if ((channel < MAX_PWM_COUNT)) // ensure channel is valid
    {
        us = estd::clamp(us, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
        us -= TRIM_DURATION;
        auto ticks = usToTicks(us);

        uint8_t oldSREG = SREG;
        cli();
        _pwms[channel].ticks = ticks;
        SREG = oldSREG;
    }
}

uint16_t RcPwm::readMicroseconds() const
{
    return _index != INVALID_SERVO ? ticksToUs(_pwms[_index].ticks) + TRIM_DURATION : 0;
}

bool RcPwm::attached() const
{
    return _pwms[_index].pin.is_active;
}

void RcPwm::runImpl(bool start)
{
    if (_counter < 0)
    {
        if (!start)
        {
            _needs_run = true;
            TCCR1B = 0;
            return;
        }

        // start new PWM train
        TCNT1 = 0;
        TCCR1B = _BV(CS11); // run with pre-scaler 8
    }
    else if (_counter < RcPwm::_pwm_count && _pwms[_counter].pin.is_active)
    {
        // pulse this channel low if activated
        digitalWrite(_pwms[_counter].pin.pin_index, LOW);
    }

    // increment to the next channel
    ++_counter;

    if (_counter < _pwm_count && _counter < MAX_PWM_COUNT)
    {
        OCR1A = TCNT1 + _pwms[_counter].ticks;

        if (_pwms[_counter].pin.is_active)
        {
             // its an active channel so pulse it high
            digitalWrite(_pwms[_counter].pin.pin_index, HIGH);
        }
    }
    else
    {
        // finished all channels so wait for the refresh period to expire before starting over
        if (static_cast<unsigned>(TCNT1) + 4 < usToTicks(REFRESH_INTERVAL)) // allow a few ticks to ensure the next OCR1A not missed
        {
            OCR1A = static_cast<unsigned>(usToTicks(REFRESH_INTERVAL));
        }
        else
        {
            // at least REFRESH_INTERVAL has elapsed
            OCR1A = TCNT1 + 4;
        }

        _counter = -1;
    }
}

bool RcPwm::needsToRun()
{
    return _needs_run;
}

void RcPwm::runNow()
{
    uint8_t oldSREG = SREG;
    cli();

    runImpl(true);
    _needs_run = false;

    SREG = oldSREG;
}

SIGNAL (TIMER1_COMPA_vect)
{
    RcPwm::runImpl(false);
}

void RcPwm::initISR()
{
    TCCR1A = 0; // normal counting mode
    TCCR1B = _BV(CS11); // set prescaler of 8
    TCNT1 = 0; // clear the timer count

    TIFR1 |= _BV(OCF1A); // clear any pending interrupts;
    TIMSK1 |= _BV(OCIE1A); // enable the output compare interrupt
    RcPwm::_needs_run = true;
}

boolean RcPwm::isTimerActive()
{
    // returns true if any pwm is active on this timer
    for (uint8_t c = 0; c < MAX_PWM_COUNT; ++c)
    {
        if (_pwms[c].pin.is_active)
            return true;
    }
    return false;
}
