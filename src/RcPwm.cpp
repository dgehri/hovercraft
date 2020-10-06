#include "RcPwm.h"

#define US_TO_TICKS(_us) \
    ((clockCyclesPerMicrosecond() * (_us)) / 8)  // converts microseconds to tick (assumes prescale of 8)  // 12 Aug 2009
#define TICKS_TO_US(_ticks) \
    (((unsigned)(_ticks) * 8) / clockCyclesPerMicrosecond())  // converts from ticks back to microseconds


#define TRIM_DURATION 2  // compensation ticks to trim adjust for digitalWrite delays // 12 August 2009
#define MIN_PULSE_WIDTH 544  // the shortest pulse sent to a pwm
#define MAX_PULSE_WIDTH 2400  // the longest pulse sent to a pwm
#define DEFAULT_PULSE_WIDTH 1500  // default pulse width when pwm is attached
#define REFRESH_INTERVAL 20000  // minumim time to refresh _pwms in microseconds
#define INVALID_SERVO 255  // flag indicating an invalid pwm index


uint8_t RcPwm::_pwmCount = 0;
volatile int8_t RcPwm::_counter = 0;
RcPwm::Pwm RcPwm::_pwms[MAX_PWM_COUNT];

void RcPwm::isr()
{
    if (_counter < 0)
    {
        TCNT1 = 0; // channel set to -1 indicated that refresh interval completed so reset the timer
    }
    else
    {
        if (_counter < RcPwm::_pwmCount && _pwms[_counter].pin.isActive)
        {
            digitalWrite(_pwms[_counter].pin.nbr, LOW); // pulse this channel low if activated
        }
    }

    ++_counter; // increment to the next channel

    if (_counter < _pwmCount && _counter < MAX_PWM_COUNT)
    {
        OCR1A = TCNT1 + _pwms[_counter].ticks;

        if (_pwms[_counter].pin.isActive) // check if activated
        {
            digitalWrite(_pwms[_counter].pin.nbr, HIGH); // its an active channel so pulse it high
        }
    }
    else
    {
        // finished all channels so wait for the refresh period to expire before starting over
        if (static_cast<unsigned>(TCNT1) + 4 < US_TO_TICKS(REFRESH_INTERVAL)) // allow a few ticks to ensure the next OCR1A not missed
        {
            OCR1A = static_cast<unsigned>(US_TO_TICKS(REFRESH_INTERVAL));
        }
        else
        {
            OCR1A = TCNT1 + 4; // at least REFRESH_INTERVAL has elapsed
        }

        _counter = -1; // this will get incremented at the end of the refresh period to start again at the first channel
    }
}

SIGNAL (TIMER1_COMPA_vect)
{
    RcPwm::isr();
}

static void initISR()
{
    TCCR1A = 0; // normal counting mode
    TCCR1B = _BV(CS11); // set prescaler of 8
    TCNT1 = 0; // clear the timer count

    // here if not ATmega8 or ATmega128
    TIFR1 |= _BV(OCF1A); // clear any pending interrupts;
    TIMSK1 |= _BV(OCIE1A); // enable the output compare interrupt
}

boolean RcPwm::isTimerActive()
{
    // returns true if any pwm is active on this timer
    for (uint8_t channel = 0; channel < MAX_PWM_COUNT; channel++)
    {
        if (_pwms[channel].pin.isActive)
            return true;
    }
    return false;
}

RcPwm::RcPwm()
{
    if (_pwmCount < MAX_PWM_COUNT)
    {
        _index = _pwmCount++; // assign a pwm index to this instance
        _pwms[_index].ticks = US_TO_TICKS(DEFAULT_PULSE_WIDTH); // store default values  - 12 Aug 2009
    }
    else
        _index = INVALID_SERVO; // too many _pwms
}

uint8_t RcPwm::attach(int pin)
{
    if (_index < MAX_PWM_COUNT)
    {
        pinMode(pin, OUTPUT); // set pwm pin to output
        _pwms[_index].pin.nbr = pin;

        // initialize the timer if it has not already been initialized
        if (!isTimerActive())
            initISR();
        _pwms[_index].pin.isActive = true; // this must be set after the check for isTimerActive
    }
    return _index;
}

void RcPwm::detach()
{
    _pwms[_index].pin.isActive = false;
}

void RcPwm::writeMicroseconds(uint16_t value)
{
    // calculate and store the values for the given channel
    byte channel = _index;
    if ((channel < MAX_PWM_COUNT)) // ensure channel is valid
    {
        if (value < MIN_PULSE_WIDTH) // ensure pulse width is valid
            value = MIN_PULSE_WIDTH;
        else if (value > MAX_PULSE_WIDTH)
            value = MAX_PULSE_WIDTH;

        value = value - TRIM_DURATION;
        value = US_TO_TICKS(value); // convert to ticks after compensating for interrupt overhead - 12 Aug 2009

        uint8_t oldSREG = SREG;
        cli();
        _pwms[channel].ticks = value;
        SREG = oldSREG;
    }
}

uint16_t RcPwm::readMicroseconds() const
{
    unsigned int pulsewidth;
    if (_index != INVALID_SERVO)
        pulsewidth = TICKS_TO_US(_pwms[_index].ticks) + TRIM_DURATION; // 12 aug 2009
    else
        pulsewidth = 0;

    return pulsewidth;
}

bool RcPwm::attached() const
{
    return _pwms[_index].pin.isActive;
}
