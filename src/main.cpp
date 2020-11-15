#include "Motor.h"
#include "Gyro.h"
#include "RcChannel.h"
#include "Timer.h"
#include "eeprom_util.h"
#include "LedGauge.h"
#include <Arduino.h>
#include <Adafruit_INA219.h>
#include <estd/algorithm.h>

constexpr uint8_t PIN_RX_DIR = 2;
constexpr uint8_t PIN_RX_THRUST = 3;
constexpr uint8_t PIN_RX_HOVER = 4;
constexpr uint8_t PIN_TX_HOVER = 11;
constexpr uint8_t PIN_TX_LEFT_FAN = 7;
constexpr uint8_t PIN_TX_RIGHT_FAN = 8;
constexpr uint8_t PIN_NEOPIXEL = 6;

// all in microseconds
constexpr int16_t FAIL_SAFE_TIMEOUT_US = 1000;
constexpr uint32_t INIT_TIME_US = 3000000;
constexpr int16_t DEAD_ZONE = 10;
constexpr int16_t DIR_CENTER = 1500;
constexpr int16_t HOVER_MID_VALUE = 1500;
constexpr int16_t STOP_VAL = 990;
constexpr int16_t START_VAL = 1020;
constexpr int16_t MIN_VAL = 980;
constexpr int16_t MAX_VAL = 2020;
constexpr int16_t TUNE_VAL = 1800;
constexpr int16_t INIT_VAL = 900;
constexpr int16_t ZERO_HOVER_FAN = 980;
constexpr int16_t ZERO_LEFT_FAN = 1470;
constexpr int16_t ZERO_RIGHT_FAN = 1477;
constexpr int16_t HOVER_DEFAULT_VAL = 1100;
constexpr int16_t HOVER_FAILSAFE_VALUE = 1030;

uint32_t init_time = 0;
bool fail_safe = false;
bool init_done = false;
volatile bool rx_done = false;
int16_t int_count = 0;
int16_t hover_val = HOVER_DEFAULT_VAL;
const Range range = {MIN_VAL, MAX_VAL};
const Range thrust_range = {1020, 1980};
Motor left_motor(PIN_TX_LEFT_FAN, thrust_range);
Motor right_motor(PIN_TX_RIGHT_FAN, thrust_range);
Motor hover_motor(PIN_TX_HOVER, range);
RcChannel thrust_channel_rx(PIN_RX_THRUST, DIR_CENTER);
RcChannel dir_channel_rx(PIN_RX_DIR, DIR_CENTER);
RcChannel hover_channel_rx(PIN_RX_HOVER, MIN_VAL);
Gyro gyro;
LedGauge gauge(PIN_NEOPIXEL);
Adafruit_INA219 ina(0x44);
bool is3s = false;

enum class State
{
    Init,
    Calibration,
    Idle,
    Hover,
    FailSafe,
    Tune
};

State state = State::Init;

struct RxData
{
    int16_t thrust_us;
    int16_t dir_us;
    int16_t hover_us;
};

void setup()
{
    init_done = false;
    init_time = micros();

    Serial.begin(9600);
    Serial.println("Timo's HoverCraft");

    Serial.println("\nConfiguring...");

    Serial.println("- Pixels");
    gauge.setup();

    Serial.println("- Gyro");
    gyro.setup();

    Serial.println("- Left Motor");
    left_motor.setup();

    Serial.println("- Right Motor");
    right_motor.setup();

    Serial.println("- Hover Motor");
    hover_motor.setup();

    Serial.println("- Thrust PWM");
    thrust_channel_rx.setup();

    Serial.println("- Steering PWM");
    dir_channel_rx.setup();

    Serial.println("- Hover PWM");
    hover_channel_rx.setup();

    Serial.println("- Timer");
    Timer::instance().setup();

    Serial.println("- Voltage measurement");
    ina.begin();
}

// pin change interrupt for receiving RC signals
ISR(PCINT2_vect)  // handle pin change interrupt for D0 to D7 here
{
    auto pind = PIND;
    auto cnt = Timer::instance().get_count();

    if (thrust_channel_rx.rx(pind, cnt))
    {
        // force rising edge of DIR
        pind |= bit(PIN_RX_DIR);
    }

    dir_channel_rx.rx(pind, cnt);
    rx_done = hover_channel_rx.rx(pind, cnt);
}

RxData read_rc_inputs()
{
    RxData rx;
    rx.thrust_us = thrust_channel_rx.pulse_length() / COUNT_PER_MICROS;
    rx.dir_us = dir_channel_rx.pulse_length() / COUNT_PER_MICROS;
    rx.hover_us = hover_channel_rx.pulse_length() / COUNT_PER_MICROS;
    return rx;
}

/**
 * Calculate gyro damping factor from steering input
 * 
 * @return int16_t 32 for fully applying gyro value; 0 for not applying
 */
int16_t calculate_damping_factor(const RxData& rxData)
{
    using estd::min;

    int16_t dir_damping_factor; // 0 .. 32 (full .. no steering)

    if (rxData.dir_us < DIR_CENTER - DEAD_ZONE)
    {
        dir_damping_factor = min((max(rxData.dir_us, MIN_VAL) - MIN_VAL) / 16, 32);
    }
    else if (rxData.dir_us > DIR_CENTER + DEAD_ZONE)
    {
        dir_damping_factor = min((MAX_VAL - min(rxData.dir_us, MAX_VAL)) / 16, 32);
    }
    else
    {
        // no steering => fully apply gyro
        dir_damping_factor = 32;
    }   

    return dir_damping_factor;
}

void handle_hover_state(const RxData& rxData, int16_t gyro_z)
{
    hover_motor.set(hover_val);

    // directional component from steering
    auto dir_steering = (rxData.dir_us - DIR_CENTER);

    // directional component from thrust
    auto dir_thrust = rxData.thrust_us;

    // directional component from gyro
    auto gyro_damping_factor = calculate_damping_factor(rxData);
    auto dir_gyro = (gyro_z * ((gyro_damping_factor / 2) + 16)) / 64;

    // calculate set-point value
    auto right_us = dir_thrust - (dir_steering + dir_gyro);
    auto left_us = dir_thrust + (dir_steering + dir_gyro);

    // compensate for battery type
    if (is3s)
    {
        right_us = ((right_us - DIR_CENTER) / 2) + DIR_CENTER;
        left_us = ((left_us - DIR_CENTER) / 2) + DIR_CENTER;
    }
    else
    {
        right_us = (((right_us - DIR_CENTER) * 3) / 4) + DIR_CENTER;
        left_us = (((left_us - DIR_CENTER) * 3) / 4) + DIR_CENTER;
    }
    
    // apply steering trim
    right_us += ZERO_LEFT_FAN - DIR_CENTER;
    left_us += ZERO_RIGHT_FAN - DIR_CENTER;

    static constexpr int16_t MAX_DELTA = 50;
    right_motor.setFiltered(right_us, ZERO_RIGHT_FAN, MAX_DELTA);
    left_motor.setFiltered(left_us, ZERO_LEFT_FAN, MAX_DELTA);
}

void update_state_machine(const RxData& rxData, int16_t gyro_z)
{
    fail_safe = rxData.dir_us > 2 * MAX_VAL;

    bool hover_rx_high = rxData.hover_us > HOVER_MID_VALUE;
    static bool hover_rx_was_high = hover_rx_high;
    bool toggle_hover = hover_rx_high != hover_rx_was_high;
    hover_rx_was_high = hover_rx_high;

    bool tune = rxData.thrust_us > TUNE_VAL;

    switch (state)
    {
    case State::Init:
        {
            // detect battery
            auto v = ina.getBusVoltage_V();
            if (v > 9.0f)
            {
                is3s = true;
            }

            int16_t h = eeprom_read_int(EEPROM_HOVER_VALUE_ADDR);
            if (h > MIN_VAL and h < MAX_VAL)
            {
                hover_val = h;
            }

            right_motor.set(DIR_CENTER, false);
            left_motor.set(DIR_CENTER, false);
            hover_motor.set(INIT_VAL, false);
            if (micros() - init_time > INIT_TIME_US)
            {
                state = State::Idle;
            }
        }
        break;

    case State::Idle:
        hover_motor.set(ZERO_HOVER_FAN);
        left_motor.set(ZERO_LEFT_FAN);
        right_motor.set(ZERO_RIGHT_FAN);
        if (toggle_hover && !fail_safe)
        {
            if (!tune)
            {
                state = State::Hover;
                // hover_motor.disable();
                // left_motor.disable();
                // right_motor.disable();
            }
            else
            {
                state = State::Calibration;
            }
        }
        break;

    case State::Calibration:
        gyro.calibrate();
        state = State::Tune;
        break;

    case State::Hover:
        if (fail_safe)
        {
            state = State::FailSafe;
        }
        else if (toggle_hover)
        {
            state = State::Idle;
        }
        else
        {
            handle_hover_state(rxData, gyro_z);
        }
        break;

    case State::FailSafe:
        hover_motor.set(HOVER_FAILSAFE_VALUE);
        left_motor.set(ZERO_LEFT_FAN);
        right_motor.set(ZERO_RIGHT_FAN);
        if (!fail_safe)
        {
            state = State::Hover;
        }
        break;

    case State::Tune:
        if (fail_safe || toggle_hover || !tune)
        {
            state = State::Idle;
            if (hover_val > HOVER_FAILSAFE_VALUE && hover_val < MAX_VAL)
            {
                eeprom_write(EEPROM_HOVER_VALUE_ADDR, hover_val);
            }
        }
        else
        {
            hover_val = MIN_VAL + abs(rxData.dir_us - DIR_CENTER);
            hover_motor.set(hover_val);
        }
        break;
    }
}

const char* to_string(State state)
{
    switch (state)
    {
    case State::Init: return "Init";
    case State::Calibration: return "Calibration";
    case State::Idle: return "Idle";
    case State::Hover: return "Hover";
    case State::Tune: return "Tune";
    case State::FailSafe: return "FailSafe";
    default: return "<invalid>";
    }
}

template <typename T>
void serial_print(const char* label, T value, char eol = '\t')
{
    Serial.print(label);
    Serial.print(value);
    Serial.print(eol);
}

float v = 0.0;
float v_comp = 0.0;

void serial_out(const RxData& rxData, int16_t gyro_z)
{
    static int16_t k = 0;

    // avoid blocking
    if (Serial.availableForWrite() < 32)
        return;

    switch (k++)
    {
    case 0: serial_print(" rxData.thr: ", rxData.thrust_us); break;
    case 1: serial_print(" rxData.dir: ", rxData.dir_us); break;
    case 2: serial_print(" rxData.hover: ", rxData.hover_us); break;
    case 3: serial_print(" tx_r: ", right_motor.value()); break;
    case 4: serial_print(" tx_l: ", left_motor.value()); break;
    case 5: serial_print(" tx_hm: ", hover_motor.value()); break;
    case 6: serial_print(" gz: ", gyro_z); break;
    case 7: serial_print(" FS: ", fail_safe); break;
    case 8: serial_print(" ST: ", to_string(state)); break;
    case 9: serial_print(" HV: ", hover_val); break;
    case 10: serial_print(" V: ", v); break;
    case 11: serial_print(" Vc: ", v_comp); break;
    default: k = 0; Serial.println(); break;
    }
}

void loop()
{
    static uint32_t last_run = 0;
    auto now = Timer::instance().get_count();

    if (rx_done || RcPwm::needsToRun())
    {
        rx_done = false;
        last_run = now;

        auto rx_data = read_rc_inputs();
        auto gyro_z = gyro.read();
        update_state_machine(rx_data, gyro_z);

        RcPwm::runNow();

        v = ina.getBusVoltage_V();

        serial_out(rx_data, gyro_z);
    }

    if (now - last_run < COUNT_PER_MICROS * 1000)
    {
        //  0.45 mV voltage drop per hover tx - ZERO_HOVER_FAN
        //  0.60 mV voltage drop per thrust tx
        auto dv_l = abs(left_motor.value() - ZERO_LEFT_FAN) * 0.6e-3f;
        auto dv_r = abs(right_motor.value() - ZERO_RIGHT_FAN) * 0.6e-3f;
        auto dv_h = (hover_motor.value() - ZERO_HOVER_FAN) * 0.45e-3f;
        v_comp = v + dv_l + dv_r + dv_h;
        if (is3s)
        {
            v_comp *= 2.0f / 3.0f;
        }

        switch (state)
        {
            case State::Init:
                gauge.rainbowCycle(5);
                break;

            case State::Tune:
                {
                    int bars = (hover_motor.value() - ZERO_HOVER_FAN) / 50;
                    gauge.showBars(bars);
                }
                break;

            case State::Hover:
                gauge.showVoltage(v_comp);
                break;

            default:
                gauge.showVoltage(v_comp);
                break;
        }
    }
}
