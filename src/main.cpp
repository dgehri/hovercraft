#include "Motor.h"
#include "Gyro.h"
#include "RcChannel.h"
#include "Timer.h"
#include <Arduino.h>
#include <estd/algorithm.h>
#include <EEPROM.h>

// #define OUTPUT_READABLE_ACCELGYRO

constexpr uint8_t PIN_RX_DIR = 2;
constexpr uint8_t PIN_RX_THRUST = 3;
constexpr uint8_t PIN_RX_HOVER = 4;
constexpr uint8_t PIN_TX_HOVER = 11;
constexpr uint8_t PIN_TX_LEFT_FAN = 7;
constexpr uint8_t PIN_TX_RIGHT_FAN = 8;
constexpr uint8_t EEPROM_HOVER_VAL = 0;

// all in microseconds
constexpr int FAIL_SAFE_TIMEOUT_US = 1000;
constexpr uint32_t INIT_TIME_US = 3000000;
constexpr int DEAD_ZONE = 10;
constexpr int DIR_CENTER = 1500;
constexpr int HOVER_MID_VALUE = 1500;
constexpr int STOP_VAL = 990;
constexpr int START_VAL = 1020;
constexpr int MIN_VAL = 980;
constexpr int MAX_VAL = 2020;
constexpr int TUNE_VAL = 1800;
constexpr int INIT_VAL = 900;
constexpr int ZERO_HOVER_FAN = 980;
constexpr int ZERO_LEFT_FAN = 1470;
constexpr int ZERO_RIGHT_FAN = 1480;
constexpr int HOVER_DEFAULT_VAL = 1100;
constexpr int HOVER_FAILSAFE_VALUE = 1030;
constexpr int COUNT_PER_MICROS = 2;

uint32_t init_time = 0;
bool fail_safe = false;
bool init_done = false;
int int_count = 0;
int hover_val = HOVER_DEFAULT_VAL;
const Range range = { MIN_VAL, MAX_VAL, START_VAL, STOP_VAL };
Motor leftMotor(PIN_TX_LEFT_FAN, range);
Motor rightMotor(PIN_TX_RIGHT_FAN, range);
Motor hoverMotor(PIN_TX_HOVER, range, false);
RcChannel thrustChannel(PIN_RX_THRUST, DIR_CENTER);
RcChannel dirChannel(PIN_RX_DIR, DIR_CENTER);
RcChannel hoverChannel(PIN_RX_HOVER, MIN_VAL);
Gyro gyro;

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
    int thrust_us;
    int dir_us;
    int hover_us;
    int dir_damping_factor; // 0 .. 32 (full .. no steering)
};

void eeprom_write(int address, int value)
{
    byte low, high;
    low = value & 0xFF;
    high = (value >> 8) & 0xFF;
    EEPROM.write(address, low);
    EEPROM.write(address + 1, high);
}

int eeprom_read_int(int address)
{
    byte low, high;
    low = EEPROM.read(address);
    high = EEPROM.read(address + 1);
    return low + ((high << 8) & 0xFF00);
} 

void setup()
{
    init_done = false;
    init_time = micros();

    Serial.begin(9600);
    Serial.println("Timo's HoverCraft");

    Serial.println("\nConfiguring...");
    Serial.println("- Gyro");
    gyro.setup();

    Serial.println("- Left Motor");
    leftMotor.setup();

    Serial.println("- Right Motor");
    rightMotor.setup();

    Serial.println("- Hover Motor");
    hoverMotor.setup();

    Serial.println("- Thrust Channel");
    thrustChannel.setup();

    Serial.println("- Steering Channel");
    dirChannel.setup();

    Serial.println("- Hover Channel");
    hoverChannel.setup();

    Serial.println("- Timer");
    Timer::instance().setup();
}

ISR (PCINT2_vect) // handle pin change interrupt for D0 to D7 here
{    
    auto pind = PIND;
    auto cnt = Timer::instance().get_count();

    if (thrustChannel.rx(pind, cnt))
    {
        // force rising edge of DIR 
        pind |= bit(PIN_RX_DIR);
    }
    
    dirChannel.rx(pind, cnt);
    hoverChannel.rx(pind, cnt);
}

RxData read_rc_inputs()
{
    RxData rx;
    rx.thrust_us = thrustChannel.pulse_length() / COUNT_PER_MICROS;
    rx.dir_us = dirChannel.pulse_length() / COUNT_PER_MICROS;
    rx.hover_us = hoverChannel.pulse_length() / COUNT_PER_MICROS;

    if (rx.dir_us < DIR_CENTER - DEAD_ZONE)
    {
        rx.dir_damping_factor = (max(rx.dir_us, MIN_VAL) - MIN_VAL) / 16;
    }
    else if (rx.dir_us > DIR_CENTER + DEAD_ZONE)
    {
        rx.dir_damping_factor = (MAX_VAL - estd::min(rx.dir_us, MAX_VAL)) / 16;
    }
    else
    {
        rx.dir_damping_factor = (DIR_CENTER - MIN_VAL) / 16;
    }

    return rx;
}

void handle_hover_state(const RxData& rxData, int gyro_z)
{
    hoverMotor.set(hover_val);

    auto dir_bias = (gyro_z * ((rxData.dir_damping_factor / 2) + 16)) / 32;
    auto dir_us = rxData.dir_us + dir_bias;
    auto dir_delta_us = dir_us - DIR_CENTER;
    auto right_us = rxData.thrust_us - dir_delta_us;
    auto left_us = rxData.thrust_us + dir_delta_us;

    leftMotor.set(ZERO_LEFT_FAN + (left_us - DIR_CENTER) / 2);
    rightMotor.set(ZERO_RIGHT_FAN + (right_us - DIR_CENTER) / 2);
}

void update_state_machine(const RxData& rxData, int gyro_z)
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
                int h = eeprom_read_int(EEPROM_HOVER_VAL);
                if (h > MIN_VAL and h < MAX_VAL)
                {
                    hover_val = h;
                }
            }
            rightMotor.set(DIR_CENTER, false);
            leftMotor.set(DIR_CENTER, false);
            hoverMotor.set(INIT_VAL, false);
            if (micros() - init_time > INIT_TIME_US)
            {
                state = State::Idle;
            }
            break;

        case State::Idle:
            hoverMotor.set(ZERO_HOVER_FAN);
            leftMotor.set(ZERO_LEFT_FAN);
            rightMotor.set(ZERO_RIGHT_FAN);
            if (toggle_hover && !fail_safe)
            {
                if (!tune)
                {
                    state = State::Calibration;
                }
                else 
                {
                    state = State::Tune;
                }
            }
            break;

        case State::Calibration:
            gyro.calibrate();
            state = State::Hover;
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
            hoverMotor.set(HOVER_FAILSAFE_VALUE);
            leftMotor.set(ZERO_LEFT_FAN);
            rightMotor.set(ZERO_RIGHT_FAN);
            if (!fail_safe)
            {
                state = State::Hover;
            }
            break;

        case State::Tune:
            if (fail_safe || toggle_hover || !tune)
            {
                state = State::Idle;
                if (hover_val > MIN_VAL && hover_val < MAX_VAL)
                {
                    eeprom_write(EEPROM_HOVER_VAL, hover_val);
                }
            }
            else
            {
                hover_val = MIN_VAL + abs(rxData.dir_us - DIR_CENTER);
                hoverMotor.set(hover_val);
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

void serial_out(const RxData& rxData, int gyro_z)
{
    static int k = 0;

    // avoid blocking
    if (Serial.availableForWrite() < 32)
        return;

    switch (k++)
    {
        case 0:
            serial_print(" rxData.thr: ", rxData.thrust_us);
            break;
        case 1:
            serial_print(" rxData.dir: ", rxData.dir_us);
            break;
        case 2:
            serial_print(" rxData.hover: ", rxData.hover_us);
            break;
        case 3:
            serial_print(" tx_r: ", rightMotor.value());
            break;
        case 4:
            serial_print(" tx_l: ", leftMotor.value());
            break;
        case 5:
            serial_print(" tx_hm: ", hoverMotor.value());
            break;
        case 6:
            serial_print(" df: ", rxData.dir_damping_factor);
            break;
        case 7:
            serial_print(" gz: ", gyro_z);
            break;
        case 8:
            serial_print(" FS: ", fail_safe);
            break;
        case 9:
            serial_print(" ST: ", to_string(state));
        case 10:
            serial_print(" HV: ", hover_val);
        default:
            k = 0;
            Serial.println();
    }
}

void loop()
{
    auto&& rxData = read_rc_inputs();
    auto gyro_z = gyro.read();
    update_state_machine(rxData, gyro_z);
    serial_out(rxData, gyro_z);
}