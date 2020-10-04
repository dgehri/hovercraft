#pragma once
#include <EEPROM.h>

constexpr int16_t EEPROM_HOVER_VALUE_ADDR = 0;
constexpr int16_t EEPROM_IS_INIT_ADDR = 2;
constexpr int16_t EEPROM_GYRO_BASELINE_ADDR = 4;


void eeprom_write(int16_t address, uint16_t value)
{
    byte low, high;
    low = value & 0xFF;
    high = (value >> 8) & 0xFF;
    EEPROM.write(address, low);
    EEPROM.write(address + 1, high);
}

uint16_t eeprom_read_int(int16_t address)
{
    byte low, high;
    low = EEPROM.read(address);
    high = EEPROM.read(address + 1);
    return low + ((high << 8) & 0xFF00);
}

