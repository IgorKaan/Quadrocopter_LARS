#pragma once

#include <Arduino.h>

class Mode
{
protected:
    bool activated; // Параметр активирован (присутствует в схеме дрона)
    bool value;     // Значение параметра
    uint8_t range_min;
    uint8_t range_max;

public:
    Mode();
    virtual void set_activated(bool activated);
    bool get_activated();
    virtual void set_value(uint16_t ch_val);
    bool get_value();
    bool set_range(uint8_t min, uint8_t max);
    uint8_t get_range_min();
    uint8_t get_range_max();
};