#include <Arduino.h>
#include "Mode.hpp"

Mode::Mode()
{
    activated = false;
    value = false;
    set_range(1300, 1700);
}

void Mode::set_activated(bool activated)
{
    this->activated = activated;
    if (!activated)
    {
        this->activated = false;
        value = false;
        set_range(1300, 1700);
    }
}

bool Mode::get_activated()
{
    return activated;
}

void Mode::set_value(uint16_t ch_val)
{
    if (activated)
    {
        if (ch_val >= range_min &&
            ch_val <= range_max)
        {
            if (!value)
            {
                value = true;
            }
        }
        else if (value)
        {
            value = false;
        }
    }
}

bool Mode::get_value()
{
    return value;
}

bool Mode::set_range(uint8_t min, uint8_t max)
{
    if (max < min || min < 900 || max > 2100)
    {
        return false;
    }
    else
    {
        range_min = min;
        range_max = max;
        return true;
    }
}

uint8_t Mode::get_range_min()
{
    return range_min;
}

uint8_t Mode::get_range_max()
{
    return range_max;
}
