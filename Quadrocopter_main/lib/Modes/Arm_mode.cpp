#include <Arduino.h>
#include "Arm_mode.hpp"

/*
 * ARM
 */
Arm_mode::Arm_mode(uint16_t *throttle_ch)
{
    blocked = false;
    max_angle = 20;
}

void Arm_mode::set_activated(bool activated)
{
    this->activated = activated;
    if (!activated)
    {
        value = false;
        blocked = false;
        set_range(1300, 1700);
    }
}

void Arm_mode::set_value(uint16_t ch_arm, uint16_t ch_throttle,
                         float roll, float pitch)
{
    if (activated)
    {
        if (ch_arm >= range_min &&
            ch_arm <= range_max)
        {
            if (!blocked && !value)
            {
                if (ch_throttle <= 1005 &&
                    fabs(roll) <= max_angle &&
                    fabs(pitch) <= max_angle)
                {
                    value = true;
                }
                else
                {
                    blocked = true;
                }
            }
        }
        else
        {
            value = false;
            blocked = false;
        }
    }
}

void Arm_mode::set_blocked(bool blocked)
{
    this->blocked = blocked;
}

bool Arm_mode::get_blocked()
{
    return blocked;
}

bool Arm_mode::set_max_angle(float max_angle)
{
    if (max_angle >= 0 && max_angle <= 180)
    {
        this->max_angle = max_angle;
        return true;
    }
    return false;
}

float Arm_mode::get_max_angle()
{
    return max_angle;
}