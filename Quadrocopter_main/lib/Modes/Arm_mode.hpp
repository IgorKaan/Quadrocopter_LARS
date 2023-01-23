#pragma once

#include <Arduino.h>
#include "Mode.hpp"

class Arm_mode : public Mode
{
private:
    bool blocked;    // Параметр заблокирован (значение не может стать true)
    float max_angle; // Максимальный угол, при котором возможен Arm, 180 - любое положение

public:
    Arm_mode(uint16_t *throttle_ch);
    void set_activated(bool activated);
    void set_value(uint16_t ch_val, uint16_t throttle_val,
                   float roll, float pitch);
    void set_blocked(bool blocked);
    bool get_blocked();
    bool set_max_angle(float max_angle);
    float get_max_angle();
};