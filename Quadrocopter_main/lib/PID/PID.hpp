#pragma once

class PIDImpl
{
    public:
        PIDImpl(float dt, float max, float min, float imax, float imin, float Kp, float Kd, float Ki);
        ~PIDImpl();
        float calculate(float setpoint, float pv);

    private:
        float _dt;
        float _max;
        float _min;
        float _imax;
        float _imin;
        float _Kp;
        float _Kd;
        float _Ki;
        float _pre_error;
        float _integral;
};


