#include "PID.hpp"
#include "config.hpp"
#include <cmath>
//#include <Arduino.h>
// PIDImpl::PIDImpl( double dt, double max, double min, double Kp, double Kd, double Ki )
// {
//     pimpl = new PIDImpl(dt, max ,min ,Kp ,Kd , Ki);
// }
// double PIDImpl::calculate( double setpoint, double pv )
// {
//     return pimpl->calculate(setpoint,pv);
// }
// PIDImpl::~PIDImpl() 
// {
//     delete pimpl;
// }

/**
 * Implementation
 */
PIDImpl::PIDImpl(float dt, float max, float min, float imax, float imin, float Kp, float Kd, float Ki) :
    _dt(dt),
    _max(max),
    _min(min),
    _imax(imax),
    _imin(imin),
    _Kp(Kp),
    _Kd(Kd),
    _Ki(Ki),
    _pre_error(0),
    _integral(0)
{
}

float PIDImpl::calculate(float setpoint, float pv)
{
    
    // Calculate error
    float error = setpoint - pv;

    // Proportional term
    float Pout = _Kp * error;

    // Integral term
    _integral += error * _dt;
    float Iout = _Ki * _integral;
    if (Iout > _imax) {
        Iout = _imax;
    }
    if (Iout < _imin) {
        Iout = _imin;
    }

    // Derivative term
    float derivative = (error - _pre_error) / _dt;
    float Dout = _Kd * derivative;
    //Serial.println(Dout);
    // Calculate total output
    double output = Pout + Iout + Dout;
    //float output = Pout;

    // Restrict to max/min
    if( output > _max )
        output = _max;
    else if( output < _min )
        output = _min;

    // Save error to previous error
    _pre_error = error;

    return output;
}

PIDImpl::~PIDImpl() = default;

