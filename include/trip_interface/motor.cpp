#include "motor.h"


// SETTERS -----------------------------------------------------

void Motor::setActualVelocity(double velocity, vel_unit_t unit)
{
    vel_rpm_ = convert_vel_into_rpm(velocity, unit);
}

void Motor::setDesiredVelocity(double velocity, vel_unit_t unit)
{
    if(isTorqueMode())
    {
        throw std::logic_error("Error: cannot control velocity when in Troque Control Mode.\n");
    }
    vel_sp_rpm_ = convert_vel_into_rpm(velocity, unit);
}


void Motor::setActualTorque(double tau)
{   
    double amp = tau * torque_constant_;
    setActualCurrent(amp);
}

void Motor::setDesiredTorque(double tau)
{   
    double amp = tau * torque_constant_;
    setDesiredCurrent(amp);
}

void Motor::setActualCurrent(double amp)
{
    current_ = amp;
}

void Motor::setDesiredCurrent(double amp)
{
    if(isVelocityMode())
    {
        throw std::logic_error("Error: cannot set desired torque/current when in Velocity Control Mode.\n");
    }
    current_sp_ = amp;
}

// GETTERS ---------------------------------------------

double Motor::getActualVelocity(vel_unit_t unit) const
{
    return convert_vel_from_rpm(vel_rpm_, unit);
}

double Motor::getDesiredVelocity(vel_unit_t unit) const
{
    double vel_rpm_sat = saturate(vel_sp_rpm_, max_rpm_);
    return convert_vel_from_rpm(vel_rpm_sat, unit);
}

double Motor::getActualTorque() const
{
    return current_ / torque_constant_;
}

double Motor::getDesiredTorque() const
{
    return current_sp_ / torque_constant_;
}

double Motor::getActualCurrent() const
{
    return current_;
}

double Motor::getDesiredCurrent() const
{
    return current_sp_;
}

double convert_vel_into_rpm(double value, vel_unit_t unit)
{
    double res = 0.0;
    switch (unit)
    {
    case RADPS:
        res = RADPS_TO_RPM * value;
        break;
    case RPM:
        res = value;
        break;
    case DEGPS:
        res = DEGPS_TO_RPM * value;
        break;
    default:
        throw std::invalid_argument("Error: a non standard measurement unit for velocity was selected for conversion into RPM!\n");
    }
    return res;
}

double convert_vel_from_rpm(double value, vel_unit_t unit)
{
    double res = 0.0;
    switch (unit)
    {
    case RADPS:
        res = value / RADPS_TO_RPM;
        break;
    case RPM:
        res = value;
        break;
    case DEGPS:
        res = value / DEGPS_TO_RPM;
        break;
    default:
        throw std::invalid_argument("Error: a non standard measurement unit for velocity was selected for conversion into RPM!\n");
    }
    return res;
}

double saturate(double val, double max_val)
{
    if(val > max_val)
        return max_val;
    else if(val < -max_val)
        return -max_val;
    else
        return val;
}