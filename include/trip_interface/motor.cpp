#include "motor.h"

double saturate(double val, double max_val)
{
    if(val > max_val)
        return max_val;
    else if(val < -max_val)
        return -max_val;
    else
        return val;
}

Motor::Motor(int id, std::shared_ptr<SerialInterface> Device)
{
    id_ = id;
    max_rpm_ = MAX_RPM_DEFAULT;
    Device_ = Device;
}

Motor::Motor(int id, double max_rpm, std::shared_ptr<SerialInterface> Device)
{
    id_ = id;
    max_rpm_ = max_rpm; 
    Device_ = Device;
}

void Motor::moveRADPS(double rad_per_sec)
{
    double rpm = RADPS_TO_RPM * rad_per_sec;
    moveRPM(rpm);
}
void Motor::moveDEGPS(double deg_per_sec)
{
    double rpm = DEGPS_TO_RPM * deg_per_sec;
    moveRPM(rpm);
}
void Motor::moveRPM(double rpm)
{
    double rpm_saturated = saturate(rpm, max_rpm_);
    std::ostringstream oss;
    oss << "CSET," << id_ << "," << std::fixed << std::setprecision(2) << rpm_saturated << std::endl;
    std::string command = oss.str();
    Device_->send(command);
}

// value within [-1,1] works in openloop
void Motor::move(double vel_normalized)
{
    double vel_saturated = saturate(vel_normalized, 1.0);
    std::ostringstream oss;
    oss << "MSET," << id_ << "," << std::fixed << std::setprecision(3) << vel_saturated << std::endl;
    std::string command = oss.str();
    Device_->send(command);
}
