#include "motor.h"

Motor::Motor(int id, std::shared_ptr<SerialRobotInterface> Device)
{
    id_ = id;
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
    Device_->setMotorSpeed(id_, rpm);
}
