#include "motor.h"

Motor::Motor(int id, std::shared_ptr<SerialRobotInterface> Device)
{
    this->id = id;
    this->Device = Device;
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
    Device->setMotorSpeed(id, rpm);
}
