#ifndef MOTOR_H
#define MOTOR_H

#include <iostream>
#include <memory>
#include <string>
#include <cmath>
#include "serial_robot_interface.h"

#define RADPS_TO_RPM (30.0 / M_PI)
#define DEGPS_TO_RPM (1.0/5.0)


class Motor
{
private:
    int id;
    std::shared_ptr<SerialRobotInterface> Device;
public:
    Motor(int id, std::shared_ptr<SerialRobotInterface> Device);
    
    void moveRPM(double rpm);
    void moveDEGPS(double deg_per_sec);
    void moveRADPS(double rad_per_sec);
};

#endif