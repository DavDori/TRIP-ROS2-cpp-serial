#ifndef MOTOR_H
#define MOTOR_H

#include <iostream>
#include <memory>
#include <string>
#include <cmath>
#include "serial_interface.h"

#define RADPS_TO_RPM (30.0 / M_PI)
#define DEGPS_TO_RPM (1.0/5.0)
#define CTRL_MOTOR_MAX_VALUE 1000
#define MAX_RPM_DEFAULT 10

double saturate(double val, double max_val);

class Motor
{
private:
    std::shared_ptr<SerialInterface> Device_;
    double rpm_;
    double max_rpm_;
    int id_;

public:
    Motor(int id, std::shared_ptr<SerialInterface> Device);
    Motor(int id, double max_rpm, std::shared_ptr<SerialInterface> Device);
    
    void moveRPM(double rpm);
    void moveDEGPS(double deg_per_sec);
    void moveRADPS(double rad_per_sec);
    void move(double vel_normalized); // value within [-1,1] works in openloop
    double getRPM() const {return rpm_;}
    double getMaxRPM() const {return max_rpm_;}
    int getID() const {return id_;}
};

#endif