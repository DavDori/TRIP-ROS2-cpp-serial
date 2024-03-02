#ifndef DIFFERENTIAL_DRIVE_MODEL_H
#define DIFFERENTIAL_DRIVE_MODEL_H

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <stdexcept> // exception handling

class DifferentialDriveModel
{
private:
    double wheel_radius_;
    double wheel_distance_;
    double linear_speed_;
    double angular_speed_;
    double gearbox_;

public:
    DifferentialDriveModel(double wheel_radius, double wheel_distance, double gearbox);

    void setUnicycleSpeed(double lin_speed, double ang_speed);
    void setDifferentialSpeed(double left_wheel_speed, double right_wheel_speed);
    double getLeftWheelRotationalSpeed() const;
    double getRightWheelRotationalSpeed() const;
    double getLeftMotorRotationalSpeed() const;
    double getRightMotorRotationalSpeed() const;
    double getLinearSpeed() const;
    double getAngularSpeed() const;
};


#endif
