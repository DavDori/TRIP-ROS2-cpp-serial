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
    double linear_vel_;
    double angular_vel_;
    double gearbox_;

public:
    DifferentialDriveModel(double wheel_radius, double wheel_distance, double gearbox);
    DifferentialDriveModel(double wheel_radius, double wheel_distance);

    void setUnicycleVel(double lin_vel, double ang_vel);
    void setDifferentialVel(double left_wheel_vel, double right_wheel_vel);
    double getLeftWheelRotationalVel() const;
    double getRightWheelRotationalVel() const;
    double getLeftMotorRotationalVel() const;
    double getRightMotorRotationalVel() const;
    double getLinearVel() const;
    double getAngularVel() const;

    double getWheelRadius() const {return wheel_radius_;}
    double getWheelDistance() const {return wheel_distance_;}
    double getGearbox() const {return gearbox_;}
};


#endif
