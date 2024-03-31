#include "differential_drive_model.h"

DifferentialDriveModel::DifferentialDriveModel(double wheel_radius, double wheel_distance, double gearbox)
{
    if(0.0 == wheel_radius)
        throw std::invalid_argument("Error: wheel radius cannot be 0!");
    if(0.0 == gearbox)
        throw std::invalid_argument("Error: gearbox ratio cannot be 0!");
    if(0.0 == wheel_distance)
        throw std::invalid_argument("Error: distance between  wheels cannot be 0!");
    wheel_distance_ = wheel_distance;
    wheel_radius_ = wheel_radius;
    gearbox_ = gearbox;
    setUnicycleVel(0.0,0.0);
}

DifferentialDriveModel::DifferentialDriveModel(double wheel_radius, double wheel_distance)
{
    DifferentialDriveModel(wheel_radius, wheel_distance, 1.0);
}

void DifferentialDriveModel::setUnicycleVel(double lin_vel, double ang_vel)
{
    linear_vel_ = lin_vel;
    angular_vel_ = ang_vel;
}

void DifferentialDriveModel::setDifferentialVel(double left_wheel_vel, double right_wheel_vel)
{
    linear_vel_ = 0.5*wheel_radius_*(left_wheel_vel + right_wheel_vel);
    angular_vel_ = wheel_radius_*(-left_wheel_vel + right_wheel_vel) / wheel_distance_;
}

double DifferentialDriveModel::getLeftWheelRotationalVel() const
{
    double wheel_vel = linear_vel_ - angular_vel_ * (0.5*wheel_distance_);
    return wheel_vel / wheel_radius_;
}
double DifferentialDriveModel::getRightWheelRotationalVel() const
{
    double wheel_vel = linear_vel_ + angular_vel_ * (0.5*wheel_distance_);
    return wheel_vel / wheel_radius_;
}

double DifferentialDriveModel::getLeftMotorRotationalVel() const
{
    return getLeftWheelRotationalVel() * gearbox_;
}
double DifferentialDriveModel::getRightMotorRotationalVel() const
{
    return getRightWheelRotationalVel() * gearbox_;
}

double DifferentialDriveModel::getLinearVel() const
{
    return linear_vel_;
}
double DifferentialDriveModel::getAngularVel() const
{
    return angular_vel_;
}
