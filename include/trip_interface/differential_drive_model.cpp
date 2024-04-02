#include "differential_drive_model.h"

DifferentialDriveModel::DifferentialDriveModel(double wheel_radius, double wheel_distance, double gearbox)
    : wheel_radius_(wheel_radius), wheel_distance_(wheel_distance), 
      linear_vel_(0.0), angular_vel_(0.0), gearbox_(gearbox)
{
    if(wheel_radius <= 0.0)
        throw std::invalid_argument("Error: wheel radius cannot be less than 0!");
    if(gearbox <= 0.0)
        throw std::invalid_argument("Error: gearbox ratio cannot be less than 0!");
    if(wheel_distance <= 0.0)
        throw std::invalid_argument("Error: distance between  wheels cannot be less than 0!");
}

DifferentialDriveModel::DifferentialDriveModel(double wheel_radius, double wheel_distance)
    : wheel_radius_(wheel_radius), wheel_distance_(wheel_distance), 
      linear_vel_(0.0), angular_vel_(0.0), gearbox_(1.0)
{
    if(wheel_radius <= 0.0)
        throw std::invalid_argument("Error: wheel radius cannot be less than 0!");
    if(wheel_distance <= 0.0)
        throw std::invalid_argument("Error: distance between  wheels cannot be less than 0!");
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
