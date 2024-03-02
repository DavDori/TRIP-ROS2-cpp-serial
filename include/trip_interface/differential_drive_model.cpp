#include "differential_drive_model.h"

DifferentialDriveModel::DifferentialDriveModel(double wheel_radius, double wheel_distance, double gearbox)
{
    wheel_distance_ = wheel_distance;
    wheel_radius_ = wheel_radius;
    gearbox_ = gearbox;
    setUnicycleSpeed(0.0,0.0);
}

void DifferentialDriveModel::setUnicycleSpeed(double lin_speed, double ang_speed)
{
    linear_speed_ = lin_speed;
    angular_speed_ = ang_speed;
}

void DifferentialDriveModel::setDifferentialSpeed(double left_wheel_speed, double right_wheel_speed)
{
    linear_speed_ = 0.5*wheel_radius_*(left_wheel_speed + right_wheel_speed);
    angular_speed_ = wheel_radius_*(-left_wheel_speed + right_wheel_speed) / wheel_distance_;
}

double DifferentialDriveModel::getLeftWheelRotationalSpeed() const
{
    double wheel_speed = linear_speed_ - angular_speed_ * (0.5*wheel_distance_);
    return wheel_speed / wheel_radius_;
}
double DifferentialDriveModel::getRightWheelRotationalSpeed() const
{
    double wheel_speed = linear_speed_ + angular_speed_ * (0.5*wheel_distance_);
    return wheel_speed / wheel_radius_;
}

double DifferentialDriveModel::getLeftMotorRotationalSpeed() const
{
    return getLeftWheelRotationalSpeed() * gearbox_;
}
double DifferentialDriveModel::getRightMotorRotationalSpeed() const
{
    return getRightWheelRotationalSpeed() * gearbox_;
}

double DifferentialDriveModel::getLinearSpeed() const
{
    return linear_speed_;
}
double DifferentialDriveModel::getAngularSpeed() const
{
    return angular_speed_;
}
