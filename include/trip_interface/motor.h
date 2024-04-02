#ifndef MOTOR_H
#define MOTOR_H

#include <vector>
#include <iostream>
#include <memory>
#include <string>
#include <cmath>

#define RADPS_TO_RPM (30.0 / M_PI)
#define DEGPS_TO_RPM (1.0/6.0)

typedef enum ctrl_mode_t {VELOCITY, TORQUE} ctrl_mode_t;
typedef enum vel_unit_t {RADPS, RPM, DEGPS} vel_unit_t;

double convert_vel_into_rpm(double value, vel_unit_t unit);
double convert_vel_from_rpm(double value, vel_unit_t unit);
double saturate(double val, double max_val);


class Motor
{
private:
    ctrl_mode_t mode_;
    double torque_constant_;
    double max_rpm_;
    double current_;
    double current_sp_;
    double vel_rpm_;
    double vel_sp_rpm_;

public:
    Motor(ctrl_mode_t mode, double torque_constant, double max_rpm)
        : mode_(mode), torque_constant_(torque_constant), max_rpm_(max_rpm),
          current_(0.0), current_sp_(0.0), vel_rpm_(0.0), vel_sp_rpm_(0.0) {}
    Motor(double max_rpm)
        : mode_(VELOCITY), torque_constant_(0.001), max_rpm_(max_rpm),
          current_(0.0), current_sp_(0.0), vel_rpm_(0.0), vel_sp_rpm_(0.0) {}
    
    void setActualVelocity(double velocity, vel_unit_t unit);
    void setDesiredVelocity(double velocity, vel_unit_t unit);
    void setActualTorque(double tau);
    void setDesiredTorque(double tau);
    void setActualCurrent(double amp);
    void setDesiredCurrent(double amp);

    double getActualVelocity(vel_unit_t unit) const;
    double getDesiredVelocity(vel_unit_t unit) const;
    double getActualTorque() const;
    double getDesiredTorque() const;
    double getActualCurrent() const;
    double getDesiredCurrent() const;

    bool isVelocityMode() {return mode_ == VELOCITY;}
    bool isTorqueMode() {return mode_ == TORQUE;}
};


#endif