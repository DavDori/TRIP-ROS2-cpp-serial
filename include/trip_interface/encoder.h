#ifndef ENCODER_H
#define ENCODER_H

#include <iostream>
#include <memory>
#include <string>
#include <cmath>
#include <stdexcept> // exception handling

class Encoder
{
private:
    double pulse_per_revolution;
    double pulse_count;
    double reference_pulse_count;
    double velocity_rpm;
    int id;
public:
    Encoder(int id, double ppr);
    
    void setReferencePulseCount(double pulse_count);
    void setVelocity(double velocity_rpm);
    void setPulseCount(double count);

    
    double getSpeedRPM() const;
    double getSpeedRADpS() const;

    double getPulseCount() const;
    double getRevolutions() const;
    double getRadiants() const;
    double getDegrees() const;
};

#endif