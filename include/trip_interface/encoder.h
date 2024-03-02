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
    double pulse_per_revolution_;
    double pulse_count_;
    double reference_pulse_count_;
    double velocity_rpm_;
    int id_;
public:
    Encoder(int id, double ppr);
    
    void setReferencePulseCount(double pulse_count);
    void setVelocityRPM(double velocity_rpm);
    void setPulseCount(double count);

    
    double getSpeedRPM() const;
    double getSpeedRADpS() const;

    double getPulseCount() const;
    double getRevolutions() const;
    double getRadiants() const;
    double getDegrees() const;

    int getID() const;
    double getPPR() const;

};

#endif