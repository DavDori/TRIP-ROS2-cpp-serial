#ifndef ENCODER_H
#define ENCODER_H

#include <vector>
#include <iostream>
#include <memory>
#include <cmath>
#include <stdexcept> // exception handling


class Encoder
{
private:
    double ppr_;
    long reference_pulse_;
    long pulses_;
    int id_;
    double vel_rpm_;
public:
    Encoder(double ppr);
    
    void setReferencePulseCount(long pulse_count);
    void setPulseCount(long pulses);

    long getPulseCount() const;
    double getRevolutions() const;
    double getRadiants() const;
    double getDegrees() const;
    double getVelocityRPM()const;
    double getPPR() const;
};

#endif