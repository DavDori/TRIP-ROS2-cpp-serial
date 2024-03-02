#include "encoder.h"


Encoder::Encoder(int id, double ppr)
{
    if(0.0 == ppr)
        throw std::invalid_argument("Error: parameter pulses per revolution for encoder cannot be 0.0!");
    pulse_per_revolution_ = ppr;
    id_ = id;    
    reference_pulse_count_ = 0.0;
    velocity_rpm_ = 0.0;
    pulse_count_ = 0.0;
}

void Encoder::setReferencePulseCount(double pulse_count)
{
    reference_pulse_count_ = pulse_count;
}

void Encoder::setVelocity(double velocity_rpm)
{
    velocity_rpm_ = velocity_rpm;
}

void Encoder::setPulseCount(double count)
{
    pulse_count_ = count;
}

double Encoder::getPulseCount() const
{
    return pulse_count_ - reference_pulse_count_;
}

double Encoder::getRevolutions() const
{
    return getPulseCount() / pulse_per_revolution_;
}

double Encoder::getRadiants() const
{
    return getRevolutions() * 2 * M_PI;
}

double Encoder::getDegrees() const
{
    return getRevolutions() * 360.0;
}

double Encoder::getSpeedRADpS() const
{
    return velocity_rpm_ * M_PI / 30.0;
}

double Encoder::getSpeedRPM() const
{
    return velocity_rpm_;
}
