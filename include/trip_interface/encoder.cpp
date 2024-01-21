#include "encoder.h"


Encoder::Encoder(int id, double ppr)
{
    this->pulse_per_revolution = ppr;
    this->id = id;    
    reference_pulse_count = 0.0;
}

void Encoder::setReferencePulseCount(double pulse_count)
{
    reference_pulse_count = pulse_count;
}

void Encoder::setVelocity(double velocity_rpm)
{
    this->velocity_rpm = velocity_rpm;
}

void Encoder::setPulseCount(double count)
{
    this->pulse_count = count;
}

double Encoder::getPulseCount() const
{
    return pulse_count - reference_pulse_count;
}

double Encoder::getRevolutions() const
{
    return getPulseCount() / pulse_per_revolution;
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
    return velocity_rpm * M_PI / 30.0;
}

double Encoder::getSpeedRPM() const
{
    return velocity_rpm;
}
