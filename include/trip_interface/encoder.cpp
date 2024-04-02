#include "encoder.h"


Encoder::Encoder(double ppr)
{
    if(ppr <= 0.0)
        throw std::invalid_argument("Error: encoder pulse per revolution cannot be  less than 0!\n");
    ppr_ = ppr;
    
    reference_pulse_ = 0;
    pulses_ = 0;
}

void Encoder::setReferencePulseCount(long pulses)
{
    reference_pulse_ = pulses;
}

void Encoder::setPulseCount(long pulses)
{
    pulses_ = pulses;
}


long Encoder::getPulseCount() const
{
    return pulses_ - reference_pulse_;
}

double Encoder::getRevolutions() const
{
    return double(getPulseCount()) / ppr_;
}

double Encoder::getRadiants() const
{
    return getRevolutions() * 2 * M_PI;
}

double Encoder::getDegrees() const
{
    return getRevolutions() * 360.0;
}

double Encoder::getVelocityRPM() const
{
    return vel_rpm_;
}

double Encoder::getPPR() const
{
    return ppr_;
}
