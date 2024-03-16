#ifndef ENCODER_H
#define ENCODER_H

#include <iostream>
#include <memory>
#include <string>
#include <cmath>
#include <stdexcept> // exception handling
#include "serial_interface.h"

#define SEPARATOR_CHAR ','
#define ID_RPM_CHAR_START 'V'
#define ID_RPM_CHAR_END 'T' //rpm as last value
#define ID_PULSE_CHAR_START 'P'
#define ID_PULSE_CHAR_END 'V'

class Encoder
{
private:
    std::shared_ptr<SerialInterface> Device_;
    double pulse_per_revolution_;
    long pulse_count_;
    long reference_pulse_count_;
    double velocity_rpm_;
    int id_;

    bool isEncoderMessage(const std::string& message) const;
    void parseEncodersMessage(const std::string& message); 
    double extractRPM(const std::string& token) const;
    long extractPulseCount(const std::string& token) const;
    std::string extractDataString(const std::string& token, char start, char end) const;
    void setVelocityRPM(double velocity_rpm);
    void setPulseCount(long count);

public:
    Encoder(int id, double ppr, std::shared_ptr<SerialInterface> Device);
    
    void setReferencePulseCount(long pulse_count);
    
    void readMeasurement();
    double getSpeedRPM() const;
    double getSpeedRADpS() const;
    double getSpeedDEGpS() const;

    long getPulseCount() const;
    double getRevolutions() const;
    double getRadiants() const;
    double getDegrees() const;

    int getID() const;
    double getPPR() const;

};

#endif