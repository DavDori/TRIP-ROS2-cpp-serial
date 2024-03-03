#ifndef ENCODER_H
#define ENCODER_H

#include <iostream>
#include <memory>
#include <string>
#include <cmath>
#include <stdexcept> // exception handling
#include "serial_interface.h"

#define ID_RPM_STRING 'V' // string that identifies an encoder measurement
#define ID_TIME_STRING 'T' // string that identifies a time value
#define RPM_MESSAGE_OFFSET 1 // offset before the rpm value of the encoder measurement
#define SEPARATOR_STRING ','

class Encoder
{
private:
    std::shared_ptr<SerialInterface> Device_;
    double pulse_per_revolution_;
    double pulse_count_;
    double reference_pulse_count_;
    double velocity_rpm_;
    int id_;

    bool isEncoderMessage(const std::string& message) const;
    void parseEncodersMessage(const std::string& message); 
    double extractRPM(const std::string& message);
    void setVelocityRPM(double velocity_rpm);
    void setPulseCount(double count);

public:
    Encoder(int id, double ppr, std::shared_ptr<SerialInterface> Device);
    
    void setReferencePulseCount(double pulse_count);
    
    void readMeasurement();
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