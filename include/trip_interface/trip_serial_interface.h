#ifndef TRIP_SERIAL_INTERFACE_H
#define TRIP_SERIAL_INTERFACE_H

#include <iostream>
#include <memory>
#include <string>
#include <sstream>
#include <vector>
#include <iomanip>  // For formatting the double variable
#include <sstream>  // For converting int and double to string

#define SEPARATOR_CHAR ','
#define ID_RPM_CHAR_START 'V'
#define ID_RPM_CHAR_END '*' //rpm as last value
#define ID_PULSE_CHAR_START 'P'
#define ID_PULSE_CHAR_END '*'
#define ENC_CHAR_ID 'E'

std::vector<double> extractEncodersVelocity(const std::string& msg);
std::vector<long> extractEncodersPulses(const std::string& msg);

bool isEncoderMsg(const std::string& msg);
bool isVelocityMsg(const std::string& msg);
bool isPulseMsg(const std::string& msg);

double extractVelRPM(const std::string& token);
long extractPulseCount(const std::string& token);
std::string extractDataString(const std::string& token, char start, char end);

std::string encodeVelocitySetpoint(double vel_rpm, int id);
std::string encodeVelocityAbsolute(double vel, int id);

double mapRange(double val1, double max1, double max2);
double saturate(double val, double max_val);

#endif

