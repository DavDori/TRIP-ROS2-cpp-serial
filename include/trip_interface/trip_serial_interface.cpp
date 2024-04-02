#include "trip_serial_interface.h"


std::vector<double> extractEncodersVelocity(const std::string& msg)
{
    std::vector<double> vel_set;
    // expected message with shape E0V0.00,E1V0.00,...,EXVXX,
    std::istringstream input_stream(msg);
    for(std::string token;
        std::getline(input_stream, token, SEPARATOR_CHAR);)
    {
        if(isEncoderMsg(token) == false || 
            isVelocityMsg(token) == false) 
        {
            continue;
        }
        double vel = extractVelRPM(token);
        vel_set.push_back(vel); 
    }
    return vel_set;
}

std::vector<long> extractEncodersPulses(const std::string& msg)
{
    std::vector<long> pulses_set;
    // expected message with shape E0P0.00,E1P10.00,...,EXPXX,
    std::istringstream input_stream(msg);
    for(std::string token;
        std::getline(input_stream, token, SEPARATOR_CHAR);)
    {
        if(isEncoderMsg(token) == false || 
            isPulseMsg(token) == false) 
        {
            continue;
        }
        double pulse = extractPulseCount(token);
        pulses_set.push_back(pulse); 
    }
    return pulses_set;
}

bool isEncoderMsg(const std::string& msg) 
{
    if(0 == msg.size())
    {
        throw std::runtime_error("Encoder message has size 0");               
    }
    char id = msg.at(0);    
    return ENC_CHAR_ID == id; // message information are about encoder
}


bool isVelocityMsg(const std::string& msg)
{
    if(msg.size() < 3)
    {
        return false;              
    }
    char id = msg.at(2);    
    return ID_RPM_CHAR_START == id; // message information are about encoder
}

bool isPulseMsg(const std::string& msg)
{
    if(msg.size() < 3)
    {
        return false;              
    }
    char id = msg.at(2);
    return ID_PULSE_CHAR_START == id; // message information are about encoder velocity
}

double extractVelRPM(const std::string& token)
{
    // a parsed string should look like: E0P100V0.00
    std::string rpm_data_string = extractDataString(token, ID_RPM_CHAR_START, ID_RPM_CHAR_END);
    double velocity_rpm;
    std::istringstream(rpm_data_string) >> velocity_rpm;
    return velocity_rpm;
}

long extractPulseCount(const std::string& token)
{
    // a parsed string should look like: E0P100V0.00
    std::string pulse_data_string = extractDataString(token, ID_PULSE_CHAR_START, ID_PULSE_CHAR_END);

    long pulse_count;
    std::istringstream(pulse_data_string) >> pulse_count;
    return pulse_count;
}

std::string extractDataString(const std::string& token, char start, char end)
{
    size_t pos_start = token.find(start);
    size_t pos_end;

    if('*' == end) //read untill the end of token
        pos_end = token.length();
    else
        pos_end = token.find(end);

    if(std::string::npos == pos_start || std::string::npos == pos_end)
    {
        throw std::runtime_error("Error: cannot isolate the data within the two characters provided");
    }
    // the value is after a character such as "V" (in this case 1 characters)
    size_t length_data = pos_end - pos_start - 1;
    std::string data_substring = token.substr(pos_start + 1, length_data);
    return data_substring;
}

std::string encodeVelocitySetpoint(double vel_rpm, int id)
{
    std::ostringstream oss;
    oss << "CSET," << id << "," << std::fixed << std::setprecision(2) << vel_rpm << std::endl;
    return oss.str();
}

// value within [-1,1] works in openloop
std::string encodeVelocityAbsolute(double vel, int id)
{
    if(abs(vel) > 1.0)
        throw std::invalid_argument("Error: absolute velocity command must be within 1 and -1.\n");
    std::ostringstream oss;
    oss << "MSET," << id << "," << std::fixed << std::setprecision(3) << vel << std::endl;
    return oss.str();
}

double mapRange(double val1, double max1, double max2)
{
    double val2 = 0.0;
    if(max1 != 0.0)
        val2 = val1 * max2 / max1;
    return val2;
}


std::string replaceString(std::string source, std::string find, std::string replacement) {
    std::string::size_type pos = 0;
    while ((pos = source.find(find, pos)) != std::string::npos) {
        source.replace(pos, find.size(), replacement);
        pos++;
    }
    return source;
}

