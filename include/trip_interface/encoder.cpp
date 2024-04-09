#include "encoder.h"

Encoder::Encoder(int id, double ppr, std::shared_ptr<SerialInterface> Device)
{
    if(0.0 == ppr)
        throw std::invalid_argument("Error: parameter pulses per revolution for encoder cannot be 0.0!");
    pulse_per_revolution_ = ppr;
    id_ = id;    
    reference_pulse_count_ = 0;
    velocity_rpm_ = 0.0;
    pulse_count_ = 0;
    Device_ = Device;
}

void Encoder::setReferencePulseCount(long pulse_count)
{
    reference_pulse_count_ = pulse_count;
}

void Encoder::setVelocityRPM(double velocity_rpm)
{
    velocity_rpm_ = velocity_rpm;
}

void Encoder::setPulseCount(long count)
{
    pulse_count_ = count;
}

long Encoder::getPulseCount() const
{
    return pulse_count_ - reference_pulse_count_;
}

double Encoder::getRevolutions() const
{
    return double(getPulseCount()) / pulse_per_revolution_;
}

double Encoder::getRadiants() const
{
    return getRevolutions() * 2 * M_PI;
}

double Encoder::getDegrees() const
{
    return getRevolutions() * 360.0;
}

int Encoder::getID() const
{
    return id_;
}

double Encoder::getPPR() const
{
    return pulse_per_revolution_;
}

double Encoder::getSpeedRADpS() const
{
    return velocity_rpm_ * M_PI / 30.0;
}

double Encoder::getSpeedDEGpS() const
{
    return velocity_rpm_ * 6;
}

double Encoder::getSpeedRPM() const
{
    return velocity_rpm_;
}

void Encoder::readMeasurement()
{
    // extract encoder information using the last message received
    // this allows to call only once the encoder feedback for all encoders,
    // and reduces the load on serial transmission. Because of this, the
    // mesasge to trigger the encoder feedback has to be before this
    // function send("E\n")
    try
    {
        std::string msg_in = Device_->getLastMessage();
        parseMsg(msg_in);
    }
    catch(std::exception& e)
    {
        std::cout << e.what() << std::endl;
    }   
}

void Encoder::parseMsg(const std::string& msg)
{
    // expected message with shape E0V0.00,E1V0.00,
    // expected message with shape E0P0.00,E1P10.00,
    std::istringstream input_stream(msg);
    for(std::string token;
        std::getline(input_stream, token, SEPARATOR_CHAR);)
    {
        if(isEncoderMsg(token) == false) {continue;}
        if(hasSameMsgID(token) == false) {continue;}
        if(isVelocityMsg(token))
        {
            double velocity_rpm = extractRPM(token);
            setVelocityRPM(velocity_rpm);
        }
        else if(isPulseMsg(token))
        {
            long pulse_count  = extractPulseCount(token);
            setPulseCount(pulse_count);
        }
    }
}


bool Encoder::isEncoderMsg(const std::string& msg) const
{
    if(0 == msg.size())
    {
        throw std::runtime_error("Encoder message has size 0");               
    }
    char id = msg.at(0);    
    return ENC_CHAR_ID == id; // message information are about encoder
}


bool Encoder::isVelocityMsg(const std::string& msg) const
{
    if(msg.size() < 3)
    {
        return false;              
    }
    char id = msg.at(2);    
    return ID_RPM_CHAR_START == id; // message information are about encoder
}

bool Encoder::isPulseMsg(const std::string& msg) const
{
    if(msg.size() < 3)
    {
        return false;              
    }
    char id = msg.at(2);
    return ID_PULSE_CHAR_START == id; // message information are about encoder velocity
}

bool Encoder::hasSameMsgID(const std::string& msg) const
{
    if(msg.size() < 2)
    {
        return false;              
    }
    int id = msg.at(1) - '0';
    return id_ == id; // message information are about this encoder
}

double Encoder::extractRPM(const std::string& token) const
{
    // a parsed string should look like: E0P100V0.00
    std::string rpm_data_string = extractDataString(token, ID_RPM_CHAR_START, ID_RPM_CHAR_END);
    double velocity_rpm;
    std::istringstream(rpm_data_string) >> velocity_rpm;
    return velocity_rpm;
}

long Encoder::extractPulseCount(const std::string& token) const
{
    // a parsed string should look like: E0P100V0.00
    std::string pulse_data_string = extractDataString(token, ID_PULSE_CHAR_START, ID_PULSE_CHAR_END);

    long pulse_count;
    std::istringstream(pulse_data_string) >> pulse_count;
    return pulse_count;
}

std::string Encoder::extractDataString(const std::string& token, char start, char end) const
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
