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
        if(isEncoderMessage(msg_in) == false)
        {
            throw std::runtime_error("In encoder received unexpected message prefix");
        }
        parseEncodersMessage(msg_in);
    }
    catch(std::exception& e)
    {
        std::cout << e.what() << std::endl;
    }   
}

bool Encoder::isEncoderMessage(const std::string& message) const
{
    if(0 == message.size())
    {
        throw std::runtime_error("Encoder message has size 0");               
    }
    char id = message.at(0);
    return 'E' == id; // message information are about encoder
}

void Encoder::parseEncodersMessage(const std::string& message)
{
    // expected message shape is E0P20V0.00T300,E1P1500V0.00T303,
    std::istringstream input_stream(message);
    std::string token;
    
    for(size_t i = 0; i <= id_; i++)
    {
        std::getline(input_stream, token, SEPARATOR_CHAR);
    }
    double velocity_rpm = extractRPM(token);
    long pulse_count  = extractPulseCount(token);
    setVelocityRPM(velocity_rpm);
    setPulseCount(pulse_count);
}

double Encoder::extractRPM(const std::string& token) const
{
    // a parsed string should look like: E0P100V0.00T300
    std::string rpm_data_string = extractDataString(token, ID_RPM_CHAR_START, ID_RPM_CHAR_END);

    double velocity_rpm;
    std::istringstream(rpm_data_string) >> velocity_rpm;
    return velocity_rpm;
}

long Encoder::extractPulseCount(const std::string& token) const
{
    // a parsed string should look like: E0P100V0.00T300
    std::string pulse_data_string = extractDataString(token, ID_PULSE_CHAR_START, ID_PULSE_CHAR_END);

    long pulse_count;
    std::istringstream(pulse_data_string) >> pulse_count;
    return pulse_count;
}

std::string Encoder::extractDataString(const std::string& token, char start, char end) const
{
    size_t pos_start = token.find(start);
    size_t pos_end = token.find(end);

    if(std::string::npos == pos_start || std::string::npos == pos_end)
    {
        throw std::runtime_error("Error: cannot isolate the data within the two characters provided");
    }
    // the value is after a character such as "V" (in this case 1 characters)
    size_t length_data = pos_end - pos_start - 1;
    std::string data_substring = token.substr(pos_start + 1, length_data);
    return data_substring;
}
