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
    std::istringstream input_stream(message);
    std::string token;
    
    for(size_t i = 0; i < id_; i++)
    {
        std::getline(input_stream, token, SEPARATOR_STRING);
    }
    double velocity_rpm = extractRPM(token);
    setVelocityRPM(velocity_rpm);
}

double Encoder::extractRPM(const std::string& message)
{
    // a parsed string should look like: RPM: V10.0T12.5
    size_t pos_msg = message.find(ID_RPM_STRING);
    size_t pos_separator = message.find(ID_TIME_STRING);

    if(std::string::npos == pos_msg || std::string::npos == pos_separator)
    {
        throw std::runtime_error("Error: expected an ID in encoder velocity message");
    }
    // the rpm velocity value is after the string "RPM: " (in this case 5 characters)
    size_t length_rpm = pos_separator - pos_msg - RPM_MESSAGE_OFFSET;
    std::string rpm_substring = message.substr(pos_msg + RPM_MESSAGE_OFFSET, length_rpm);

    double velocity_rpm;
    std::istringstream(rpm_substring) >> velocity_rpm;
    return velocity_rpm;
}