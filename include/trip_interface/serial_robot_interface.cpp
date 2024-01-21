#include "serial_robot_interface.h"

SerialRobotInterface::SerialRobotInterface(const std::string &port, speed_t baud_rate, std::vector<std::shared_ptr<Encoder>> encoders)
{
    this->serial_handle_ = open( port.c_str(), O_RDWR| O_NOCTTY );
    this->encoders_ = encoders;
    this->baud_rate_ = baud_rate;
    if ( serial_handle_ == INVALID_HANDLE )
    {
        throw CANNOT_OPEN_PORT;
    }
    fcntl(serial_handle_, F_SETFL, O_APPEND | O_NONBLOCK & ~FNDELAY);
    initSerialPort();

}

void SerialRobotInterface::initSerialPort()
{
    if (!isConnected())
        throw DEVICE_NOT_CONNECTED;
    struct termios newtio;
    tcgetattr(serial_handle_, &newtio);

    // Set the Tx and Rx Baud Rate to BOUDRATE
    cfsetospeed(&newtio, (speed_t)baud_rate_);
    cfsetispeed(&newtio, (speed_t)baud_rate_);

    // Enable the Receiver and  Set local Mode
    newtio.c_iflag = IGNBRK;            /* Ignore Break Condition & no processing under input options*/
    newtio.c_lflag = 0;                 /* Select the RAW Input Mode through Local options*/
    newtio.c_oflag = 0;                 /* Select the RAW Output Mode through Local options*/
    newtio.c_cflag |= (CLOCAL | CREAD); /* Select the Local Mode & Enable Receiver through Control options*/

    // Make RAW Mode more explicit by turning Canonical Mode off, Echo off, Echo Erase off and Signals off*/
    newtio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // Disable Software Flow Control
    newtio.c_iflag &= ~(IXON | IXOFF | IXANY);

    // Set Data format to 8N1
    newtio.c_cflag &= ~CSIZE;  /* Mask the Character Size Bits through Control options*/
    newtio.c_cflag |= CS8;     /* Select Character Size to 8-Bits through Control options*/
    newtio.c_cflag &= ~PARENB; /* Select Parity Disable through Control options*/
    newtio.c_cflag &= ~PARODD; /* Select the Even Parity (Disabled) through Control options*/
    newtio.c_cflag &= ~CSTOPB; /*Set number of Stop Bits to 1*/

    // Timout Parameters. Set to 0 characters (VMIN) and 10 second (VTIME) timeout. This was done to prevent the read call from blocking indefinitely.*/
    newtio.c_cc[VMIN] = 0;
    newtio.c_cc[VTIME] = 100;

    /* Flush the Input buffer and set the attribute NOW without waiting for Data to Complete*/
    tcflush(serial_handle_, TCIFLUSH);
    tcsetattr(serial_handle_, TCSANOW, &newtio);
}

void SerialRobotInterface::disconnect() {
    if (isConnected())
        close(serial_handle_);

    serial_handle_ = INVALID_HANDLE;
}

bool SerialRobotInterface::isConnected() const {
    return serial_handle_ != INVALID_HANDLE;
}


void SerialRobotInterface::readEncodersMeasurements()
{
    std::string msg_out = "ENC"; 
    try
    {
        std::cout << "sending enc request: " << msg_out << std::endl;
        sendMessage(msg_out);
        std::string msg_in = readMessage();
        std::cout << "recived: " << msg_in << std::endl;
        elaborateMessage(msg_in);
    }
    catch(errors code)
    {
        printErrorCode(code);
    }   
}

void SerialRobotInterface::setMotorSpeed(int id, double velocity)
{
    std::ostringstream oss;
    oss << "CSET," << id << "," << std::fixed << std::setprecision(2) << velocity;
    std::string command = oss.str();
    sendMessage(command);
}

void SerialRobotInterface::sendMotorCmd(int id, double cmd_value)
{
    std::ostringstream oss;
    oss << "MSET," << id << "," << std::fixed << std::setprecision(2) << cmd_value;
    std::string command = oss.str();
    sendMessage(command);
}

void SerialRobotInterface::sendMessage(const std::string &message)
{
    if (!isConnected())
        throw DEVICE_NOT_CONNECTED;

    int count_sent = write(serial_handle_, message.c_str(), message.length());

    // Verify weather the Transmitting Data on UART was Successful or Not
    if (count_sent < 0)
        throw TRANSMISSION_FAILED;
    usleep(WAIT_AFTER_MESSAGE_TX_MS * 1000l);
    
}

std::string SerialRobotInterface::readMessage() const
{
    if(!isConnected())
        throw DEVICE_NOT_CONNECTED;
    char buf[BUFFER_SIZE + 1] = "";
    std::string message = "";
    int count_rvc;
    // while (count_rvc > 0 && count_rvc > BUFFER_SIZE) {
    //     count_rvc = read(serial_handle_, buf, BUFFER_SIZE);
    //     message.append(buf, count_rvc);
    // }
    while ((count_rvc = read(serial_handle_, buf, BUFFER_SIZE)) > 0) {
        message.append(buf, count_rvc);

        // No further data.
        if (count_rvc < BUFFER_SIZE)
            break;
    }
    if (count_rvc < 0) {
        if (errno == EAGAIN)
            throw SERIAL_IO;
        else
            throw NO_SERIAL_RECEIVE;
    }
    return message;
}

void SerialRobotInterface::elaborateMessage(const std::string &message)
{

    // Extract the first 3 characters
    std::string prefix = message.substr(0, 3);

    // The hash function converts the string into a integer number
    try
    {
        std::cout << message << std::endl;
        unsigned long hash_value = hash_djb2(prefix.c_str());
        if(hash_djb2("ENC") == hash_value)
        {
            std::istringstream input_stream(message);
            std::string line;
            for(size_t i = 0; i < encoders_.size(); i++)
            {
                std::getline(input_stream, line);
                double velocity_rpm = extractRPM(line);
                encoders_.at(i)->setVelocity(velocity_rpm);
            }
        }
        else
        {
            throw PREFIX_INVALID;
        }
    }
    catch(errors code){
        printErrorCode(code);
    }
}

double SerialRobotInterface::extractRPM(const std::string& message)
{
    size_t pos_msg = message.find(ID_RPM_STRING);
    size_t pos_separator = message.find(SEPARATOR_STRING);

    if(std::string::npos == pos_msg || std::string::npos == pos_separator)
    {
        throw POSITION_MESSAGE_ID_ERROR;
    }
    /* the rpm velocity value is after the string "RPM: " (in this case 5 characters)*/
    size_t length_rpm = pos_separator - pos_msg - RPM_MESSAGE_OFFSET;
    std::string rpmSubstring = message.substr(pos_msg + RPM_MESSAGE_OFFSET, length_rpm);

    // Convert the substrings to double and unsigned long
    double velocity_rpm;
    std::istringstream(rpmSubstring) >> velocity_rpm;
    return velocity_rpm;
}

double mapRange(double val1, double max1, double max2)
{
    double val2 = 0.0;
    if(max1 != 0.0)
        val2 = val1 * max2 / max1;
    return val2;
}

double saturate(double val, double max_val)
{
    if(val > max_val)
        return max_val;
    else if(val < -max_val)
        return -max_val;
    else
        return val;
}

unsigned long hash_djb2(const std::string& str) {
    unsigned long hash = 5381;
    for (char c : str) {
        hash = ((hash << 5) + hash) + c;
    }

    return hash;
}