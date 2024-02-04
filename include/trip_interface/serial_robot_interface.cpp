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
    initSerialPort();

}

void SerialRobotInterface::initSerialPort()
{
    if (!isConnected())
        throw DEVICE_NOT_CONNECTED;
    struct termios tty;
    tcgetattr(serial_handle_, &tty);

      // Read in existing settings, and handle any error
    if(tcgetattr(serial_handle_, &tty) != 0) {
        throw DEVICE_NOT_CONNECTED;
    }

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size 
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    int timeout_ms = 100;
    tty.c_cc[VTIME] = (cc_t)(timeout_ms/100);    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;
    // Set the Tx and Rx Baud Rate to BOUDRATE
    cfsetospeed(&tty, (speed_t)baud_rate_);
    cfsetispeed(&tty, (speed_t)baud_rate_);


    /* Flush the Input buffer and set the attribute NOW without waiting for Data to Complete*/
    tcflush(serial_handle_, TCIFLUSH);
    tcsetattr(serial_handle_, TCSANOW, &tty);
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
    std::string msg_out = "E\n"; 
    try
    {
        sendMessage(msg_out);
        std::string msg_in = readLine();
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
    oss << "CSET," << id << "," << std::fixed << std::setprecision(2) << velocity << std::endl;
    std::string command = oss.str();
    sendMessage(command);
}

void SerialRobotInterface::sendMotorCmd(int id, double cmd_value)
{
    std::ostringstream oss;
    oss << "MSET," << id << "," << std::fixed << std::setprecision(2) << cmd_value << std::endl;
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
}

std::string SerialRobotInterface::readLine() const
{
    if(!isConnected())
        throw DEVICE_NOT_CONNECTED;
    char c;
    std::string message = "";
    int bytes_read;
    while ((bytes_read = read(serial_handle_, &c, 1)) > 0) 
    {
        if (bytes_read > 0) {
            if (c == '\n' || c == '\r') {
                break;
            }
            message.push_back(c);
        } else if (bytes_read < 0) {
            break;
        }
    }
    if (bytes_read < 0) {
        if (errno == EAGAIN)
            throw SERIAL_IO;
        else
            throw NO_SERIAL_RECEIVE;
    }
    return message;
}



void SerialRobotInterface::elaborateMessage(const std::string &message)
{
    // The hash function converts the string into a integer number
    
    try
    {
        if(message.size() == 0)
        {
            throw EMPTY_STRING;               
        }
        char id = message.at(0);
        if('E' == id)
        {
            parseEncodersMessage(message);
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