#ifndef SERIAL_ROBOT_INTERFACE_H
#define SERIAL_ROBOT_INTERFACE_H

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <cmath>
#include <vector>
#include "encoder.h"
#include <iomanip>  // For formatting the double variable
#include <sstream>  // For converting int and double to string

#include <stdexcept> // exception handling
#include <stdio.h> // standard input / output functions
#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h> // File control definitions
#include <errno.h> // Error number definitions
#include <termios.h> // POSIX terminal control definitionss
#include <time.h>   // time calls
#include <string>

#define INVALID_HANDLE -1
#define CTRL_MOTOR_MAX_VALUE 1000
#define ID_RPM_STRING 'V' // string that identifies an encoder measurement
#define ID_TIME_STRING 'T' // string that identifies a time value
#define RPM_MESSAGE_OFFSET 1 // offset before the rpm value of the encoder measurement
#define SEPARATOR_STRING ','
#define BUFFER_SIZE 256
#define WAIT_AFTER_MESSAGE_TX_MS 10

double mapRange(double val1, double max1, double max2);
double saturate(double val, double max_val);
unsigned long hash_djb2(const std::string& str);


class SerialRobotInterface 
{
private:
    int serial_handle_;
    speed_t baud_rate_;
    std::vector<std::shared_ptr<Encoder>> encoders_;

    void sendMotorCmd(int id, double cmd_value);
    void sendMessage(const std::string& message);
    std::string readLine() const;
    void elaborateMessage(const std::string& message);
    void parseEncodersMessage(const std::string& message);
    void initSerialPort();
    unsigned long extractTime(const std::string& message);
    double extractRPM(const std::string& message);
    void disconnect();
    bool isConnected() const;
public:
    SerialRobotInterface(const std::string& port, speed_t baud_rate, std::vector<std::shared_ptr<Encoder>> encoders);
    ~SerialRobotInterface() {
        disconnect();
    }

    void readEncodersMeasurements();
    void setMotorSpeed(int id, double velocity);
};



#endif

