#ifndef SERIAL_INTERFACE_H
#define SERIAL_INTERFACE_H

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <cmath>
#include <vector>
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
#define BUFFER_SIZE 256
#define WAIT_AFTER_MESSAGE_TX_MS 10


class SerialInterface 
{
private:
    int serial_handle_;
    speed_t baud_rate_;
    std::string last_message_;

    void initSerialPort();
    bool isConnected() const;
public:
    SerialInterface(const std::string& port, speed_t baud_rate);
    SerialInterface();
    ~SerialInterface() {
        disconnect();
    }

    void disconnect();
    void connect(const std::string& port, speed_t baud_rate);
    std::string readLine();

    std::string getLastMessage() const {return last_message_;}
    void send(const std::string& message);
};


#endif

