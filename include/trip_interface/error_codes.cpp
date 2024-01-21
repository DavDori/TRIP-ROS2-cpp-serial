#include "error_codes.h"

void printErrorCode(errors code)
{
    switch (code)
    {
    case FAILED_SET_MOTOR_CMD:
        std::cout << "Failed to send motor command CODE:["<< code << "]" << std::endl;
        break;
    case FAILED_CONNECTION:
        std::cout << "Failed to setup connection CODE:["<< code << "]" << std::endl;
        break;
    case FAILED_READ_ENCODER_COUNT:
        std::cout << "Failed to read encoder count CODE:["<< code << "]" << std::endl;
        break;
    case FAILED_READ_ENCODER_SPEED:
        std::cout << "Failed to read encoder speed CODE:["<< code << "]" << std::endl;
        break;
    case WORNG_INDEXING:
        std::cout << "Index is not allowed, choose 0 or 1 CODE:["<< code << "]" << std::endl;
        break;
    case WRONG_SIZE_JOINTSTATE_CMD:
        std::cout << "ERROR: JointState message does not have exactly 2 elements in the 'position' array. CODE:["<< code << "]"<< std::endl;
        break;
    case PREFIX_INVALID:
        std::cout << "ERROR: message prefix invalid. CODE:["<< code << "]"<< std::endl;
        break;
    case POSITION_MESSAGE_ID_ERROR:
        std::cout << "ERROR: expected id was not found in the message recived. CODE:["<< code << "]"<< std::endl;
        break;
    case CANNOT_OPEN_PORT:
        std::cout << "ERROR: cannot open serial port. CODE:["<< code << "]"<< std::endl;
        break;
    case DEVICE_NOT_CONNECTED:
        std::cout << "ERROR: there is not connected device. CODE:["<< code << "]"<< std::endl;
        break;
    case TRANSMISSION_FAILED:
        std::cout << "ERROR: serial data transmission failed. CODE:["<< code << "]"<< std::endl;
        break;
    case NO_SERIAL_RECEIVE:
        std::cout << "ERROR: no serial message has been recived. CODE:["<< code << "]"<< std::endl;
        break;
    case SERIAL_IO:
        std::cout << "ERROR: serial IO. CODE:["<< code << "]"<< std::endl;
        break;
    default:
        std::cout << "Code not found CODE:["<< code << "]" << std::endl;
        break;
    }
}