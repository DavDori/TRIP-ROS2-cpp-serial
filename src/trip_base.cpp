#include <stdio.h>
#include <string.h>

#include <iostream>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "trip_interface/serial_robot_interface.h"
#include "trip_interface/encoder.h"
#include "trip_interface/motor.h"

#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"

#define MILLI 1000.0

class RobotNode : public rclcpp::Node 
{
private:
    rclcpp::TimerBase::SharedPtr timer_enc;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr enc_pub;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr cmd_sub;

    std::shared_ptr<SerialRobotInterface> Device;
    std::shared_ptr<Encoder> EncoderLeft;
    std::shared_ptr<Encoder> EncoderRight;
    std::shared_ptr<Motor> MotorLeft;
    std::shared_ptr<Motor> MotorRight;

    double getTimeSec()
    {
        return this->get_clock()->now().seconds();
    }

public:
    RobotNode() : rclcpp::Node("trip")
    {
        double ppr, max_rpm;
        std::string port;
        long baud_rate;
        int rate_feedback;

        this->declare_parameter("port", "/dev/ttyUSB0");
        this->declare_parameter("baudrate", 9600);
        this->declare_parameter("motor_max_rpm", 30.0);
        this->declare_parameter("pub_rate_feedback_Hz", 1);
        this->declare_parameter("pulse_per_revolution", 1024.0);

        this->get_parameter("port", port);
        this->get_parameter("baudrate", baud_rate);
        this->get_parameter("motor_max_rpm", max_rpm);
        this->get_parameter("pub_rate_feedback_Hz", rate_feedback);
        this->get_parameter("pulse_per_revolution", ppr);

        try{
            EncoderLeft.reset(new Encoder(0, ppr));
            EncoderRight.reset(new Encoder(1, ppr));
            Device.reset(new SerialRobotInterface(
                port, 
                baud_rate, 
                std::vector<std::shared_ptr<Encoder>>{EncoderLeft,EncoderRight}));

            MotorLeft.reset(new Motor(0, Device));
            MotorRight.reset(new Motor(1, Device));
        }
        catch(std::exception& e)
        {
            std::cout << e.what() << std::endl;
        }   

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);
        
        enc_pub = this->create_publisher<sensor_msgs::msg::JointState>("trip/encoders", 1);
        cmd_sub = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joints_cmd", qos, 
            std::bind(&RobotNode::velocityCmdCallback, this, std::placeholders::_1)
            );
        timer_enc = this->create_wall_timer(
            std::chrono::milliseconds(int(MILLI / rate_feedback)), 
            std::bind(&RobotNode::feedbackCallback, this)
            );
    }

    ~RobotNode(){}

    void velocityCmdCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        try
        {
            setMotorSpeeds(msg);
        }
        catch(std::exception& e)
        {
            std::cout << e.what() << std::endl;
        }   
    }

    void setMotorSpeeds(const sensor_msgs::msg::JointState::SharedPtr joint_state_msg)
    {
        if (joint_state_msg->velocity.size() < 2)
            throw std::length_error("Error: velocity component of joint state message must be > 2 [v,omega]");

        double motor_left_speed = joint_state_msg->velocity[0];
        double motor_right_speed = joint_state_msg->velocity[1];

        MotorLeft->moveRADPS(motor_left_speed);
        MotorRight->moveRADPS(motor_right_speed);
    }


    void feedbackCallback() 
    {
        try
        {
            sendEncoderMessage();
        }
        catch(std::exception& e)
        {
            std::cout << e.what() << std::endl;
        }   
    }

    void sendEncoderMessage()
    {
        sensor_msgs::msg::JointState msg;

        msg.name = {"left", "right"};
        msg.position = std::vector<double>{
            EncoderLeft->getRadiants(),
            EncoderRight->getRadiants()};
        msg.velocity = std::vector<double>{
            EncoderLeft->getSpeedRPM(),
            EncoderRight->getSpeedRPM()};

        enc_pub->publish(msg);
    }
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
