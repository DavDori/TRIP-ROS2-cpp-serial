#include <stdio.h>
#include <string.h>

#include <iostream>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "trip_interface/trip_serial_interface.h"
#include "trip_interface/encoder.h"
#include "trip_interface/motor.h"
#include "serial/serial.h"

#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"

#define MILLI 1000.0

class RobotNode : public rclcpp::Node 
{
private:
    rclcpp::TimerBase::SharedPtr timer_enc_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr enc_pub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr cmd_sub_;

    std::shared_ptr<serial::Serial> com_handle_;
    std::shared_ptr<Encoder> encoder_l_;
    std::shared_ptr<Encoder> encoder_r_;
    std::shared_ptr<Motor> motor_l_;
    std::shared_ptr<Motor> motor_r_;

    double getTimeSec()
    {
        return this->get_clock()->now().seconds();
    }

public:
    RobotNode() : rclcpp::Node("trip")
    {
        double ppr;
        double max_rpm;
        std::string port;
        long baud_rate;
        int rate_enc;

        this->declare_parameter("port", "/dev/ttyACM0");
        this->declare_parameter("baudrate", 19200);
        this->declare_parameter("motor_max_rpm", 20.0);
        this->declare_parameter("pub_rate_enc_Hz", 5);
        this->declare_parameter("pulse_per_revolution", 1024.0);

        this->get_parameter("port", port);
        this->get_parameter("baudrate", baud_rate);
        this->get_parameter("motor_max_rpm", max_rpm);
        this->get_parameter("pub_rate_enc_Hz", rate_enc);
        this->get_parameter("pulse_per_revolution", ppr);

        try{
            com_handle_.reset(new serial::Serial(port, baud_rate));
            encoder_l_.reset(new Encoder(ppr));
            encoder_r_.reset(new Encoder(ppr));
            motor_l_.reset(new Motor(max_rpm));
            motor_r_.reset(new Motor(max_rpm));
        }
        catch(std::exception& e)
        {
            std::cout << e.what() << std::endl;
            rclcpp::shutdown();
        }   

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);
        
        enc_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("trip/enc", 1);
        cmd_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joints_cmd", qos, 
            std::bind(&RobotNode::velocityCmdCallback, this, std::placeholders::_1)
            );
        timer_enc_ = this->create_wall_timer(
            std::chrono::milliseconds(int(MILLI / rate_enc)), 
            std::bind(&RobotNode::encoderCallback, this)
            );
    }

    ~RobotNode(){com_handle_->close();}

    void velocityCmdCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        try
        {
            if (msg->velocity.size() < 2)
                throw std::length_error("Error: velocity component of joint state message must be > 2 [v,omega]");

            double motor_l_vel = msg->velocity[0];
            double motor_r_vel = msg->velocity[1];
            motor_l_->setDesiredVelocity(motor_l_vel, RADPS);
            motor_r_->setDesiredVelocity(motor_r_vel, RADPS);
            double value_l = motor_l_->getDesiredVelocity(RPM);
            double value_r = motor_r_->getDesiredVelocity(RPM);
            std::string cmd_l = encodeVelocitySetpoint(value_l, 0);
            std::string cmd_r = encodeVelocitySetpoint(value_r, 1);

            com_handle_->write(cmd_l + cmd_r);
        }
        catch(std::exception& e)
        {
            std::cout << e.what() << std::endl;
            rclcpp::shutdown();
        }   
    }

    void encoderCallback() 
    {
        try
        {
            com_handle_->write("E\n");
            std::string response = com_handle_->readline();
            sensor_msgs::msg::JointState msg;

            msg.name = {"left", "right"};
            msg.velocity = extractEncodersVelocity(response);

            enc_pub_->publish(msg);
        }
        catch(std::exception& e)
        {
            std::cout << e.what() << std::endl;
            rclcpp::shutdown();
        }   
    }
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
