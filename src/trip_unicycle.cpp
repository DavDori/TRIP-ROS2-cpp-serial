#include <stdio.h>
#include <string.h>
#include <iostream>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "trip_interface/trip_serial_interface.h"
#include "trip_interface/encoder.h"
#include "trip_interface/motor.h"
#include "trip_interface/differential_drive_model.h"
#include "serial/serial.h"

#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"

#define MILLI 1000.0

class RobotNode : public rclcpp::Node 
{
private:
    rclcpp::TimerBase::SharedPtr timer_enc_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr enc_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;

	std::shared_ptr<DifferentialDriveModel> model_;
    std::shared_ptr<serial::Serial> com_handle_;
    std::shared_ptr<Encoder> encoder_l_;
    std::shared_ptr<Encoder> encoder_r_;
    std::shared_ptr<Motor> motor_l_;
    std::shared_ptr<Motor> motor_r_;

public:
    RobotNode() : rclcpp::Node("trip_serial")
    {
        double ppr;
        double max_rpm;
        double wheel_radius, wheel_distance, gearbox;
        std::string port;
        long baud_rate;
        int rate_enc;

        this->declare_parameter("port", "/dev/ttyACM0");
        this->declare_parameter("baudrate", 19200);
        this->declare_parameter("motor_max_rpm", 12.0);
        this->declare_parameter("sprocket_radius_m", 0.035);
        this->declare_parameter("track_distance_m", 0.16);
        this->declare_parameter("gearbox_ratio", 1.0);
        this->declare_parameter("pub_rate_enc_Hz", 10);
        this->declare_parameter("pulse_per_revolution", 1024.0);

        this->get_parameter("port", port);
        this->get_parameter("baudrate", baud_rate);
        this->get_parameter("motor_max_rpm", max_rpm);
        this->get_parameter("sprocket_radius_m", wheel_radius);
        this->get_parameter("track_distance_m", wheel_distance);
        this->get_parameter("gearbox_ratio", gearbox);
        this->get_parameter("pub_rate_enc_Hz", rate_enc);
        this->get_parameter("pulse_per_revolution", ppr);

        try{
            com_handle_.reset(new serial::Serial(port, baud_rate));
            model_.reset(new DifferentialDriveModel(wheel_radius, wheel_distance, gearbox));
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
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", qos, 
            std::bind(&RobotNode::velocityCmdCallback, this, std::placeholders::_1)
            );
        timer_enc_ = this->create_wall_timer(
            std::chrono::milliseconds(int(MILLI / rate_enc)), 
            std::bind(&RobotNode::encoderCallback, this)
            );
    }

    ~RobotNode(){com_handle_->close();}

    void velocityCmdCallback(const geometry_msgs::msg::Twist msg)
    {
        double lin_vel = msg.linear.x;
        double ang_vel = msg.angular.z;
        /*
        convert unicycle velocities [linear, angular] into each 
        motors required speed to achieve that unicycle behaviour
        */
        try
        {
            model_->setUnicycleVel(lin_vel, ang_vel);
            double motor_l_vel = model_->getLeftMotorRotationalVel();
            double motor_r_vel = model_->getRightMotorRotationalVel();
            motor_l_->setDesiredVelocity(motor_l_vel, RADPS);
            motor_r_->setDesiredVelocity(motor_r_vel, RADPS);
            double value_l = motor_l_->getDesiredVelocity(RPM);
            double value_r = motor_r_->getDesiredVelocity(RPM);
            std::string cmd_l = encodeVelocitySetpoint(value_l, 0);
            std::string cmd_r = encodeVelocitySetpoint(value_r, 1);

            com_handle_->write(cmd_l + cmd_r);
        }
        catch(const std::exception& e)
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
