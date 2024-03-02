#include <stdio.h>
#include <string.h>
#include <iostream>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "trip_interface/serial_robot_interface.h"
#include "trip_interface/encoder.h"
#include "trip_interface/motor.h"
#include "trip_interface/differential_drive_model.h"

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

	std::shared_ptr<DifferentialDriveModel> Model_;
    std::shared_ptr<SerialRobotInterface> Device_;
    std::shared_ptr<Encoder> EncoderLeft_;
    std::shared_ptr<Encoder> EncoderRight_;
    std::shared_ptr<Motor> MotorLeft_;
    std::shared_ptr<Motor> MotorRight_;

    double getTimeSec()
    {
        return this->get_clock()->now().seconds();
    }

public:
    RobotNode() : rclcpp::Node("trip_serial")
    {
        double ppr;
        double max_rpm;
        double wheel_radius, wheel_distance, gearbox;
        std::string port;
        long baud_rate;
        int rate_feedback;

        this->declare_parameter("port", "/dev/ttyACM0");
        this->declare_parameter("baudrate", 9600);
        this->declare_parameter("motor_max_rpm", 20.0);
        this->declare_parameter("sprocket_radius_m", 0.035);
        this->declare_parameter("track_distance_m", 0.16);
        this->declare_parameter("gearbox_ratio", 1.0);
        this->declare_parameter("pub_rate_feedback_Hz", 20);
        this->declare_parameter("pulse_per_revolution", 1024.0);

        this->get_parameter("port", port);
        this->get_parameter("baudrate", baud_rate);
        this->get_parameter("motor_max_rpm", max_rpm);
        this->get_parameter("sprocket_radius_m", wheel_radius);
        this->get_parameter("track_distance_m", wheel_distance);
        this->get_parameter("gearbox_ratio", gearbox);
        this->get_parameter("pub_rate_feedback_Hz", rate_feedback);
        this->get_parameter("pulse_per_revolution", ppr);

        try{
            EncoderLeft_.reset(new Encoder(0, ppr));
            EncoderRight_.reset(new Encoder(1, ppr));
            Device_.reset(new SerialRobotInterface(
                port, 
                baud_rate, 
                std::vector<std::shared_ptr<Encoder>>{EncoderLeft_,EncoderRight_}));
            MotorLeft_.reset(new Motor(0, Device_));
            MotorRight_.reset(new Motor(1, Device_));
            Model_.reset(new DifferentialDriveModel(wheel_radius, wheel_distance, 1.0));
        }
        catch(std::exception& e)
        {
            std::cout << e.what() << std::endl;
        }   

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);
        
        enc_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("trip/encoders", 1);
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", qos, 
            std::bind(&RobotNode::velocityCmdCallback, this, std::placeholders::_1)
            );
        timer_enc_ = this->create_wall_timer(
            std::chrono::milliseconds(int(MILLI / rate_feedback)), 
            std::bind(&RobotNode::feedbackCallback, this)
            );
    }

    ~RobotNode(){}

    void velocityCmdCallback(const geometry_msgs::msg::Twist msg)
    {
        double lin_vel = msg.linear.x;
        double ang_vel = msg.angular.z;
        /*
        convert unicycle velocities [linear, angular] into each 
        motors required speed to achieve that unicycle behaviour
        */
        Model_->setUnicycleSpeed(lin_vel, ang_vel);
        double motor_left_speed = Model_->getLeftMotorRotationalSpeed();
        double motor_right_speed = Model_->getRightMotorRotationalSpeed();
        setMotorSpeeds(motor_left_speed, motor_right_speed);
    }

    void setMotorSpeeds(double motor_left_speed, double motor_right_speed)
    {
        MotorLeft_->moveRADPS(motor_left_speed);
        MotorRight_->moveRADPS(motor_right_speed);
    }


    void feedbackCallback() 
    {
        Device_->readEncodersMeasurements();
        sendEncoderMessage();
    }

    void sendEncoderMessage()
    {
        sensor_msgs::msg::JointState msg;

        msg.name = {"left", "right"};
        msg.position = std::vector<double>{
            EncoderLeft_->getRadiants(),
            EncoderRight_->getRadiants()};
        msg.velocity = std::vector<double>{
            EncoderLeft_->getSpeedRPM(),
            EncoderRight_->getSpeedRPM()};

        enc_pub_->publish(msg);
    }
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
