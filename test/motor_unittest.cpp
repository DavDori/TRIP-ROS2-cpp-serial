#include "trip_interface/motor.h"
#include "trip_interface/serial_interface.h"
#include <pty.h>
#include <gtest/gtest.h>
#include <memory>

class MotorTests : public ::testing::Test 
{
protected:
    virtual void SetUp() 
    {
        if (openpty(&master_, &slave_, name_, NULL, NULL) == -1) 
        {
            perror("openpty");
            exit(127);
        }

        ASSERT_TRUE(std::string(name_).length() > 0);
        ASSERT_TRUE(master_ > 0);
        ASSERT_TRUE(slave_ > 0);

        device_ = std::make_shared<SerialInterface>(std::string(name_), 9600);
        motor0_ = std::make_shared<Motor>(0, 15.0, device_);
        motor1_ = std::make_shared<Motor>(1, 20.5, device_);
    }

    virtual void TearDown() 
    {
        device_->disconnect();
    }

    std::shared_ptr<SerialInterface> device_;
    std::shared_ptr<Motor> motor0_;
    std::shared_ptr<Motor> motor1_;

    int master_;
    int slave_;
    char name_[100];
};



// Test for Encoder() constructor
TEST_F(MotorTests, Constructor)
{
    EXPECT_EQ(motor0_->getID(), 0);
    EXPECT_EQ(motor1_->getID(), 1);
    EXPECT_EQ(motor0_->getMaxRPM(), 15.0);
    EXPECT_EQ(motor1_->getMaxRPM(), 20.5);
}

// Test for moveRPM() method when setting in range  and positive values
TEST_F(MotorTests, MoveInRangePositiveRPM) 
{
    double in_range_rpm = 5.23;
    char buffer[12] = "";

    motor0_->moveRPM(in_range_rpm);
    read(master_, buffer, 12);

    EXPECT_EQ(std::string(buffer, 12), std::string("CSET,0,5.23\n"));
    EXPECT_EQ(motor0_->getRPM(), 5.23);

    motor1_->moveRPM(in_range_rpm);
    read(master_, buffer, 12);

    EXPECT_EQ(std::string(buffer, 12), std::string("CSET,1,5.23\n"));
    EXPECT_EQ(motor1_->getRPM(), 5.23);
}

// Test for moveRPM() method when setting in range and negative values
TEST_F(MotorTests, MoveInRangeNegativeRPM) 
{
    double in_range_rpm = -5.23;
    char buffer[13] = "";

    motor0_->moveRPM(in_range_rpm);
    read(master_, buffer, 13);

    EXPECT_EQ(std::string(buffer, 13), std::string("CSET,0,-5.23\n"));
    EXPECT_EQ(motor0_->getRPM(), -5.23);

    motor1_->moveRPM(in_range_rpm);
    read(master_, buffer, 13);

    EXPECT_EQ(std::string(buffer, 13), std::string("CSET,1,-5.23\n"));
    EXPECT_EQ(motor1_->getRPM(), -5.23);
}

// Test for moveRPM() method when setting out of range and positive values
TEST_F(MotorTests, MoveOutOfRangePositiveRPM) 
{
    double out_of_range_rpm = 1000;
    char buffer[13] = "";

    motor0_->moveRPM(out_of_range_rpm);
    read(master_, buffer, 13);

    EXPECT_EQ(std::string(buffer, 13), std::string("CSET,0,15.00\n"));
    EXPECT_EQ(motor0_->getRPM(), 15.0);

    motor1_->moveRPM(out_of_range_rpm);
    read(master_, buffer, 13);

    EXPECT_EQ(std::string(buffer, 13), std::string("CSET,1,20.50\n"));
    EXPECT_EQ(motor1_->getRPM(), 20.5);
}

// Test for moveRPM() method when setting out of range and negative values
TEST_F(MotorTests, MoveOutOfRangeNegativeRPM) 
{
    double out_of_range_rpm = -1000;
    char buffer[14] = "";

    motor0_->moveRPM(out_of_range_rpm);
    read(master_, buffer, 14);

    EXPECT_EQ(std::string(buffer, 14), std::string("CSET,0,-15.00\n"));
    EXPECT_EQ(motor0_->getRPM(), -15.0);

    motor1_->moveRPM(out_of_range_rpm);
    read(master_, buffer, 14);

    EXPECT_EQ(std::string(buffer, 14), std::string("CSET,1,-20.50\n"));
    EXPECT_EQ(motor1_->getRPM(), -20.5);
}


// no feedback control

// Test for move() method when setting in range  and positive values
TEST_F(MotorTests, MoveInRangePositive) 
{
    double in_range = 0.9;
    char buffer[13] = "";

    motor0_->move(in_range);
    read(master_, buffer, 13);

    EXPECT_EQ(std::string(buffer, 13), std::string("MSET,0,0.900\n"));

    motor1_->move(in_range);
    read(master_, buffer, 13);

    EXPECT_EQ(std::string(buffer, 13), std::string("MSET,1,0.900\n"));
}

// Test for move() method when setting in range and negative values
TEST_F(MotorTests, MoveInRangeNegative) 
{
    double in_range = -0.9;
    char buffer[14] = "";

    motor0_->move(in_range);
    read(master_, buffer, 14);

    EXPECT_EQ(std::string(buffer, 14), std::string("MSET,0,-0.900\n"));

    motor1_->move(in_range);
    read(master_, buffer, 14);

    EXPECT_EQ(std::string(buffer, 14), std::string("MSET,1,-0.900\n"));
}

// Test for move() method when setting out of range and positive values
TEST_F(MotorTests, MoveOutOfRangePositive) 
{
    double out_of_range = 10;
    char buffer[13] = "";

    motor0_->move(out_of_range);
    read(master_, buffer, 13);

    EXPECT_EQ(std::string(buffer, 13), std::string("MSET,0,1.000\n"));

    motor1_->move(out_of_range);
    read(master_, buffer, 13);

    EXPECT_EQ(std::string(buffer, 13), std::string("MSET,1,1.000\n"));
}

// Test for move() method when setting out of range and negative values
TEST_F(MotorTests, MoveOutOfRangeNegative) 
{
    double out_of_range = -1000;
    char buffer[14] = "";

    motor0_->move(out_of_range);
    read(master_, buffer, 14);

    EXPECT_EQ(std::string(buffer, 14), std::string("MSET,0,-1.000\n"));

    motor1_->move(out_of_range);
    read(master_, buffer, 14);

    EXPECT_EQ(std::string(buffer, 14), std::string("MSET,1,-1.000\n"));
}