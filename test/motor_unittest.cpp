#include "trip_interface/motor.h"
#include <gtest/gtest.h>


// Define some example values for testing
constexpr double MAX_RPM = 5000.0;
constexpr double TORQUE_CONSTANT = 0.002;

TEST(MotorTest, UnitConversion) {
    double rpm = convert_vel_from_rpm(100.0, RPM);
    double radps = convert_vel_from_rpm(100.0, RADPS);
    double degps = convert_vel_from_rpm(100.0, DEGPS);
    EXPECT_EQ(rpm, 100.0);
    EXPECT_NEAR(radps, 10.4719755, 0.000001);
    EXPECT_NEAR(degps, 600.0, 0.000001);

    rpm = convert_vel_from_rpm(convert_vel_into_rpm(100.0, RPM), RPM);
    radps = convert_vel_from_rpm(convert_vel_into_rpm(100.0, RADPS), RADPS);
    degps = convert_vel_from_rpm(convert_vel_into_rpm(100.0, DEGPS), DEGPS);
    EXPECT_NEAR(rpm, 100.0, 0.000001);
    EXPECT_NEAR(radps, 100.0, 0.000001);
    EXPECT_NEAR(degps, 100.0, 0.000001);
}

// Test case for constructor with mode, torque_constant, and max_rpm
TEST(MotorTest, ConstructorWithParameters) {
    Motor motor(VELOCITY, TORQUE_CONSTANT, MAX_RPM);

    EXPECT_EQ(motor.getActualVelocity(RPM), 0.0);
    EXPECT_EQ(motor.getDesiredVelocity(RPM), 0.0);
    EXPECT_EQ(motor.getActualTorque(), 0.0);
    EXPECT_EQ(motor.getDesiredTorque(), 0.0);
    EXPECT_EQ(motor.getActualCurrent(), 0.0);
    EXPECT_EQ(motor.getDesiredCurrent(), 0.0);
    EXPECT_FALSE(motor.isTorqueMode());
    EXPECT_TRUE(motor.isVelocityMode());
}

// Test case for constructor with max_rpm only
TEST(MotorTest, ConstructorWithMaxRpmOnly) {
    Motor motor(MAX_RPM);

    EXPECT_EQ(motor.getActualVelocity(RPM), 0.0);
    EXPECT_EQ(motor.getDesiredVelocity(RPM), 0.0);
    EXPECT_EQ(motor.getActualTorque(), 0.0);
    EXPECT_EQ(motor.getDesiredTorque(), 0.0);
    EXPECT_EQ(motor.getActualCurrent(), 0.0);
    EXPECT_EQ(motor.getDesiredCurrent(), 0.0);
    EXPECT_TRUE(motor.isVelocityMode());
    EXPECT_FALSE(motor.isTorqueMode());
}

// Test case for setting and getting actual velocity
TEST(MotorTest, SetAndGetActualVelocity) {
    Motor motor(MAX_RPM);
    motor.setActualVelocity(100.0, RPM);
    EXPECT_EQ(motor.getActualVelocity(RPM), 100.0);
    // greater than max rpm, but it should not affect since it
    // is the reading, not the setpoint
    motor.setActualVelocity(10000.0, RPM);
    EXPECT_EQ(motor.getActualVelocity(RPM), 10000.0);
}

// Test case for setting and getting desired velocity
TEST(MotorTest, SetAndGetDesiredVelocity) {
    Motor motor(MAX_RPM);
    motor.setDesiredVelocity(200.0, RPM);
    EXPECT_EQ(motor.getDesiredVelocity(RPM), 200.0);

    motor.setDesiredVelocity(20000.0, RPM);
    EXPECT_EQ(motor.getDesiredVelocity(RPM), MAX_RPM);
}

// Test case for setting and getting actual current
TEST(MotorTest, SetAndGetActualCurrent) {
    Motor motor(TORQUE, TORQUE_CONSTANT, MAX_RPM);
    motor.setActualCurrent(2.0);
    EXPECT_EQ(motor.getActualCurrent(), 2.0);
}

// Test case for setting and getting desired current
TEST(MotorTest, SetAndGetDesiredCurrent) {
    Motor motor(TORQUE, TORQUE_CONSTANT, MAX_RPM);
    motor.setDesiredCurrent(2.0);
    EXPECT_EQ(motor.getDesiredCurrent(), 2.0);
}

// Test case for setting and getting actual Torque
TEST(MotorTest, SetAndGetActualTorque) {
    Motor motor(TORQUE, TORQUE_CONSTANT, MAX_RPM);
    motor.setActualTorque(0.02);
    EXPECT_EQ(motor.getActualTorque(), 0.02);
}

// Test case for setting and getting desired Torque
TEST(MotorTest, SetAndGetDesiredTorque) {
    Motor motor(TORQUE, 2, MAX_RPM);
    motor.setDesiredTorque(0.02);
    EXPECT_EQ(motor.getDesiredTorque(), 0.02);
}