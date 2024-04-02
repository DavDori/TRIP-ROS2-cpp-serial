#include <gtest/gtest.h>
#include "trip_interface/differential_drive_model.h"

// Test case for constructor with all parameters
TEST(DifferentialDriveModelTest, ConstructorWithAllParameters) {
    const double wheel_radius = 0.1;
    const double wheel_distance = 0.5;
    const double gearbox = 2.0;

    EXPECT_NO_THROW({
        DifferentialDriveModel model(wheel_radius, wheel_distance, gearbox);
    });

    DifferentialDriveModel model(wheel_radius, wheel_distance, gearbox);
    EXPECT_EQ(model.getLinearVel(), 0.0);
    EXPECT_EQ(model.getAngularVel(), 0.0);
    EXPECT_EQ(model.getLeftMotorRotationalVel(), 0.0);
    EXPECT_EQ(model.getRightMotorRotationalVel(), 0.0);
    EXPECT_EQ(model.getLeftWheelRotationalVel(), 0.0);
    EXPECT_EQ(model.getRightWheelRotationalVel(), 0.0);
    EXPECT_EQ(model.getWheelRadius(), wheel_radius);
    EXPECT_EQ(model.getWheelDistance(), wheel_distance);
    EXPECT_EQ(model.getGearbox(), gearbox);

    EXPECT_ANY_THROW({
        DifferentialDriveModel model(-1.0, wheel_distance, gearbox);
    });
    EXPECT_ANY_THROW({
        DifferentialDriveModel model(wheel_radius, -1.0, gearbox);
    });
}

// Test case for constructor without gearbox
TEST(DifferentialDriveModelTest, ConstructorWithoutGearbox) {
    const double wheel_radius = 0.1;
    const double wheel_distance = 0.5;

    EXPECT_NO_THROW({
        DifferentialDriveModel model(wheel_radius, wheel_distance);
    });

    DifferentialDriveModel model(wheel_radius, wheel_distance);
    EXPECT_EQ(model.getLinearVel(), 0.0);
    EXPECT_EQ(model.getAngularVel(), 0.0);
    EXPECT_EQ(model.getLeftMotorRotationalVel(), 0.0);
    EXPECT_EQ(model.getRightMotorRotationalVel(), 0.0);
    EXPECT_EQ(model.getLeftWheelRotationalVel(), 0.0);
    EXPECT_EQ(model.getRightWheelRotationalVel(), 0.0);
    EXPECT_EQ(model.getWheelRadius(), wheel_radius);
    EXPECT_EQ(model.getWheelDistance(), wheel_distance);
}

// Test case for setUnicycleVel and getters for unicycle speeds
TEST(DifferentialDriveModelTest, SetGetUnicycleVel) {
    DifferentialDriveModel model(0.1, 0.5);

    const double lin_vel = 1.0;
    const double ang_vel = 0.5;
    model.setUnicycleVel(lin_vel, ang_vel);

    EXPECT_EQ(model.getLinearVel(), lin_vel);
    EXPECT_EQ(model.getAngularVel(), ang_vel);
}

// Test case for setDifferentialVel method
TEST(DifferentialDriveModelTest, SetDifferentialVel) {
    DifferentialDriveModel model(0.1, 0.5);

    const double left_wheel_vel = 1.0;
    const double right_wheel_vel = 0.5;
    model.setDifferentialVel(left_wheel_vel, right_wheel_vel);

    // Add assertions to check the calculated linear and angular velocities
    EXPECT_DOUBLE_EQ(model.getLinearVel(), 0.075);
    EXPECT_DOUBLE_EQ(model.getAngularVel(), -0.1);
}