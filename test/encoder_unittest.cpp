#include <gtest/gtest.h>
#include "trip_interface/encoder.h"
#include <gtest/gtest.h>
#include <memory>

// Test for Encoder() constructor
TEST(EncoderTest, Constructor)
{
    Encoder encoder(99,200);
    EXPECT_EQ(encoder.getID(), 99);
    EXPECT_EQ(encoder.getPPR(), 200);

    EXPECT_ANY_THROW(Encoder encoder(1, 0.0));
}

// Test for setReferencePulseCount() method
TEST(EncoderTest, SetReferencePulseCount) 
{
    Encoder encoder(1,100);
    encoder.setReferencePulseCount(100);
    EXPECT_EQ(encoder.getPulseCount(), -100);
    encoder.setReferencePulseCount(0.0);
    EXPECT_EQ(encoder.getPulseCount(), 0.0);
}

// Test for setVelocity() method
TEST(EncoderTest, SetVelocity) 
{
    Encoder encoder(1,200);

    encoder.setVelocityRPM(50);
    EXPECT_EQ(encoder.getSpeedRPM(), 50);
}

// Test for setPulseCount() method
TEST(EncoderTest, SetPulseCount) 
{
    Encoder encoder(1,200);

    encoder.setPulseCount(200);
    EXPECT_EQ(encoder.getPulseCount(), 200);
}

// Test for getRevolutions() method
TEST(EncoderTest, GetRevolutions) 
{
    Encoder encoder(1,200);

    encoder.setPulseCount(1000);
    EXPECT_EQ(encoder.getRevolutions(), 5); // 1000 pulses / 200 pulses per revolution = 5 revolutions
}

// Test for getRadiants() method
TEST(EncoderTest, GetRadiants) 
{
    Encoder encoder(1,20);

    encoder.setPulseCount(50);
    EXPECT_NEAR(encoder.getRadiants(), 5*M_PI, 0.000001); // 50 pulses * (2 * pi) / (20 pulses per revolution) = pi/2 radians
}

// Test for getDegrees() method
TEST(EncoderTest, GetDegrees) 
{
    Encoder encoder(1,20);

    encoder.setPulseCount(10);
    EXPECT_NEAR(encoder.getDegrees(), 180, 0.000001); // 30 pulses * 360 degrees / 20 pulses per revolution = 540 degrees
}

