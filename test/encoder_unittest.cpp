#include "trip_interface/encoder.h"
#include "trip_interface/serial_interface.h"
#include <pty.h>
#include <gtest/gtest.h>
#include <memory>

class EncoderTests : public ::testing::Test 
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
        enc0_ = std::make_shared<Encoder>(0, 200, device_);
        enc1_ = std::make_shared<Encoder>(1, 100, device_);
    }

    virtual void TearDown() 
    {
        device_->disconnect();
    }

    std::shared_ptr<SerialInterface> device_;
    std::shared_ptr<Encoder> enc0_;
    std::shared_ptr<Encoder> enc1_;

    int master_;
    int slave_;
    char name_[100];
};


// Test for Encoder() constructor
TEST_F(EncoderTests, Constructor)
{
    std::shared_ptr<SerialInterface> SerialDummy;
    Encoder encoder(99, 200.0, SerialDummy);
    EXPECT_EQ(encoder.getID(), 99);
    EXPECT_EQ(encoder.getPPR(), 200);

    EXPECT_ANY_THROW(Encoder encoder(1, 0.0, SerialDummy));
}

// Test for setReferencePulseCount() method
TEST_F(EncoderTests, SetReferencePulseCount) 
{
    std::shared_ptr<SerialInterface> SerialDummy;
    Encoder encoder(1,100.0,SerialDummy);
    encoder.setReferencePulseCount(100);
    EXPECT_EQ(encoder.getPulseCount(), -100);
    encoder.setReferencePulseCount(0.0);
    EXPECT_EQ(encoder.getPulseCount(), 0.0);
}


// Test for getPulseCount() method
TEST_F(EncoderTests, GetPulseCount) 
{
    write(master_, "E0P1000V5.00T300,E1P500V8.40T303,\n", 33);
    device_->readLine();
    enc0_->readMeasurement();
    enc1_->readMeasurement();
    EXPECT_EQ(enc0_->getPulseCount(), 1000.0); 
    EXPECT_EQ(enc1_->getPulseCount(), 500.0);
}

// Test for getRevolutions() method
TEST_F(EncoderTests, GetRevolutions) 
{
    write(master_, "E0P1000V5.00T300,E1P500V8.40T303,\n", 33);
    device_->readLine();
    enc0_->readMeasurement();
    enc1_->readMeasurement();
    EXPECT_NEAR(enc0_->getRevolutions(), 5.0, 0.000001); // 1000 pulses / 200 pulses per revolution = 5 revolutions
    EXPECT_NEAR(enc1_->getRevolutions(), 5.0, 0.000001); // 500 pulses / 100 pulses per revolution = 5 revolutions
}

// Test for getRadiants() method
TEST_F(EncoderTests, GetRadiants) 
{
    write(master_, "E0P400V5.00T300,E1P500V8.40T303,\n", 33);
    device_->readLine();
    enc0_->readMeasurement();
    enc1_->readMeasurement();
    EXPECT_NEAR(enc0_->getRadiants(), 4.0 * M_PI, 0.000001); 
    EXPECT_NEAR(enc1_->getRadiants(), 10.0 * M_PI, 0.000001);
}

// Test for getDegrees() method
TEST_F(EncoderTests, GetDegrees) 
{
    write(master_, "E0P400V5.00T300,E1P500V8.40T303,\n", 33);
    device_->readLine();
    enc0_->readMeasurement();
    enc1_->readMeasurement();
    EXPECT_NEAR(enc0_->getDegrees(), 720.0, 0.000001); 
    EXPECT_NEAR(enc1_->getDegrees(), 5.0*360.0, 0.000001);
}


// Test for getSpeedRPM() method
TEST_F(EncoderTests, GetSpeedRoundsPerMinute) 
{
    write(master_, "E0P400V5.00T300,E1P500V8.40T303,\n", 33);
    device_->readLine();
    enc0_->readMeasurement();
    enc1_->readMeasurement();
    EXPECT_EQ(enc0_->getSpeedRPM(), 5.0); 
    EXPECT_EQ(enc1_->getSpeedRPM(), 8.4);
}

// Test for getSpeedRADpS() method
TEST_F(EncoderTests, GetSpeedRadiantsPerSecond) 
{
    write(master_, "E0P400V5.00T300,E1P500V8.40T303,\n", 33);
    device_->readLine();
    enc0_->readMeasurement();
    enc1_->readMeasurement();
    EXPECT_NEAR(enc0_->getSpeedRADpS(), 0.523599, 0.000001); 
    EXPECT_NEAR(enc1_->getSpeedRADpS(), 0.879645, 0.000001);
}

// Test for getSpeedDEGpS() method
TEST_F(EncoderTests, GetSpeedDegreesPerSecond) 
{
    write(master_, "E0P400V5.00T300,E1P500V8.40T303,\n", 33);
    device_->readLine();
    enc0_->readMeasurement();
    enc1_->readMeasurement();
    EXPECT_NEAR(enc0_->getSpeedDEGpS(), 30.0, 0.000001); 
    EXPECT_NEAR(enc1_->getSpeedDEGpS(), 50.4, 0.000001);
}