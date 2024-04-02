#include <gtest/gtest.h>
#include "trip_interface/trip_serial_interface.h"

// Test case for extractEncodersVelocity function
TEST(InterfaceFunctionsTest, ExtractEncodersVelocity) {
    std::string msg = "E0V1.6,E1V20.1,E2V15.5,\n"; // Assuming this is the format of the message containing encoder pulses
    
    std::vector<double> expected_velocities = {1.6, 20.1, 15.5};
    std::vector<double> result = extractEncodersVelocity(msg);
    
    ASSERT_EQ(result.size(), expected_velocities.size());
    for (size_t i = 0; i < result.size(); ++i) {
        EXPECT_DOUBLE_EQ(result[i], expected_velocities[i]);
    }
}

// Test case for extractEncodersPulses function
TEST(InterfaceFunctionsTest, ExtractEncodersPulses) {
    std::string msg = "E0P100,E1P200,E2P150,\n"; // Assuming this is the format of the message containing encoder pulses
    
    std::vector<long> expected_pulses = {100, 200, 150};
    std::vector<long> result = extractEncodersPulses(msg);
    
    ASSERT_EQ(result.size(), expected_pulses.size());
    for (size_t i = 0; i < result.size(); ++i) {
        EXPECT_EQ(result[i], expected_pulses[i]);
    }
}

// Test case for encodeVelocitySetpoint function
TEST(InterfaceFunctionsTest, EncodeVelocitySetpoint) {
    int id = 1;
    
    std::string result = encodeVelocitySetpoint(100.0, id);
    EXPECT_EQ(result, "CSET,1,100.00\n");
    result = encodeVelocitySetpoint(100.0, id);
    EXPECT_EQ(result, "CSET,1,100.00\n");    
}

// Test case for encodeVelocityAbsolute function
TEST(InterfaceFunctionsTest, EncodeVelocityAbsolute) {
    int id = 2;
    
    std::string result = encodeVelocityAbsolute(0.5, id);
    EXPECT_EQ(result, "MSET,2,0.500\n");
    result = encodeVelocityAbsolute(-0.4, id);
    EXPECT_EQ(result, "MSET,2,-0.400\n");

    EXPECT_ANY_THROW(encodeVelocityAbsolute(100, id));
    EXPECT_ANY_THROW(encodeVelocityAbsolute(-10, id));
}