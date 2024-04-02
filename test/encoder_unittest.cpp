#include "trip_interface/encoder.h"
#include <gtest/gtest.h>
#include <memory>



// Test for Encoder() constructor
TEST(EncoderTests, Constructor)
{
    Encoder encoder(200.0);
    EXPECT_EQ(encoder.getPPR(), 200);

    EXPECT_ANY_THROW(Encoder encoder(0.0));
}

// Test for setReferencePulseCount() method
TEST(EncoderTests, SetReferencePulseCount) 
{
    Encoder encoder(100.0);
    encoder.setReferencePulseCount(100);
    EXPECT_EQ(encoder.getPulseCount(), -100);
    encoder.setReferencePulseCount(0.0);
    EXPECT_EQ(encoder.getPulseCount(), 0.0);
}
