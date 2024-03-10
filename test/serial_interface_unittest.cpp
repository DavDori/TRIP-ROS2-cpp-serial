#include <pty.h>
#include <gtest/gtest.h>
#include "trip_interface/serial_interface.h"
#include <memory>

class SerialInterfaceTests : public ::testing::Test 
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

        device_ = std::make_unique<SerialInterface>(std::string(name_), 9600);
    }

    virtual void TearDown() 
    {
        device_->disconnect();
    }

    std::unique_ptr<SerialInterface> device_;
    int master_;
    int slave_;
    char name_[100];
};

TEST_F(SerialInterfaceTests, ReadLine) 
{
    write(master_, "Hello world!\n", 13);
    std::string msg = device_->readLine();
    EXPECT_EQ(msg, std::string("Hello world!"));
}

TEST_F(SerialInterfaceTests, LastMessage) 
{
    write(master_, "Hello world!\n", 13);
    device_->readLine();
    std::string msg2 = device_->getLastMessage();
    EXPECT_EQ(msg2, std::string("Hello world!"));
}

TEST_F(SerialInterfaceTests, SendMessage) 
{
    char buffer[3] = "";
    device_->send("Hi!");
    read(master_, buffer, 3);
    EXPECT_EQ(std::string(buffer, 3), std::string("Hi!"));
}

