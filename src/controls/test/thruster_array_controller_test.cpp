/*
 * Created By: Logan Fillo
 * Created On: October 13 2019
 * Description: Integration testing for ThrusterArrayController
 */

#include <controllers/thruster_array_controller.h>
#include <ros/ros.h>
#include <gtest/gtest.h>

class ThrusterArrayFixture : public testing::Test 
{
    protected:
        /* data and methods for fixture */

};

TEST_F(ThrusterArrayFixture, /* test case */) {
    thruster_controllers::ThrusterArrayController controller;
    /* asserts and expect macros */
    EXPECT_TRUE(true);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "thruster_array_controller_rostest");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}