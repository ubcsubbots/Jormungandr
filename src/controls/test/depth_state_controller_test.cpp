/*
 * Created By: Logan Fillo
 * Created On: October 13 2019
 * Description: Integration testing for DepthStateController
 */

#include <controllers/depth_state_controller.h>
#include <ros/ros.h>
#include <gtest/gtest.h>

class DepthStateFixture : public testing::Test 
{
    protected:
        /* data and methods for fixture */
    
};

TEST_F(DepthStateFixture, /* test case */) {
    sensor_controllers::DepthStateController controller;
    /* asserts and expect macros */
    EXPECT_TRUE(true);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "depth_state_controller_rostest");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}