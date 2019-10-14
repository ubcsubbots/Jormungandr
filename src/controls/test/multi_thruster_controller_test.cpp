/*
 * Created By: Logan Fillo
 * Created On: October 13 2019
 * Description: Integration testing for MultiThrusterController
 */

#include <controllers/multi_thruster_controller.h>
#include <ros/ros.h>
#include <gtest/gtest.h>

class MultiThrusterFixture : public testing::Test 
{
    protected:
        /* data and methods for fixture */

};

TEST_F(MultiThrusterFixture, /* test case */) {
    thruster_controllers::MultiThrusterController controller;
    /* asserts and expect macros */
    EXPECT_TRUE(true);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "multi_thruster_controller_rostest");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}