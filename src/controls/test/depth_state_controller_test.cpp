/*
 * Created By: Logan Fillo
 * Created On: October 13 2019
 * Description: Integration testing for DepthStateController
 */

#define private public // hack to gain access to controller private namespace
#include <controllers/depth_state_controller.h>
#include <ros/ros.h>
#include <gtest/gtest.h>

/* class containing helper functions for integration tests */
class DepthStateFixture : public testing::Test 
{
    protected:
        virtual void initFixture() 
        {
            sub_ = nh_.subscribe("depth_sensor", 1, &DepthStateFixture::subCB, this);
            last_message_ = 0.0;
            ros::Rate loop_rate(1);
            loop_rate.sleep();

        }

        virtual void spinCallbacks(int num)
        {
            ros::Rate loop_rate(num);
            loop_rate.sleep();
            ros::spinOnce();
        }

        ros::NodeHandle nh_;
        ros::Subscriber sub_;
        double last_message_;

    public:
        void subCB(const std_msgs::Float64::ConstPtr& msg) {
            last_message_ = msg->data;
        }
};

TEST_F(DepthStateFixture,testSingleUpdatePublish) {
    // Setup
    initFixture(); 
    sensor_controllers::DepthStateController controller;
    hardware_interface::DepthStateInterface hw;
    ros::NodeHandle nh;
    double dummy;
    hardware_interface::DepthStateHandle handle("depth_sensor", &dummy);
    hw.registerHandle(handle);
    nh.setParam("publish_rate", 1);
    controller.init(&hw, nh, nh);
    double expected = 10.0;

    // Make the controller publish a message in update
    controller.last_publish_time_ = ros::Time(0.0);
    controller.depth_sensor_.setState(expected);
    controller.update(ros::Time(1.02), ros::Duration(0.0)); 

    // Spin for callback
    ros::Rate loop_rate(1);
    loop_rate.sleep();
    ros::spinOnce();
    EXPECT_EQ(last_message_, expected);
}

TEST_F(DepthStateFixture, /* test case */) {
    sensor_controllers::DepthStateController controller;
    EXPECT_TRUE(true);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "depth_state_controller_rostest");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}