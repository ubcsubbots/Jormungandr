//
// Created by joel on 05/05/18.
//

#include "WorldStateNode.h"
#include <gate_detect/gateDetectMsg.h>
#include <gtest/gtest.h>
#include <ros/ros.h>

class WorldStateNodeTest : public testing::Test {
  protected:
    virtual void SetUp() {
        test_publisher =
        nh_.advertise<gate_detect::gateDetectMsg>("/gateDetect/output", 1);
        test_subscriber = nh_.subscribe(
        "world_state/output", 1, &WorldStateNodeTest::callback, this);

        // Let the publishers and subscribers set itself up timely
        ros::Rate loop_rate(1);
        loop_rate.sleep();
    }

    ros::NodeHandle nh_;
    ros::Publisher test_publisher;
    ros::Subscriber test_subscriber;

  public:
    void callback(const worldstate::state_msg::ConstPtr& msg) {
        message_output = msg->state;
    }
};

TEST_F(WorldStateNodeTest, worldStateNode_locatingGate_Test) {
    gate_detect::gateDetectMsg data;

    data.detectRight   = false;
    data.detectLeft    = false;
    data.detectTop     = false;
    data.distanceRight = 0.0;
    data.distanceLeft  = 0.0;
    data.distanceTop   = 0.0;
    data.angleLeft     = 0.0;
    data.angleRight    = 0.0;
    data.angleTop      = 0.0;

    test_publisher.publish(data);

    ros::Rate loop_rate(1);
    loop_rate.sleep();

    ros::spinOnce();

    EXPECT_EQ(worldstate::state_msg::locatingGate, message_output);
}

int main(int argc, char** argv) {
    // !! Don't forget to initialize ROS, since this is a test within the ros
    // framework !!
    ros::init(argc, argv, "world_state_node_rostest");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
