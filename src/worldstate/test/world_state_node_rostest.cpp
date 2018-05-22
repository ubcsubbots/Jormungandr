//
// Created by joel on 05/05/18.
//

#include "WorldStateNode.h"
#include <gate_detect/gateDetectMsg.h>
#include <gtest/gtest.h>
#include <constants.h>
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
    int message_output;

  public:
    void callback(const worldstate::stateMsg::ConstPtr& msg) {
        message_output = msg->state;
    }
};

TEST_F(WorldStateNodeTest, worldStateNode_locatingGate_test) {
    gate_detect::gateDetectMsg data;

    data.detectRight = false;
    data.detectLeft = false;
    data.detectTop = false;
    data.distanceRight = 0.0;
    data.distanceLeft = 0.0;
    data.distanceTop = 0.0;
    data.angleLeft = 0.0;
    data.angleRight = 0.0;
    data.angleTop = 0.0;

    test_publisher.publish(data);

    ros::Rate loop_rate(1);
    loop_rate.sleep();

    ros::spinOnce();

    worldstate::stateMsg buf;
    buf.state = worldstate::stateMsg_<u_int8_t>::locatingGate;

    EXPECT_EQ(buf.state, message_output);
}

TEST_F(WorldStateNodeTest, worldStateNode_transition_state_test) {
    gate_detect::gateDetectMsg data;

    data.detectRight   = true;
    data.detectLeft    = true;
    data.detectTop     = false;
    data.distanceRight = 150.2;
    data.distanceLeft  = 150.2;
    data.distanceTop   = 0.0;
    data.angleRight    = 30;
    data.angleLeft     = 30;
    data.angleTop      = 0.0;

    test_publisher.publish(data);

    ros::Rate loop_rate(1);
    loop_rate.sleep();

    ros::spinOnce();

    worldstate::stateMsg buf;
    buf.state = worldstate::stateMsg_<u_int8_t>::aligningWithGate;

//    EXPECT_EQ(buf.state, message_output);
    /*

    data.detectRight   = true;
    data.detectLeft    = false;
    data.detectTop     = false;
    data.distanceRight = 150.2;
    data.distanceLeft  = 0.0;
    data.distanceTop   = 0.0;
    data.angleLeft     = 0.0;
    data.angleRight    = 0.0;
    data.angleTop      = 0.0;

    test_publisher.publish(data);

    loop_rate.sleep();
    ros::spinOnce();
    buf.state = worldstate::stateMsg_<u_int8_t>::aligningWithGate;

    EXPECT_EQ(buf.state, message_output);

    data.detectRight   = true;
    data.detectLeft    = true;
    data.detectTop     = true;
    data.distanceRight = 150.2;
    data.distanceLeft  = 150.2;
    data.distanceTop   = subbots::global_constants::CLEARANCE_HEIGHT;
    data.angleLeft     = 0.0;
    data.angleRight    = 0.0;
    data.angleTop      = 0.0;

    test_publisher.publish(data);

    loop_rate.sleep();
    ros::spinOnce();
    buf.state = worldstate::stateMsg_<u_int8_t>::aligningWithGate;

    EXPECT_EQ(buf.state, message_output);

    data.detectRight   = true;
    data.detectLeft    = true;
    data.detectTop     = true;
    data.distanceRight = 150.2;
    data.distanceLeft  = 150.2;
    data.distanceTop   = 60;
    data.angleLeft     = 0.0;
    data.angleRight    = 0.0;
    data.angleTop      = 0.0;

    test_publisher.publish(data);

    loop_rate.sleep();
    ros::spinOnce();
    buf.state = worldstate::stateMsg_<u_int8_t>::passingGate;

    EXPECT_EQ(buf.state, message_output);*/
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "world_state_node_rostest");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
