/*
 *  Author: Joel Ahn
 *  Date:  31/03/18
 *  Purpose: Creates a dummy node to publish and subscribe to the relevant
 *           world_state_node topics to determine that the correct values
 *           are being outputted.
 */

#include "DecisionNode.h"

#include "LocateGate.h"
#include <gtest/gtest.h>
#include <std_msgs/String.h>

class DecisionNodeTest : public testing::Test {
  protected:
    virtual void SetUp() {
        test_publisher =
        nh_.advertise<worldstate::StateMsg>("/world_state_node/output", 1);
        test_subscriber = nh_.subscribe(
        "/decision_node/info", 1, &DecisionNodeTest::callback, this);

        // Let the publishers and subscribers set itself up timely
        ros::Rate loop_rate(1);
        loop_rate.sleep();
    }

    ros::NodeHandle nh_;
    ros::Publisher test_publisher;
    ros::Subscriber test_subscriber;
    std::string message_output = "";

  public:
    void callback(const std_msgs::String msg) { message_output = msg.data; }
};

TEST_F(DecisionNodeTest, decisionNode_transition_test) {
    worldstate::StateMsg msg;
    msg.state = msg.locatingGate;

    test_publisher.publish(msg);

    ros::Rate loop_rate(1);
    loop_rate.sleep();
    ros::spinOnce();

    std_msgs::String buf;
    buf.data = "LocateGate";

    EXPECT_EQ(buf.data, message_output);

    msg.state = msg.aligningWithGate;

    test_publisher.publish(msg);
    loop_rate.sleep();
    ros::spinOnce();
    buf.data = "LineUpWithGate";

    EXPECT_EQ(buf.data, message_output);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "decision_node_rostest");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
