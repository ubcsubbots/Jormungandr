/*
 * Created By: Gareth Ellis
 * Created On: July 22nd, 2018
 * Description: Tests for the `dead_mans_switch` node
 */

// GTest Includes
#include <gtest/gtest.h>

// ROS Includes
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Duration.h>


class DeadMansSwitchNodeTest : public testing::Test {
  protected:
    virtual void SetUp() {
        message_output = {};
        num_messages_received = 0;

        test_publisher = nh_.advertise<std_msgs::Empty>("/dead_mans_switch/is_alive_input", 1);
        test_subscriber = nh_.subscribe(
        "/dead_mans_switch/time_since_last_is_alive", 1, &DeadMansSwitchNodeTest::callback, this);

        // Let the publishers and subscribers set itself up timely
        ros::Rate loop_rate(1);
        loop_rate.sleep();
    }

    ros::NodeHandle nh_;
    ros::Publisher test_publisher;
    ros::Subscriber test_subscriber;
    ros::Duration message_output;

    int num_messages_received;


  public:
    void callback(const std_msgs::Duration msg) { num_messages_received++;message_output = msg.data; }
};

// The node should not publish anything until it receives the first message
TEST_F(DeadMansSwitchNodeTest, node_startup){
   // Don't publish anything for a while, and make sure that the node
   // doesn't publish anything
   ros::Duration(4).sleep();
   ros::Rate loop_rate(1);
   loop_rate.sleep();
   ros::spinOnce();
   EXPECT_EQ(0, num_messages_received);
}

// Check that the node gives a reasonably accurate measure of
// how long it's been since it last received message
TEST_F(DeadMansSwitchNodeTest, normal_operation) {
    test_publisher.publish(std_msgs::Empty());
    ros::Duration(5).sleep();
    ros::spinOnce();
    EXPECT_NEAR(5, message_output.toSec(), 0.07);

    test_publisher.publish(std_msgs::Empty());
    ros::Duration(0.2).sleep();
    ros::spinOnce();
    EXPECT_NEAR(0.2, message_output.toSec(), 0.07);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "dead_mans_switch_rostest");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
