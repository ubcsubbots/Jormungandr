/*
 * Created By: Cameron Newton
 * Created On: Sept 12nd, 2018
 * Description: Test functions for the odom_to_twist_stamped node
 */

// GTest Includes
#include <gtest/gtest.h>

// ROS Includes
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

class OdomToTwistStampedTest : public testing::Test {
  protected:
    virtual void SetUp() {
        test_publisher =
        nh_.advertise<nav_msgs::Odometry>("/decision_node/output", 1000);
        test_subscriber = nh_.subscribe("/odom_to_twist_stamped/output",
                                        1,
                                        &OdomToTwistStampedTest::callback,
                                        this);

        // Let the publishers and subscribers set itself up timely
        ros::Rate loop_rate(1);
        loop_rate.sleep();
    }

    ros::NodeHandle nh_;
    ros::Publisher test_publisher;
    ros::Subscriber test_subscriber;
    geometry_msgs::TwistStamped message_output;

  public:
    void callback(const geometry_msgs::TwistStamped outMsg) {
        message_output = outMsg;
    }

    // Compare two messages
    bool compareMsgs(const geometry_msgs::TwistStamped& msg1,
                     const geometry_msgs::TwistStamped& msg2) {
        return (msg1.twist.linear.x == msg2.twist.linear.x &&
                msg1.twist.linear.y == msg2.twist.linear.y &&
                msg1.twist.linear.z == msg2.twist.linear.z &&
                msg1.twist.angular.x == msg2.twist.angular.x &&
                msg1.twist.angular.y == msg2.twist.angular.y &&
                msg1.twist.angular.z == msg2.twist.angular.z);
    }
};

// Reformat an up message
TEST_F(OdomToTwistStampedTest, up_message) {
    nav_msgs::Odometry out_message;
    out_message.twist.twist.linear.x  = 1.0;
    out_message.twist.twist.linear.y  = 1.0;
    out_message.twist.twist.angular.z = 1.0;
    out_message.pose.pose.position.z  = 1.0;
    test_publisher.publish(out_message);
    ros::Rate loop_rate(1);
    loop_rate.sleep();
    ros::spinOnce();

    geometry_msgs::TwistStamped up_message;
    up_message.twist.linear.x = 1.0;
    up_message.twist.linear.y = 1.0;
    up_message.twist.linear.z =
    -0.5; // Default up speed, must change if default speed changes in node
    up_message.twist.angular.z = 1.0;

    EXPECT_TRUE(compareMsgs(up_message, message_output));
}

// Reformat a down message
TEST_F(OdomToTwistStampedTest, down_message) {
    nav_msgs::Odometry out_message;
    out_message.twist.twist.linear.x  = -1.0;
    out_message.twist.twist.linear.y  = -1.0;
    out_message.twist.twist.angular.z = -1.0;
    out_message.pose.pose.position.z  = -1.0;
    test_publisher.publish(out_message);
    ros::Rate loop_rate(1);
    loop_rate.sleep();
    ros::spinOnce();

    geometry_msgs::TwistStamped down_message;
    down_message.twist.linear.x = -1.0;
    down_message.twist.linear.y = -1.0;
    down_message.twist.linear.z =
    0.5; // Default up speed, must change if default speed changes in node
    down_message.twist.angular.z = -1.0;

    EXPECT_TRUE(compareMsgs(down_message, message_output));
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "odom_to_twist_stamped");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
