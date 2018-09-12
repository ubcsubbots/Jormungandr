/*
 * Created By: Cameron Newton
 * Created On: Sept 12nd, 2018
 * Description: A node that remaps the output of the decision node from an Odom
 * message
 *              to a TwistStamped for the simulator
 */

#include "OdomToTwistStamped.h"

OdomToTwistStamped::OdomToTwistStamped(int argc,
                                       char** argv,
                                       std::string node_name) {
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    std::string subscribe_topic = "/decision_node/output";
    std::string publish_topic   = private_nh.resolveName("output");

    subscriber_ = nh.subscribe(
    subscribe_topic, 10, &OdomToTwistStamped::odomMessageCallback, this);
    publisher_ = nh.advertise<geometry_msgs::TwistStamped>(publish_topic, 1000);
}

void OdomToTwistStamped::odomMessageCallback(nav_msgs::Odometry odomMsg) {
    geometry_msgs::TwistStamped outMsg;

    outMsg.twist.linear.x  = odomMsg.twist.twist.linear.x;
    outMsg.twist.linear.y  = odomMsg.twist.twist.linear.y;
    outMsg.twist.angular.z = odomMsg.twist.twist.angular.z;

    // Linear z speed in UWSim is backwards, up is negative down is positive
    if (odomMsg.pose.pose.position.z > 0)
        outMsg.twist.linear.z = -0.5;
    else if (odomMsg.pose.pose.position.z < 0)
        outMsg.twist.linear.z = 0.5;

    publisher_.publish(outMsg);
}
