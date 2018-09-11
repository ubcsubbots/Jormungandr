/*
 * Created By: Cameron Newton
 * Created On: July 19, 2018
 * Description: Subroutine that adjusts the depth of the robot
 */

#include "AdjustDepth.h"
#include "constants.h"

std::vector<ros::Subscriber> AdjustDepth::getSubscriptions(ros::NodeHandle nh) {
    std::vector<ros::Subscriber> subs;
    subs.push_back(
    nh.subscribe("gate_location", 10, &AdjustDepth::decisionCallback, this));
    return subs;
}

void AdjustDepth::decisionCallback(
const gate_detect::GateDetectMsg::ConstPtr& msg) {
    nav_msgs::Odometry command;

    command.twist.twist.linear.z = DOWN;

    publishCommand(command);
}