/*
 * Created By: Reid Oliveira
 * Created On: May 19, 2018
 * Description: Subroutine that simply proceeds forward through the gate
 */

#include "GoThroughGate.h"

std::vector<ros::Subscriber>
GoThroughGate::getSubscriptions(ros::NodeHandle nh) {
    std::vector<ros::Subscriber> subs;
    subs.push_back(
    nh.subscribe("gate_location", 10, &GoThroughGate::decisionCallback, this));
    return subs;
}

void GoThroughGate::decisionCallback(
const gate_detect::GateDetectMsg::ConstPtr& msg) {
    // logic: just go forward
    double x_linear = FORWARD;

    nav_msgs::Odometry command;
    command.twist.twist.angular = makeVector(0.0, 0.0, 0.0);
    command.twist.twist.linear  = makeVector(x_linear, 0.0, 0.0);
    publishCommand(command);
}