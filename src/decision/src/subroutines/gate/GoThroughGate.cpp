/*
 * Created By: Reid Oliveira
 * Created On: May 19, 2018
 * Description: Subroutine that simply proceeds forward through the gate
 */

#include "GoThroughGate.h"

void GoThroughGate::setupSubscriptions(ros::NodeHandle nh) {
    nh.subscribe("gate_location", 10, &GoThroughGate::decisionCallback, this);
}

void GoThroughGate::decisionCallback(
const gate_detect::GateDetectMsg::ConstPtr& msg) {
    // logic: just go forward
    double x_linear = FORWARD;

    geometry_msgs::Twist command;
    command.angular = makeVector(0.0, 0.0, 0.0);
    command.linear  = makeVector(x_linear, 0.0, 0.0);
    publishCommand(command);
}