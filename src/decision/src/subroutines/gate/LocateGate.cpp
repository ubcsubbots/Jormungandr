/*
 * Created By: Reid Oliveira
 * Created On: March 17, 2018
 * Description: Subroutine that attempts to position the gate so that it can be
 * seen by the camera.
 */

#include "LocateGate.h"

std::vector<ros::Subscriber> LocateGate::getSubscriptions(ros::NodeHandle nh) {
    std::vector<ros::Subscriber> subs;
    subs.push_back(
    nh.subscribe("gate_location", 10, &LocateGate::decisionCallback, this));
    return subs;
}

void LocateGate::decisionCallback(
const gate_detect::GateDetectMsg::ConstPtr& msg) {
    // logic: rotate on z to attempt to make the gate in view
    double z_rotation;

    if (msg->detectedLeftPole && !msg->detectedRightPole) {
        z_rotation = RIGHT;
    } else if (msg->detectedRightPole && !msg->detectedLeftPole) {
        z_rotation = LEFT;
    } else {
        z_rotation = RIGHT * 2;
    }

    geometry_msgs::TwistStamped command;
    command.twist.angular = makeVector(0.0, 0.0, z_rotation);
    command.twist.linear  = makeVector(0.0, 0.0, 0.0);
    publishCommand(command);
}