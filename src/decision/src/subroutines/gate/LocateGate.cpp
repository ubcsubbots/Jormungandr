/*
 * Created By: Reid Oliveira
 * Created On: March 17, 2018
 * Description: Subroutine that attempts to position the gate so that it can be
 * seen by the camera.
 */

#include "LocateGate.h"

void LocateGate::setupSubscriptions(ros::NodeHandle nh) {
    nh.subscribe("gate_location", 10, &LocateGate::decisionCallback, this);
}

void LocateGate::decisionCallback(
const gate_detect::gateDetectMsg::ConstPtr& msg) {
    // logic: rotate on z to attempt to make the gate in view
    double z_rotation;

    if (msg->detectLeft && !msg->detectRight) {
        z_rotation = RIGHT;
    } else if (msg->detectRight && !msg->detectLeft) {
        z_rotation = LEFT;
    } else {
        z_rotation = RIGHT * 2;
    }

    geometry_msgs::Twist command;
    command.angular = makeVector(0.0, 0.0, z_rotation);
    command.linear  = makeVector(0.0, 0.0, 0.0);
    publishCommand(command);
}