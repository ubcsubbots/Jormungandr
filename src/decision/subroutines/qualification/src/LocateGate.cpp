/*
 * Created By: Reid Oliveira
 * Created On: March 17, 2018
 * Description:
 */

#include "LocateGate.h"

void LocateGate::setupSubscriptions(ros::NodeHandle nh) {

    nh.subscribe("gate_location", 10, &LocateGate::decisionCallback, this);
}

void LocateGate::decisionCallback(const gate_detect::gateDetectMsg::ConstPtr& msg) {

    // logic: rotate on z to attempt to make the gate in view
    double z;

    if (msg->detectLeft && !msg->detectRight) {
        z = RIGHT;
    } else if (msg->detectRight && !msg->detectLeft) {
        z = LEFT;
    } else {
        z = RIGHT * 2;
    }

    geometry_msgs::Twist command;
    command.angular = makeVector(0.0,0.0,z);
    command.linear = makeVector(0.0,0.0,0.0);
    publishCommand(command);
}