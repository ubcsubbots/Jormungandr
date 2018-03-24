/*
 * Created By: Reid Oliveira
 * Created On: March 17, 2018
 * Description:
 */

#include "LocateGate.h"

void LocateGate::setupSubscriptions(ros::NodeHandle nh) {

    nh.subscribe("gate", 10, &LocateGate::decisionCallback, this);
}

void LocateGate::decisionCallback(const std_msgs::String::ConstPtr& msg) {

    // processing logic: rotate until we can see the whole gate

    // attempt to rotate until the whole gate is in view


    // send the message
    geometry_msgs::Twist command;
    command.angular = makeVector(0.0,0.0,0.0);
    command.linear = makeVector(0.0,0.0,0.0);
    publishCommand(command);

}