/*
 * Created By: Reid Oliveira
 * Created On: March 24, 2018
 * Description:
 */

#include "LineUpWithGate.h"

void LineUpWithGate::setupSubscriptions(ros::NodeHandle nh) {

    nh.subscribe("gate_location", 10, &LineUpWithGate::decisionCallback, this);
}

void LineUpWithGate::decisionCallback(const gate_detect::gateDetectMsg::ConstPtr& msg) {

    // logic: given the location of the poles, try to put ourselves centred in front

    double x_linear = 0.0;
    double y_linear = 0.0;
    double z_linear = 0.0;
    double x_angular = 0.0;
    double y_angular = 0.0;
    double z_angular = 0.0;

    if (msg->distanceRight > msg->distanceLeft) {
        // move right
        y_linear = -1.0;
        z_angular = 1.0;
    } else {
        // move left
        y_linear = 1.0;
        z_angular = -1.0;
    }

    // send the message
    geometry_msgs::Twist command;
    command.angular = makeVector(x_angular, y_angular, z_angular);
    command.linear = makeVector(x_linear,y_linear,z_linear);
    publishCommand(command);

}
