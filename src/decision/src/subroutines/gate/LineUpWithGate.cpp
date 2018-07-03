/*
 * Created By: Reid Oliveira
 * Created On: March 24, 2018
 * Description: Subroutine that tries to position the sub in front and
 * orthogonal with the gate, ready to go through.
 */

#include "LineUpWithGate.h"
#include "constants.h"

void LineUpWithGate::setupSubscriptions(ros::NodeHandle nh) {
    nh.subscribe("gate_location", 10, &LineUpWithGate::decisionCallback, this);
    nh.subscribe("imu", 10, &LineUpWithGate::balance, this);
}

void LineUpWithGate::balance(const geometry_msgs::Twist::ConstPtr& msg) {
    // if not parallel with ground, become parallel with ground
}

void LineUpWithGate::decisionCallback(
const gate_detect::GateDetectMsg::ConstPtr& msg) {
    // logic: given the location of the poles, try to put ourselves centred in
    // front

    double x_linear  = 0.0;
    double y_linear  = 0.0;
    double z_linear  = 0.0;
    double x_angular = 0.0;
    double y_angular = 0.0;
    double z_angular = 0.0;

    // we should integrate IMU in here for info such as if parallel with ground,
    // etc.

    if (msg->distanceRightPole > msg->distanceLeftPole) {
        y_linear = RIGHT;
    } else {
        y_linear = LEFT;
    }

    if (!(std::abs(msg->distanceTopPole * sin(msg->angleTopPole)) >
          CLEARANCE_HEIGHT &&
          msg->angleTopPole < 0)) {
        z_linear = DOWN;
    }

    // send the message
    geometry_msgs::Twist command;
    command.angular = makeVector(x_angular, y_angular, z_angular);
    command.linear  = makeVector(x_linear, y_linear, z_linear);
    publishCommand(command);
}
