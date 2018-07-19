/*
 * Created By: Reid Oliveira
 * Created On: May 19, 2018
 * Description: Subroutine that simply proceeds forward through the gate
 */

#include "ApproachGate.h"
#include "constants.h"

std::vector<ros::Subscriber>
ApproachGate::getSubscriptions(ros::NodeHandle nh) {
    std::vector<ros::Subscriber> subs;
    subs.push_back(
    nh.subscribe("gate_location", 10, &ApproachGate::decisionCallback, this));
    return subs;
}

void ApproachGate::decisionCallback(
const gate_detect::GateDetectMsg::ConstPtr& msg) {
    nav_msgs::Odometry command;

    command.twist.twist.linear.x = FORWARD;

    // Attempt to point to the middle of the gate
    if ((msg->angleLeftPole + msg->angleRightPole) >
        constants_["ERROR_TOLERANCE_SIDE_POLES_ANGLE"]) {
        command.twist.twist.angular.z = RIGHT / 2;
    } else if ((msg->angleLeftPole + msg->angleRightPole) <
               constants_["ERROR_TOLERANCE_SIDE_POLES_ANGLE"]) {
        command.twist.twist.angular.z = LEFT / 2;
    }

    // Check top clearance for acceptable
    float top_pole_clearance = (float)sin(msg->angleTopPole) * msg->distanceTopPole;

    if (abs((int)(top_pole_clearance - constants_["TARGET_TOP_POLE_CLEARANCE"])) >
        constants_["ERROR_TOLERANCE_TOP_POLE_CLEARANCE"]) {
        command.twist.twist.linear.z = top_pole_clearance - constants_["TARGET_TOP_POLE_CLEARANCE"];
    }

    publishCommand(command);
}