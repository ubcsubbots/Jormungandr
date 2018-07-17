/*
 * Created By: Reid Oliveira
 * Created On: May 19, 2018
 * Description: Subroutine that simply proceeds forward through the gate
 */

#include "ApproachGate.h"
#include <constants.h>

std::vector<ros::Subscriber>
ApproachGate::getSubscriptions(ros::NodeHandle nh) {
    std::vector<ros::Subscriber> subs;
    subs.push_back(
    nh.subscribe("gate_location", 10, &ApproachGate::decisionCallback, this));
    return subs;
}

void ApproachGate::decisionCallback(
const gate_detect::GateDetectMsg::ConstPtr& msg) {
    geometry_msgs::TwistStamped command;

    command.twist.linear.x = FORWARD;

    // Attempt to point to the middle of the gate
    if ((msg->angleLeftPole + msg->angleRightPole) >
        (subbots::global_constants::ERROR_TOLERANCE_SIDE_POLES_ANGLE)) {
        command.twist.angular.z = RIGHT / 2;
    } else if ((msg->angleLeftPole + msg->angleRightPole) <
               (subbots::global_constants::ERROR_TOLERANCE_SIDE_POLES_ANGLE)) {
        command.twist.angular.z = LEFT / 2;
    }

    // Check top clearance for acceptable
    float top_pole_clearance = sin(msg->angleTopPole) * msg->distanceTopPole;

    if ((top_pole_clearance -
         subbots::global_constants::TARGET_TOP_POLE_CLEARANCE) >
        subbots::global_constants::ERROR_TOLERANCE_TOP_POLE_CLEARANCE) {
        command.twist.linear.z = DOWN / 2;
    } else if ((top_pole_clearance -
                subbots::global_constants::TARGET_TOP_POLE_CLEARANCE) <
               -subbots::global_constants::ERROR_TOLERANCE_TOP_POLE_CLEARANCE) {
        command.twist.linear.z = UP / 2;
    }

    publishCommand(command);
}