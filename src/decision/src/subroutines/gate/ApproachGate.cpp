/*
 * Created By: Reid Oliveira
 * Created On: May 19, 2018
 * Description: Subroutine that simply proceeds forward through the gate
 */

#include <constants.h>
#include "ApproachGate.h"

void ApproachGate::setupSubscriptions(ros::NodeHandle nh) {
    subscriber_ = nh.subscribe(
            "gateDetect/output", 10, &ApproachGate::decisionCallback, this);
}

void ApproachGate::sleep() {
    publisher_.shutdown();
    subscriber_.shutdown();
}

void ApproachGate::decisionCallback(
        const gate_detect::GateDetectMsg::ConstPtr& msg) {

    geometry_msgs::TwistStamped command;

    command.twist.linear.x = FORWARD;

    if ((msg->angleLeftPole + msg->angleRightPole) > (subbots::global_constants::ERROR_TOLERANCE_SIDE_POLES_ANGLE)) {
        command.twist.angular.z = RIGHT;
    } else if ((msg->angleLeftPole + msg->angleRightPole) < (subbots::global_constants::ERROR_TOLERANCE_SIDE_POLES_ANGLE)) {
        command.twist.angular.z = LEFT;
    }

    if ((msg->angleTopPole - subbots::global_constants::TARGET_TOP_POLE_ANGLE) > subbots::global_constants::TARGET_TOP_POLE_ANGLE) {
        command.twist.linear.z = UP;
    } else if ((msg->angleTopPole - subbots::global_constants::TARGET_TOP_POLE_ANGLE) < -subbots::global_constants::ERROR_TOLERANCE_TOP_POLE_ANGLE){
        command.twist.linear.z = DOWN;
    }

    publishCommand(command);
}