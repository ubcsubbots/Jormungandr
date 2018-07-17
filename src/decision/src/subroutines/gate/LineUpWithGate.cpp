/*
 * Created By: Reid Oliveira
 * Created On: March 24, 2018
 * Description: Subroutine that tries to position the sub in front and
 * orthogonal with the gate, ready to go through.
 */

#include "LineUpWithGate.h"
#include "constants.h"

std::vector<ros::Subscriber>
LineUpWithGate::getSubscriptions(ros::NodeHandle nh) {
    std::vector<ros::Subscriber> subs;
    subs.push_back(
    nh.subscribe("gate_location", 10, &LineUpWithGate::decisionCallback, this));
    return subs;
}

void LineUpWithGate::decisionCallback(
const gate_detect::GateDetectMsg::ConstPtr& msg) {
    // logic: given the location of the poles, try to put ourselves centred in
    // front

    // send the message
    geometry_msgs::TwistStamped command;

    double x_linear  = 0.0;
    double y_linear  = 0.0;
    double z_linear  = 0.0;
    double x_angular = 0.0;
    double y_angular = 0.0;
    double z_angular = 0.0;

    // If we're at an acceptable distance away from the top pole, adjust for
    // clearance
    if (msg->detectedTopPole &&
        abs(msg->distanceTopPole -
            subbots::global_constants::TARGET_TOP_POLE_DISTANCE) <
        subbots::global_constants::ERROR_TOLERANCE_TOP_POLE_DISTANCE) {
        float top_pole_clearance =
        sin(msg->angleTopPole) * msg->distanceTopPole;
        if ((top_pole_clearance -
             subbots::global_constants::TARGET_TOP_POLE_CLEARANCE) >
            subbots::global_constants::ERROR_TOLERANCE_TOP_POLE_CLEARANCE) {
            command.twist.linear.z = DOWN;
            publishCommand(command);
            return;
        } else if ((top_pole_clearance -
                    subbots::global_constants::TARGET_TOP_POLE_CLEARANCE) <
                   subbots::global_constants::
                   ERROR_TOLERANCE_TOP_POLE_CLEARANCE) {
            command.twist.linear.z = UP;
            publishCommand(command);
            return;
        }
    }

    // Make sure we're pointing at the middle of the gate
    if ((msg->angleRightPole + msg->angleLeftPole) >
        subbots::global_constants::ERROR_TOLERANCE_SIDE_POLES_ANGLE) {
        command.twist.angular.z = RIGHT;
    } else if ((msg->angleRightPole + msg->angleLeftPole) <
               subbots::global_constants::ERROR_TOLERANCE_SIDE_POLES_ANGLE) {
        command.twist.angular.z = LEFT;
    }

    // Get withing a good passing distance for gate
    float averageDistanceToGate =
    (msg->distanceLeftPole + msg->distanceRightPole + msg->distanceTopPole) /
    (msg->detectedLeftPole + msg->detectedRightPole + msg->detectedTopPole);
    if ((averageDistanceToGate -
         subbots::global_constants::TARGET_SIDE_POLES_DISTANCE) <
        -subbots::global_constants::ERROR_TOLERANCE_SIDE_POLES_DISTANCE) {
        command.twist.linear.x = BACKWARD;
    } else if ((averageDistanceToGate -
                subbots::global_constants::TARGET_SIDE_POLES_DISTANCE) >
               subbots::global_constants::ERROR_TOLERANCE_SIDE_POLES_DISTANCE) {
        command.twist.linear.x = FORWARD;
    }

    // Position ourselves laterally in front of the gate
    if ((msg->distanceRightPole - msg->distanceLeftPole) >
        subbots::global_constants::ERROR_TOLERANCE_SIDE_POLES_DISTANCE) {
        command.twist.linear.y = LEFT;
    } else if ((msg->distanceRightPole - msg->distanceLeftPole) <
               -subbots::global_constants::
               ERROR_TOLERANCE_SIDE_POLES_DISTANCE) {
        command.twist.linear.y = RIGHT;
    }

    publishCommand(command);
    // publishCommand(command);
}
