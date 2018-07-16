/*
 * Created By: Reid Oliveira
 * Created On: March 24, 2018
 * Description: Subroutine that tries to position the sub in front and
 * orthogonal with the gate, ready to go through.
 */

#include "LineUpWithGate.h"
#include "constants.h"

void LineUpWithGate::setupSubscriptions(ros::NodeHandle nh) {
    subscriber_ = nh.subscribe(
    "gateDetect/output", 10, &LineUpWithGate::decisionCallback, this);
    // nh.subscribe("imu", 10, &LineUpWithGate::balance, this);
}

void LineUpWithGate::balance(const geometry_msgs::Twist::ConstPtr& msg) {
    // if not parallel with ground, become parallel with ground
}

void LineUpWithGate::sleep() {
    publisher_.shutdown();
    subscriber_.shutdown();
}

void LineUpWithGate::decisionCallback(
const gate_detect::GateDetectMsg::ConstPtr& msg) {
    // logic: given the location of the poles, try to put ourselves centred in
    // front

    // send the message
    geometry_msgs::TwistStamped command;

    double x_linear = 0.0;
    double y_linear = 0.0;
    double z_linear = 0.0;
    double x_angular = 0.0;
    double y_angular = 0.0;
    double z_angular = 0.0;

    if (msg->detectedTopPole) {
        if (abs(msg->distanceTopPole - subbots::global_constants::TARGET_TOP_POLE_DISTANCE) <
            subbots::global_constants::ERROR_TOLERANCE_TOP_POLE_DISTANCE) {
        if ((msg->angleTopPole - subbots::global_constants::TARGET_TOP_POLE_ANGLE) >
            subbots::global_constants::ERROR_TOLERANCE_TOP_POLE_ANGLE) {
            command.twist.linear.z = UP;
            publishCommand(command);
            return;
        } else if ((msg->angleTopPole - subbots::global_constants::TARGET_TOP_POLE_ANGLE) <
                   subbots::global_constants::ERROR_TOLERANCE_TOP_POLE_ANGLE) {
            command.twist.linear.z = DOWN;
            publishCommand(command);
            return;
        }
        }
    }

    float averageDistanceToGate = (msg->distanceLeftPole + msg->distanceRightPole + msg->distanceTopPole) /
                                  (msg->detectedLeftPole + msg->detectedRightPole + msg->detectedTopPole);


    if ((msg->angleRightPole + msg->angleLeftPole) > subbots::global_constants::ERROR_TOLERANCE_SIDE_POLES_ANGLE) {
        command.twist.angular.z = RIGHT;
    } else if ((msg->angleRightPole + msg->angleLeftPole) <
               subbots::global_constants::ERROR_TOLERANCE_SIDE_POLES_ANGLE) {
        command.twist.angular.z = LEFT;
    }

    if ((averageDistanceToGate - subbots::global_constants::TARGET_SIDE_POLES_DISTANCE) < -subbots::global_constants::ERROR_TOLERANCE_SIDE_POLES_DISTANCE){
        command.twist.linear.x = BACKWARD;
    } else if((averageDistanceToGate - subbots::global_constants::TARGET_SIDE_POLES_DISTANCE) > subbots::global_constants::ERROR_TOLERANCE_SIDE_POLES_DISTANCE){
        command.twist.linear.x = FORWARD;
    }

    if((msg->distanceRightPole - msg->distanceLeftPole) > subbots::global_constants::ERROR_TOLERANCE_SIDE_POLES_DISTANCE){
        command.twist.linear.y = RIGHT;
    } else if((msg->distanceRightPole - msg->distanceLeftPole) < -subbots::global_constants::ERROR_TOLERANCE_SIDE_POLES_DISTANCE){
        command.twist.linear.y = LEFT;
    }

    publisher_.publish(command);
    // publishCommand(command);
}
