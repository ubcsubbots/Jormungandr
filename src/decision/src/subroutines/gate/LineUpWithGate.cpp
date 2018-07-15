/*
 * Created By: Reid Oliveira
 * Created On: March 24, 2018
 * Description: Subroutine that tries to position the sub in front and
 * orthogonal with the gate, ready to go through.
 */

#include "LineUpWithGate.h"

std::vector<ros::Subscriber>
LineUpWithGate::getSubscriptions(ros::NodeHandle nh) {
    std::vector<ros::Subscriber> subs;
    subs.push_back(
    nh.subscribe("gate_location", 10, &LineUpWithGate::decisionCallback, this));
    return subs;
}

void LineUpWithGate::decisionCallback(
const gate_detect::gateDetectMsg::ConstPtr& msg) {
    // logic: given the location of the poles, try to put ourselves centred in
    // front

    double x_linear  = 0.0;
    double y_linear  = 0.0;
    double z_linear  = 0.0;
    double x_angular = 0.0;
    double y_angular = 0.0;
    double z_angular = 0.0;

    geometry_msgs::Twist command;

    if (!distance_to_gate_acceptable_) {
        double averageDistanceToGate;

        averageDistanceToGate =
        (msg->distanceLeft + msg->distanceRight + msg->distanceTop) /
        (msg->detectLeft + msg->detectRight + msg->detectTop);

        if (averageDistanceToGate > 7) {
            if (msg->angleTop > 0.25) {
                command.linear.z = DOWN;
            } else if (msg->angleTop < -0.25) {
                command.linear.z = UP;
            }
            if ((msg->angleLeft + msg->angleRight) > 0.25) {
                command.angular.z = -0.25;
            } else if ((msg->angleLeft + msg->angleRight) < -0.25) {
                command.angular.z = 0.25;
            } else {
                command.linear.x = FORWARD;
            }
        } else {
            distance_to_gate_acceptable_ = true;
        }
    }

    if (!align_top_) {
        if (msg->angleTop > 0.25) {
            command.linear.z = DOWN;
            publishCommand(command);
            return;
        } else if (msg->angleTop < -0.25) {
            command.linear.z = UP;
            publishCommand(command);
            return;
        } else {
            align_top_ = true;
        }
    }

    if (msg->angleTop > 0.25) {
        command.linear.z = DOWN;
    } else if (msg->angleTop < -0.25) {
        command.linear.z = UP;
    }
    if ((msg->angleLeft + msg->angleRight) > 0.25) {
        command.angular.z = 0.25;
    } else if ((msg->angleLeft + msg->angleRight) < -0.25) {
        command.angular.z = -0.25;
    }

    publishCommand(command);
}
