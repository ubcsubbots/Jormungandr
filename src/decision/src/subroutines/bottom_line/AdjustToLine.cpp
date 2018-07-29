/*
* Created By: Cameron Newton
* Created On: July 15th, 2018
* Description: Subroutine to adjust to bottom marker
*/

#include "AdjustToLine.h"

std::vector<ros::Subscriber>
AdjustToLine::getSubscriptions(ros::NodeHandle nh) {
    std::string gateDetectTopic = "line_detect_output";

    std::vector<ros::Subscriber> subs;
    subs.push_back(
    nh.subscribe(gateDetectTopic, 10, &AdjustToLine::lineDetectCallback, this));
    return subs;
}

void AdjustToLine::lineDetectCallback(
const line_detect::LineDetectMsg::ConstPtr& msg) {
    nav_msgs::Odometry command;

    // First adjust angle untill parallel with line, then check lateral
    // distance.

    // Check angle of line in front of robot, if angle is out of error tolerance
    // adjust accordingly
    if (msg->angleToParallelFrontMarker >
        (*constants_)["ERROR_TOLERANCE_LINE_ANGLE"]) {
        command.twist.twist.angular.z = TWISTRIGHT;
        publishCommand(command);
        return;
    } else if (msg->angleToParallelFrontMarker <
               -(*constants_)["ERROR_TOLERANCE_LINE_ANGLE"]) {
        command.twist.twist.angular.z = TWISTLEFT;
        publishCommand(command);
        return;
    }

    // Check if lateral distance to marker is out of error tolerance, if it is
    // adjust accordingly
    if (msg->lateralDistanceFromFrontMarker >
        (*constants_)["ERROR_TOLERANCE_LINE_LATERAL_DISTANCE"]) {
        command.twist.twist.linear.y = LEFT;
        publishCommand(command);
    } else if (msg->lateralDistanceFromFrontMarker <
               -(*constants_)["ERROR_TOLERANCE_LINE_LATERAL_DISTANCE"]) {
        command.twist.twist.linear.y = RIGHT;
        publishCommand(command);
    }
}
