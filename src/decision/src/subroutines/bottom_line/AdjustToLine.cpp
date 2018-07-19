//
// Created by drdc-s2632 on 17/07/18.
//

#include "AdjustToLine.h"

std::vector<ros::Subscriber>
AdjustToLine::getSubscriptions(ros::NodeHandle nh) {
    std::string gateDetectTopic = "line_detect_output";

    std::vector<ros::Subscriber> subs;
    subs.push_back(nh.subscribe(
            gateDetectTopic, 10, &AdjustToLine::lineDetectCallback, this));
    return subs;
}

void AdjustToLine::lineDetectCallback(const line_detect::LineDetectMsg::ConstPtr& msg) {
    nav_msgs::Odometry command;

    if(msg->angleToParallelFrontMarker > constants_["ERROR_TOLERANCE_LINE_ANGLE"]){
        command.twist.twist.angular.z = TWISTLEFT;
        publishCommand(command);
        return;
    }else if(msg->angleToParallelFrontMarker < -constants_["ERROR_TOLERANCE_LINE_ANGLE"]){
        command.twist.twist.angular.z = TWISTRIGHT;
        publishCommand(command);
        return;
    }

    if(msg->lateralDistanceFromFrontMarker > constants_["ERROR_TOLERANCE_LINE_LATERAL_DISTANCE"]){
        command.twist.twist.linear.y = RIGHT;
        publishCommand(command);
        return;
    }else if(msg->lateralDistanceFromFrontMarker < -constants_["ERROR_TOLERANCE_LINE_LATERAL_DISTANCE"]){
        command.twist.twist.linear.y = LEFT;
        publishCommand(command);
        return;
    }

}
