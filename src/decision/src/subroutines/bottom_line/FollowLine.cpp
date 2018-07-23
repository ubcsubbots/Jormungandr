//
// Created by drdc-s2632 on 17/07/18.
//

#include "FollowLine.h"

std::vector<ros::Subscriber> FollowLine::getSubscriptions(ros::NodeHandle nh) {
    std::string gateDetectTopic = "line_detect_output";

    std::vector<ros::Subscriber> subs;
    subs.push_back(
    nh.subscribe(gateDetectTopic, 10, &FollowLine::lineDetectCallback, this));
    return subs;
}

void FollowLine::lineDetectCallback(
const line_detect::LineDetectMsg::ConstPtr& msg) {
    geometry_msgs::TwistStamped command;

    command.twist.linear.x = FORWARD;

    publishCommand(command);
}
