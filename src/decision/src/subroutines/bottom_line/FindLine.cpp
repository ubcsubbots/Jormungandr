/*
* Created By: Cameron Newton
* Created On: July 15th, 2018
* Description: Subroutine to fine bottom marker
*/

#include "FindLine.h"

std::vector<ros::Subscriber> FindLine::getSubscriptions(ros::NodeHandle nh) {
    std::string gateDetectTopic = "line_detect_output";

    timer_ =
    nh.createTimer(ros::Duration(10), &FindLine::timerCallback, this, false);
    timer_.start();

    lateralVelocityDirection_ = 1;

    std::vector<ros::Subscriber> subs;

    subs.push_back(
    nh.subscribe(gateDetectTopic, 10, &FindLine::lineDetectCallback, this));
    return subs;
}

void FindLine::lineDetectCallback(
const line_detect::LineDetectMsg::ConstPtr& msg) {
    //Reverse direction if you don't find the line after a certain amount of time
    geometry_msgs::TwistStamped command;

    command.twist.linear.y = lateralVelocityDirection_ * RIGHT;

    publishCommand(command);
}

void FindLine::timerCallback(const ros::TimerEvent& event) {
    lateralVelocityDirection_ = -lateralVelocityDirection_;
}
