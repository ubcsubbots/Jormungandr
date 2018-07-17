//
// Created by drdc-s2632 on 17/07/18.
//

#include "FindLine.h"

std::vector<ros::Subscriber>
FindLine::getSubscriptions(ros::NodeHandle nh) {
    std::string gateDetectTopic = "/line_detect/line_detect_output";
    
    timer_ = nh.createTimer(ros::Duration(10),&FindLine::timerCallback, this ,false);
    timer_.start();

    lateralVelocityDirection_ = 1;
    
    std::vector<ros::Subscriber> subs;
    
    subs.push_back(nh.subscribe(
            gateDetectTopic, 10, &FindLine::lineDetectCallback, this));
    return subs;
}

void FindLine::lineDetectCallback(const line_detect::LineDetectMsg::ConstPtr& msg){
    nav_msgs::Odometry command;

    command.twist.twist.linear.y = lateralVelocityDirection_ * RIGHT;
}

void FindLine::timerCallback(const ros::TimerEvent& event){
    lateralVelocityDirection_ = -lateralVelocityDirection_;
}

