//
// Created by drdc-s2632 on 17/07/18.
//

#include "FollowLine.h"
#include <worldstate/StateMsg.h>

std::vector<ros::Subscriber>
FollowLine::getNodeSubscriptions(ros::NodeHandle nh) {
    std::string gateDetectTopic = "/gateDetect/output";

    timer_ = nh.createTimer(ros::Duration(10),&FollowLine::timerCallback, this ,false);
    timer_.start();

    std::vector<ros::Subscriber> subs;
    subs.push_back(nh.subscribe(
            gateDetectTopic, 10, &FollowLine::lineDetectCallback, this));
    return subs;
}

void FollowLine::lineDetectCallback(const line_detect::LineDetectMsg::ConstPtr& msg){
    worldstate::StateMsg stateMsg;

    stateMsg.state = worldstate::StateMsg::followingLine;

    if(!(msg->lateralDistanceFromLine == -1.0f || msg->distanceFromEnd == -1.0f || msg-> angleToParallel == -1.0f)){
       stateMsg.state = worldstate::StateMsg::adjustingToLine;
       timer_.stop();
    }

    if(msg->distanceFromEnd < subbots::global_constants::ERROR_TOLERANCE_LINE_DISTANCE_TO_END){
        stateMsg.state = worldstate::StateMsg::locatingDie;
        timer_.stop();
    }

    publishNextState(stateMsg);
}

void FollowLine::timerCallback(const ros::TimerEvent& event){
    worldstate::StateMsg stateMsg;

    stateMsg.state = worldstate::StateMsg::locatingDie;

    publishNextState(stateMsg);
}