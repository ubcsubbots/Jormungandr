//
// Created by drdc-s2632 on 17/07/18.
//

#include "FindLine.h"
#include <worldstate/StateMsg.h>

std::vector<ros::Subscriber>
FindLine::getNodeSubscriptions(ros::NodeHandle nh) {
    std::string gateDetectTopic = "/gateDetect/output";
    
    timer_ = nh.createTimer(ros::Duration(30),&FindLine::timerCallback, this ,false);
    timer_.start();
    
    std::vector<ros::Subscriber> subs;
    
    subs.push_back(nh.subscribe(
            gateDetectTopic, 10, &FindLine::lineDetectCallback, this));
    return subs;
}

void FindLine::lineDetectCallback(const line_detect::LineDetectMsg::ConstPtr& msg){
    worldstate::StateMsg stateMsg;

    stateMsg.state = worldstate::StateMsg::findingLine;
    
    if(!(msg->lateralDistanceFromLine == -1.0f || msg->distanceFromEnd == -1.0f || msg-> angleToParallel == -1.0f)){
        stateMsg.state = worldstate::StateMsg::adjustingToLine;
        timer_.stop();
    }
    
    publishNextState(stateMsg);
}

void FindLine::timerCallback(const ros::TimerEvent& event){
    worldstate::StateMsg stateMsg;

    stateMsg.state = worldstate::StateMsg::locatingDie;

    publishNextState(stateMsg);
}

