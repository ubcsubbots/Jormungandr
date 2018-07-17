//
// Created by drdc-s2632 on 17/07/18.
//

#include "AdjustToLine.h"
#include <worldstate/StateMsg.h>

std::vector<ros::Subscriber>
AdjustToLine::getNodeSubscriptions(ros::NodeHandle nh) {
    std::string gateDetectTopic = "/gateDetect/output";

    std::vector<ros::Subscriber> subs;
    subs.push_back(nh.subscribe(
            gateDetectTopic, 10, &AdjustToLine::lineDetectCallback, this));
    return subs;
}

void AdjustToLine::lineDetectCallback(const line_detect::LineDetectMsg::ConstPtr& msg) {
    worldstate::StateMsg stateMsg;

    stateMsg.state = worldstate::StateMsg::adjustingToLine;

    if (msg->lateralDistanceFromLine == -1.0f && msg->distanceFromEnd == -1.0f &&
            msg->angleToParallel == -1.0f){
            stateMsg.state = worldstate::StateMsg::findingLine;
    }
    
    if(msg->angleToParallel < subbots::global_constants::ERROR_TOLERANCE_LINE_ANGLE
            && msg->lateralDistanceFromLine < subbots::global_constants::ERROR_TOLERANCE_LINE_LATERAL_DISTANCE){
        stateMsg.state = worldstate::StateMsg::followingLine;
    } 

    publishNextState(stateMsg);
}
