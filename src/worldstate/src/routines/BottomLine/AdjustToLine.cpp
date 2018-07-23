//
// Created by drdc-s2632 on 17/07/18.
//

#include "AdjustToLine.h"
#include <worldstate/StateMsg.h>

std::vector<ros::Subscriber>
AdjustToLine::getNodeSubscriptions(ros::NodeHandle nh) {
    std::string line_detect_topic = "line_detect_output";

    std::vector<ros::Subscriber> subs;
    subs.push_back(nh.subscribe(
    line_detect_topic, 10, &AdjustToLine::lineDetectCallback, this));
    return subs;
}

void AdjustToLine::lineDetectCallback(
const line_detect::LineDetectMsg::ConstPtr& msg) {
    worldstate::StateMsg stateMsg;

    stateMsg.state = worldstate::StateMsg::adjustingToLine;

    if (msg->lateralDistanceFromFrontMarker == -1.0f &&
        msg->distanceFromEndOfFrontMarker == -1.0f &&
        msg->angleToParallelFrontMarker == -1.0f) {
        stateMsg.state = worldstate::StateMsg::findingLine;
    }

    if (msg->angleToParallelFrontMarker <
        (*constants_)["ERROR_TOLERANCE_LINE_ANGLE"] &&
        msg->lateralDistanceFromFrontMarker <
        (*constants_)["ERROR_TOLERANCE_LINE_LATERAL_DISTANCE"]) {
        stateMsg.state = worldstate::StateMsg::followingLine;
    }

    publishNextState(stateMsg);
}
