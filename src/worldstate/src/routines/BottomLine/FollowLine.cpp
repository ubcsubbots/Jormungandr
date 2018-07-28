/*
* Created By: Cameron Newton
* Created On: July 15th, 2018
* Description: Subroutine to detect whether we can follow line
*/

#include "FollowLine.h"
#include <worldstate/StateMsg.h>

std::vector<ros::Subscriber>
FollowLine::getNodeSubscriptions(ros::NodeHandle nh) {
    std::string line_detect_topic = "line_detect_output";

    timer_ =
    nh.createTimer(ros::Duration(10), &FollowLine::timerCallback, this, false);
    timer_.start();

    std::vector<ros::Subscriber> subs;
    subs.push_back(
    nh.subscribe(line_detect_topic, 10, &FollowLine::lineDetectCallback, this));
    return subs;
}

void FollowLine::lineDetectCallback(
const line_detect::LineDetectMsg::ConstPtr& msg) {
    worldstate::StateMsg stateMsg;

    stateMsg.state = worldstate::StateMsg::followingLine;

    if (!(msg->lateralDistanceFromRearMarker == -1.0f ||
          msg->distanceFromEndRearMarker == -1.0f ||
          msg->angleToParallelRearMarker == -1.0f)) {
        stateMsg.state = worldstate::StateMsg::adjustingToLine;
        timer_.stop();
    }

    if (!(msg->angleToParallelFrontMarker == -1.0f) &&
        !(msg->angleToParallelRearMarker == -1.0f)) {
        if (msg->distanceFromEndRearMarker <
            (*constants_)["ERROR_TOLERANCE_LINE_DISTANCE_TO_END"]) {
            stateMsg.state = worldstate::StateMsg::adjustingToLine;
        }
    }

    if (msg->distanceFromEndOfFrontMarker <
        (*constants_)["ERROR_TOLERANCE_LINE_DISTANCE_TO_END"]) {
        stateMsg.state = worldstate::StateMsg::locatingDie;
        // timer_.stop();
    }

    publishNextState(stateMsg);
}

void FollowLine::timerCallback(const ros::TimerEvent& event) {
    worldstate::StateMsg stateMsg;

    // stateMsg.state = worldstate::StateMsg::locatingDie;

    // publishNextState(stateMsg);
}