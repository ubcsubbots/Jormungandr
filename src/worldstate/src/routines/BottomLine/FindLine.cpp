/*
* Created By: Cameron Newton
* Created On: July 15th, 2018
* Description: Subroutine to detect whether we've found line
*/

#include "FindLine.h"
#include <worldstate/StateMsg.h>

std::vector<ros::Subscriber>
FindLine::getNodeSubscriptions(ros::NodeHandle nh) {
    std::string line_detect_topic = "line_detect_output";

    timer_ =
    nh.createTimer(ros::Duration(30), &FindLine::timerCallback, this, false);
    timer_.start();

    std::vector<ros::Subscriber> subs;

    subs.push_back(
    nh.subscribe(line_detect_topic, 10, &FindLine::lineDetectCallback, this));

    return subs;
}

void FindLine::lineDetectCallback(
const line_detect::LineDetectMsg::ConstPtr& msg) {
    worldstate::StateMsg stateMsg;

    stateMsg.state = worldstate::StateMsg::findingLine;

    if (!(msg->lateralDistanceFromFrontMarker == -1.0f ||
          msg->distanceFromEndOfFrontMarker == -1.0f ||
          msg->angleToParallelFrontMarker == -1.0f)) {
        timer_.stop();
        stateMsg.state = worldstate::StateMsg::adjustingToLine;
    }

    publishNextState(stateMsg);
}

void FindLine::timerCallback(const ros::TimerEvent& event) {
    worldstate::StateMsg stateMsg;

    // stateMsg.state = worldstate::StateMsg::locatingDie;

    // publishNextState(stateMsg);
}
