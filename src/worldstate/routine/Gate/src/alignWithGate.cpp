//
// Created by joel on 12/05/18.
//
#include "alignWithGate.h"

void alignWithGate::setupNodeSubscriptions(ros::NodeHandle nh) {
    std::string gateDetectTopic = "/gateDetect/output";
    nh.subscribe(gateDetectTopic, 10, &alignWithGate::gateDetectCallBack, this);
}

void alignWithGate::gateDetectCallBack(
const gate_detect::gateDetectMsg::ConstPtr& msg) {
    worldstate::state_msg msg_to_publish;
    msg_to_publish.state = worldstate::state_msg::aligningWithGate;

    if (!(msg->detectLeft || msg->detectRight || msg->detectTop))
        msg_to_publish.state = worldstate::state_msg::locatingGate;

    if (msg->detectLeft && msg->detectRight && msg->detectTop) {
        double distBtwnHorizontalGates =
        fabs(msg->distanceLeft - msg->distanceRight);

        if (distBtwnHorizontalGates < ERR_TOLERANCE_HORZ_GATES &&
            msg->distanceTop > CLEARANCE)
            msg_to_publish.state = worldstate::state_msg::passingGate;
    }

    publishNextState(msg_to_publish);
}