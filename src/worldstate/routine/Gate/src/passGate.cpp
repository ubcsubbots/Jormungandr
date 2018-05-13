//
// Created by joel on 12/05/18.
//

#include "passGate.h"

void passGate::setupNodeSubscriptions (ros::NodeHandle nh){
    std::string gateDetectTopic = "/gateDetect/output";
    nh.subscribe(gateDetectTopic, 10, &passGate::gateDetectCallBack, this);
}

void passGate::gateDetectCallBack (const gate_detect::gateDetectMsg::ConstPtr & msg){

    worldstate::state_msg msg_to_publish;
    msg_to_publish.state = worldstate::state_msg::passingGate;

    /*
     * How to account for logic when robot has successfully passed
     * through gate and no longer sees the poles
     */
    if (! (msg->detectLeft || msg->detectRight || msg->detectTop))
        msg_to_publish.state = worldstate::state_msg::locatingGate;

    if (!(msg->detectLeft && msg->detectRight && msg->detectTop))
        msg_to_publish.state = worldstate::state_msg::aligningWithGate;
    else{
        double distBtwnHorizontalGates = fabs(msg->distanceLeft - msg->distanceRight);

        if (distBtwnHorizontalGates > ERR_TOLERANCE_HORZ_GATES ||
            msg->distanceTop < CLEARANCE)
            msg_to_publish.state = worldstate::state_msg::aligningWithGate;
    }

    publishNextState(msg_to_publish);

}
