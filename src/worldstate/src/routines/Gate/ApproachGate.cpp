/*
 * Created By: Joel Ahn
 * Created On: March 5th, 2018
 * Description: World State FSM node. Checks to see if
 *              robot still needs to align with gate.
 */

#include "ApproachGate.h"
#include <worldstate/StateMsg.h>

void ApproachGate::setupNodeSubscriptions(ros::NodeHandle nh) {
    std::string gateDetectTopic = "/gateDetect/output";
    gate_detect_listener_ =
            nh.subscribe(gateDetectTopic, 10, &ApproachGate::gateDetectCallBack, this);
}

void ApproachGate::sleep() {
    gate_detect_listener_.shutdown();
}

void ApproachGate::gateDetectCallBack(
        const gate_detect::GateDetectMsg::ConstPtr& msg) {
    worldstate::StateMsg msg_to_publish;

    float averageDistanceToGate = (msg->distanceLeftPole + msg->distanceRightPole + msg->distanceTopPole)/(msg->detectedLeftPole + msg->detectedRightPole + msg->detectedTopPole);

    if(averageDistanceToGate > subbots::global_constants::TARGET_AVERAGE_GATE_DISTANCE){
        msg_to_publish.state = worldstate::StateMsg::approachingGate;
    }else{
        msg_to_publish.state = worldstate::StateMsg::aligningWithGate;
    }

    // Let the World State Node know to transition to the next state
    publishNextState(msg_to_publish);
}
