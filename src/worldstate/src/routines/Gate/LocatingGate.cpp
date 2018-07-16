/*
 * Created By: Joel Ahn
 * Created On: March 5th, 2018
 * Description: World State FSM node. Checks to see if
 *              robot still needs to locate gate.
 */

#include "LocatingGate.h"
#include <worldstate/StateMsg.h>

void LocatingGate::setupNodeSubscriptions(ros::NodeHandle nh) {
    std::string gateDetectTopic = "/gateDetect/output";
    gate_detect_listener_ =
    nh.subscribe(gateDetectTopic, 10, &LocatingGate::gateDetectCallBack, this);
}

void LocatingGate::sleep() {
    gate_detect_listener_.shutdown();
}

void LocatingGate::gateDetectCallBack(
const gate_detect::GateDetectMsg::ConstPtr& msg) {
    worldstate::StateMsg msg_to_publish;
    msg_to_publish.state = worldstate::StateMsg::locatingGate;

    // If any of the poles is seen then align with the gate
    if (msg->detectedLeftPole || msg->detectedRightPole ||
        msg->detectedTopPole) {
        msg_to_publish.state = worldstate::StateMsg::approachingGate;
    }

    // Let the World State Node know to transition to the next state
    publishNextState(msg_to_publish);
}
