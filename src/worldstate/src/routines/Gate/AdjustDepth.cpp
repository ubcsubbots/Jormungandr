/*
 * Created By: Joel Ahn
 * Created On: March 5th, 2018
 * Description: World State FSM node. Checks to see if
 *              robot still needs to align with gate.
 */

#include "AdjustDepth.h"
#include <worldstate/StateMsg.h>

std::vector<ros::Subscriber>
AdjustDepth::getNodeSubscriptions(ros::NodeHandle nh) {
    std::string gateDetectTopic = "/gate_detect/output";

    timer_ =
    nh.createTimer(ros::Duration(5), &AdjustDepth::timerCallback, this, false);
    timer_.start();

    worldstate::StateMsg stateMsg;

    stateMsg.state = worldstate::StateMsg::adjustingDepth;

    publishNextState(stateMsg);

    std::vector<ros::Subscriber> subs;
    subs.push_back(
    nh.subscribe(gateDetectTopic, 10, &AdjustDepth::gateDetectCallBack, this));
    return subs;
}

void AdjustDepth::gateDetectCallBack(
const gate_detect::GateDetectMsg::ConstPtr& msg) {
    worldstate::StateMsg msg_to_publish;

    // Let the World State Node know to transition to the next state
    publishNextState(msg_to_publish);
}

void AdjustDepth::timerCallback(const ros::TimerEvent& event) {
    worldstate::StateMsg stateMsg;

    stateMsg.state = worldstate::StateMsg::locatingGate;

    publishNextState(stateMsg);
}
