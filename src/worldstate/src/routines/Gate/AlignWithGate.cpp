/*
 * Created By: Joel Ahn
 * Created On: March 5th, 2018
 * Description: World State FSM node. Checks to see if
 *              robot still needs to align with gate.
 */

#include "AlignWithGate.h"
#include <worldstate/StateMsg.h>

void AlignWithGate::setupNodeSubscriptions(ros::NodeHandle nh) {
    std::string gateDetectTopic = "/gateDetect/output";
    gate_detect_listener_ =
    nh.subscribe(gateDetectTopic, 10, &AlignWithGate::gateDetectCallBack, this);
}

void AlignWithGate::sleep() {
    gate_detect_listener_.shutdown();
}

void AlignWithGate::gateDetectCallBack(
const gate_detect::GateDetectMsg::ConstPtr& msg) {
    worldstate::StateMsg msg_to_publish;
    msg_to_publish.state = worldstate::StateMsg::aligningWithGate;

    // If no poles are seen, then look for it again
    if (!(msg->detectedTopPole || msg->detectedRightPole || msg->detectedLeftPole)) {
        msg_to_publish.state = worldstate::StateMsg::locatingGate;
    }

    if((msg->detectedTopPole) && abs(msg->angleTopPole - subbots::global_constants::TARGET_TOP_POLE_ANGLE) < subbots::global_constants::ERROR_TOLERANCE_TOP_POLE_ANGLE){

        if(abs(msg->distanceTopPole - subbots::global_constants::TARGET_TOP_POLE_DISTANCE) < subbots::global_constants::ERROR_TOLERANCE_TOP_POLE_DISTANCE){

            if((msg->detectedRightPole && msg->detectedLeftPole) && abs(msg->angleLeftPole + msg->angleRightPole) < subbots::global_constants::ERROR_TOLERANCE_SIDE_POLES_ANGLE){

                if((((msg->distanceRightPole + msg->distanceLeftPole) / 2) - subbots::global_constants::TARGET_SIDE_POLES_DISTANCE) < subbots::global_constants::ERROR_TOLERANCE_SIDE_POLES_DISTANCE){

                    msg_to_publish.state = worldstate::StateMsg::passingGate;

                }


            }

        }


    }

    // Let the World State Node know to transition to the next state
    publishNextState(msg_to_publish);
}
