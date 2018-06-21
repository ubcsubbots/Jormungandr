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
const gate_detect::gateDetectMsg::ConstPtr& msg) {
    worldstate::StateMsg msg_to_publish;
    msg_to_publish.state = worldstate::StateMsg::aligningWithGate;

    // If no poles are seen, then look for it again
    if (!msg->detectLeft && !msg->detectRight && !msg->detectTop) {
        msg_to_publish.state = worldstate::StateMsg::locatingGate;
    }

    // If side poles of the gate are seen
    if (msg->detectLeft && msg->detectRight) {
        double distBtwnHorizontalGates =
        fabs(msg->distanceLeft - msg->distanceRight);

        // Pass through the gate if it centrally aligned
        if (distBtwnHorizontalGates <
            subbots::global_constants::CLEARANCE_WIDTH) {
            msg_to_publish.state = worldstate::StateMsg::passingGate;
        }

        // If the top pole is seen, but the robot doesn't clear the height limit
        if (msg->detectTop &&
            msg->distanceTop < subbots::global_constants::CLEARANCE_HEIGHT) {
            // Keep trying to align
            msg_to_publish.state = worldstate::StateMsg::aligningWithGate;
        }
    }

    // Let the World State Node know to transition to the next state
    publishNextState(msg_to_publish);
}
