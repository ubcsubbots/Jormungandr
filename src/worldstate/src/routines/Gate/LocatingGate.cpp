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
const gate_detect::gateDetectMsg::ConstPtr& msg) {
    worldstate::StateMsg msg_to_publish;
    msg_to_publish.state = worldstate::StateMsg::locatingGate;

    // If any of the poles is seen then align with the gate
    if (msg->detectLeft || msg->detectRight || msg->detectTop) {
        msg_to_publish.state = worldstate::StateMsg::aligningWithGate;

        // If the side poles are seen
        if (msg->detectLeft && msg->detectRight) {
            double distBtwnHorizontalGates =
            fabs(msg->distanceLeft - msg->distanceRight);

            // If the robot is within the bounds of the gate
            if (distBtwnHorizontalGates <
                subbots::global_constants::CLEARANCE_WIDTH) {
                msg_to_publish.state = worldstate::StateMsg::passingGate;

                // If the top pole is seen, but the robot doesn't clear the
                // height limit
                if (msg->detectTop &&
                    msg->distanceTop <
                    subbots::global_constants::CLEARANCE_HEIGHT) {
                    // The robot should try to align
                    msg_to_publish.state =
                    worldstate::StateMsg::aligningWithGate;
                }
            }
        }
    }

    // Let the World State Node know to transition to the next state
    publishNextState(msg_to_publish);
}
