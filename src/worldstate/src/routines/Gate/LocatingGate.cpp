/*
 * Created By: Joel Ahn
 * Created On: March 5th, 2018
 * Description: World State FSM node. Checks to see if
 *              robot still needs to locate gate.
 */

#include <worldstate/StateMsg.h>
#include "routines/Gate/LocatingGate.h"

void LocatingGate::setupNodeSubscriptions(ros::NodeHandle nh) {
    std::string gateDetectTopic = "/gateDetect/output";
    gate_detect_listener_ = nh.subscribe(gateDetectTopic, 10, &LocatingGate::gateDetectCallBack, this);
}

void LocatingGate::sleep() {
    gate_detect_listener_.shutdown();
}

void LocatingGate::gateDetectCallBack(
const gate_detect::gateDetectMsg::ConstPtr& msg) {
    worldstate::StateMsg msg_to_publish;
    msg_to_publish.state = worldstate::StateMsg::locatingGate;

    /* If any of the poles is seen then align with the gate*/
    if (msg->detectLeft || msg->detectRight || msg->detectTop) {
        msg_to_publish.state = worldstate::StateMsg::aligningWithGate;

        /* If all three are seen */
        if (msg->detectLeft && msg->detectRight) {
            double distBtwnHorizontalGates =
            fabs(msg->distanceLeft - msg->distanceRight);

            /* If the robot is within the bounds of the gate */
            if (distBtwnHorizontalGates < subbots::global_constants::CLEARANCE_WIDTH) {
                msg_to_publish.state = worldstate::StateMsg::passingGate;

                /*
                 * Assume that if the top pole is not seen then the robot is deep enough
                 * Otherwise, check that there is enough height clearance for the robot to
                 * pass through without hitting the top pole.
                 */
                if (msg->detectTop && msg->distanceTop < subbots::global_constants::CLEARANCE_HEIGHT){
                    msg_to_publish.state = worldstate::StateMsg::aligningWithGate;
                }
            }
        }
    }

    /* Let the World State Node know to transition to the next state*/
    publishNextState(msg_to_publish);
}
