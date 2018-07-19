/*
 * Created By: Joel Ahn
 * Created On: March 5th, 2018
 * Description: World State FSM node. Checks to see if
 *              robot still needs to align with gate.
 */

#include "AlignWithGate.h"
#include <worldstate/StateMsg.h>

std::vector<ros::Subscriber>
AlignWithGate::getNodeSubscriptions(ros::NodeHandle nh) {
    std::string gateDetectTopic = "/gateDetect/output";

    std::vector<ros::Subscriber> subs;
    subs.push_back(nh.subscribe(
    gateDetectTopic, 10, &AlignWithGate::gateDetectCallBack, this));
    return subs;
}

void AlignWithGate::gateDetectCallBack(
const gate_detect::GateDetectMsg::ConstPtr& msg) {
    worldstate::StateMsg msg_to_publish;

    msg_to_publish.state = worldstate::StateMsg::aligningWithGate;

    // If no poles are seen, then look for it again
    if (!(msg->detectedTopPole || msg->detectedRightPole ||
          msg->detectedLeftPole)) {
        msg_to_publish.state = worldstate::StateMsg::locatingGate;
    }

    if ((msg->detectedTopPole)) {
        float top_pole_clearance =
        (float) sin(msg->angleTopPole) * msg->distanceTopPole;

        if (abs((int) (top_pole_clearance -
                       constants_["TARGET_TOP_POLE_CLEARANCE"])) <
            constants_["ERROR_TOLERANCE_TOP_POLE_CLEARANCE"]) {
            if ((msg->detectedRightPole && msg->detectedLeftPole) &&
                abs((int) (msg->angleLeftPole + msg->angleRightPole)) <
                constants_["ERROR_TOLERANCE_SIDE_POLES_ANGLE"]) {
                if ((((msg->distanceRightPole + msg->distanceLeftPole) / 2) -
                     constants_["TARGET_SIDE_POLES_DISTANCE"]) <
                    constants_["ERROR_TOLERANCE_SIDE_POLES_DISTANCE"]) {
                    msg_to_publish.state = worldstate::StateMsg::passingGate;
                }
            }
        }
    }

    // Let the World State Node know to transition to the next state
    publishNextState(msg_to_publish);
}
