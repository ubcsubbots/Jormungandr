/*
 * Created By: Joel Ahn
 * Created On: March 5th, 2018
 * Description: World State FSM node. Checks to see if
 *              robot still needs to pass gate.
 */

#include "PassGate.h"

std::vector<ros::Subscriber>
PassGate::getNodeSubscriptions(ros::NodeHandle nh) {
    std::string gateDetectTopic = "/gateDetect/output";

    std::vector<ros::Subscriber> subs;
    subs.push_back(
    nh.subscribe(gateDetectTopic, 10, &PassGate::gateDetectCallBack, this));
    // TODO: Use IMU to construct internal world map if need be
    // std::string imuDataTopic    = "/imu/is_calibrated";
    // nh.subscribe("/imu/is_calibrated", 10, &PassGate::imuDataCallback, this);
    return subs;
}

void PassGate::gateDetectCallBack(
const gate_detect::GateDetectMsg::ConstPtr& msg) {
    worldstate::StateMsg msg_to_publish;
    msg_to_publish.state = worldstate::StateMsg::passingGate;

    if (!((msg->detectedTopPole) &&
          abs(msg->angleTopPole -
              subbots::global_constants::TARGET_TOP_POLE_CLEARANCE) < 0.05)) {
        if (!((msg->detectedRightPole && msg->detectedLeftPole) &&
              abs(msg->angleLeftPole + msg->angleRightPole) < 0.05)) {
            msg_to_publish.state = worldstate::StateMsg::passingGate;
        }
    }

    // Let the World State Node know to transition to the next state
    publishNextState(msg_to_publish);
}
