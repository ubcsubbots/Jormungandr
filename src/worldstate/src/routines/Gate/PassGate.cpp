/*
 * Created By: Joel Ahn
 * Created On: March 5th, 2018
 * Description: World State FSM node. Checks to see if
 *              robot still needs to pass gate.
 */

#include "routines/Gate/PassGate.h"

void PassGate::setupNodeSubscriptions(ros::NodeHandle nh) {
    std::string gateDetectTopic = "/gateDetect/output";
    std::string imuDataTopic    = "/imu/is_calibrated";
    gate_detect_listener = nh.subscribe(gateDetectTopic, 10, &PassGate::gateDetectCallBack, this);
    //nh.subscribe("/imu/is_calibrated", 10, &PassGate::imuDataCallback, this);
}

void PassGate::sleep(){
    gate_detect_listener.shutdown();
}

void PassGate::gateDetectCallBack(
const gate_detect::gateDetectMsg::ConstPtr& msg) {
    worldstate::StateMsg msg_to_publish;
    msg_to_publish.state = worldstate::StateMsg::passingGate;

    /*
     * Assume for now if the gate disappears it has successfully
     * passed through the gate
     */
    if (!msg->detectLeft && !msg->detectRight && msg->detectTop){
        msg_to_publish.state = worldstate::StateMsg::locatingDie;
    }

    /*
     * What are the failed states?
     */

    publishNextState(msg_to_publish);
}
