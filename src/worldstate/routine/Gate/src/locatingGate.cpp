//
// Created by joel on 07/05/18.
//

#include "locatingGate.h"

void locatingGate::setupNodeSubscriptions (ros::NodeHandle nh){
    std::string gateDetectTopic = "/gateDetect/output";
    nh.subscribe(gateDetectTopic, 10, &locatingGate::gateDetectCallBack, this);
}

void locatingGate::gateDetectCallBack (const gate_detect::gateDetectMsg::ConstPtr & msg){

    worldstate::state_msg msg_to_publish = worldstate::state_msg::locatingGate;

    /* If any of the poles is seen then align with the gate*/
    if (msg->detectLeft || msg->detectRight || msg->detectTop){

        msg_to_publish = worldstate::state_msg::aligningWithGate;

        /* If all three are seen */
        if (msg->detectLeft && msg->detectRight && msg->detectTop){
            double distBtwnHoriontalGates = fabs(msg->distanceLeft - msg->distanceRight);

            if (distBtwnHoriontalGates < errorTolerance && msg->distaceTop < CLEARANCE){
                msg_to_publish = worldstate::state_msg::passingGate;
            }
        }

    }

    publishNextState(msg_to_publish);

}
