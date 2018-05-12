//
// Created by joel on 11/05/18.
//

#ifndef PROJECT_LOCATINGGATE_H
#define PROJECT_LOCATINGGATE_H

#include "State.h"
#include <gate_detect/gateDetectMsg.h>
#include <std_msgs/String.h>
#include <constants.h>

class locatingGate : public State {
public:
    locatingGate (int argc, char** argv, std::string node_name)
        : State(argc, argv, node_name) {}

    void setupNodeSubscriptions (ros::NodeHandle nh) override;

private:

    const double errorTolerance = 30; //cm

    void gateDetectCallBack(const gate_detect::gateDetectMsg::ConstPtr& msg);

};

#endif //PROJECT_LOCATINGGATE_H
