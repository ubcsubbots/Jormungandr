//
// Created by joel on 12/05/18.
//

#ifndef PROJECT_PASSGATE_H
#define PROJECT_PASSGATE_H

#include "State.h"
#include <gate_detect/gateDetectMsg.h>
#include <constants.h>

class passGate : public State {
public:
    passGate (int argc, char** argv, std::string node_name)
            : State(argc, argv, node_name) {}

    void setupNodeSubscriptions (ros::NodeHandle nh) override;

private:
    void gateDetectCallBack(const gate_detect::gateDetectMsg::ConstPtr& msg);

};

#endif //PROJECT_PASSGATE_H
