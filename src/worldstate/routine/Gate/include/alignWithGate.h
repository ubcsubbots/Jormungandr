//
// Created by joel on 12/05/18.
//

#ifndef PROJECT_ALIGNWITHGATE_H
#define PROJECT_ALIGNWITHGATE_H

#include "State.h"
#include <constants.h>
#include <gate_detect/gateDetectMsg.h>

class alignWithGate : public State {
  public:
    alignWithGate(int argc, char** argv, std::string node_name)
      : State(argc, argv, node_name) {}
    void setupNodeSubscriptions(ros::NodeHandle nh) override;

  private:
    void gateDetectCallBack(const gate_detect::gateDetectMsg::ConstPtr& msg);
};

#endif // PROJECT_ALIGNWITHGATE_H
