/*
 * Created By: Joel Ahn
 * Created On: March 5th, 2018
 * Description: World State FSM node. Checks to see if
 *              robot still needs to pass gate.
 */

#ifndef PROJECT_PASSGATE_H
#define PROJECT_PASSGATE_H

#include "../State.h"
#include <constants.h>
#include <gate_detect/gateDetectMsg.h>

/**
 * Communicating Class {alignWithGate, locatingGate, passGate}
 */
class PassGate : public State {
  public:
    PassGate(int argc, char** argv, std::string node_name)
      : State(argc, argv, node_name) {}

    void setupNodeSubscriptions(ros::NodeHandle nh) override;

  private:
    void gateDetectCallBack(const gate_detect::gateDetectMsg::ConstPtr& msg);
};

#endif // PROJECT_PASSGATE_H
