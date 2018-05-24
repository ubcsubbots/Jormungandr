/*
 * Created By: Joel Ahn
 * Created On: March 5th, 2018
 * Description: World State FSM node. Checks to see if
 *              robot still needs to locate gate.
 */

#ifndef PROJECT_LOCATINGGATE_H
#define PROJECT_LOCATINGGATE_H

#include "../State.h"
#include <constants.h>
#include <gate_detect/gateDetectMsg.h>

/**
 * Communicating Class {alignWithGate, locatingGate, passGate}
 */
class LocatingGate : public State {
  public:
    LocatingGate(int argc, char** argv, std::string node_name)
      : State(argc, argv, node_name) {}
    void setupNodeSubscriptions(ros::NodeHandle nh) override;
    void sleep() override;

  private:
    ros::Subscriber gate_detect_listener_;
    void gateDetectCallBack(const gate_detect::gateDetectMsg::ConstPtr& msg);
};

#endif // PROJECT_LOCATINGGATE_H
