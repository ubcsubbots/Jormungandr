/*
 * Created By: Joel Ahn
 * Created On: March 5th, 2018
 * Description: World State FSM node. Checks to see if
 *              robot still needs to align with gate.
 */

#ifndef PROJECT_ALIGNWITHGATE_H
#define PROJECT_ALIGNWITHGATE_H

#include "State.h"
#include <constants.h>
#include <gate_detect/gateDetectMsg.h>

/*** Communicating Class {alignWithGate, locatingGate, passGate} ***/
class AlignWithGate : public State {
  public:
    AlignWithGate(int argc, char** argv, std::string node_name)
      : State(argc, argv, node_name) {}
    void setupNodeSubscriptions(ros::NodeHandle nh) override;
    void sleep() override;

  private:
    ros::Subscriber gate_detect_listener_;

    /**
     * Decides based on image data whether the robot still needs to
     * align with the gate.
     *
     * @param msg gateDetectMsg data
     */
    void gateDetectCallBack(const gate_detect::gateDetectMsg::ConstPtr& msg);
};

#endif // PROJECT_ALIGNWITHGATE_H
