/*
 * Created By: Joel Ahn
 * Created On: March 5th, 2018
 * Description: World State FSM node. Checks to see if
 *              robot still needs to align with gate.
 */

#ifndef PROJECT_APPROACHGATE_H
#define PROJECT_APPROACHGATE_H

#include "State.h"
#include "constants.h"
#include <gate_detect/GateDetectMsg.h>

/*** Communicating Class {alignWithGate, locatingGate, passGate} ***/
class ApproachGate : public State {
  public:
    ApproachGate(std::unordered_map<std::string, double>* constants)
      : State(constants) {}
    std::vector<ros::Subscriber>
    getNodeSubscriptions(ros::NodeHandle nh) override;

  private:
    ros::Subscriber gate_detect_listener_;

    /**
     * Decides based on image data whether the robot still needs to
     * align with the gate.
     *
     * @param msg gateDetectMsg data
     */
    void gateDetectCallBack(const gate_detect::GateDetectMsg::ConstPtr& msg);
};

#endif // PROJECT_APPROACHGATE_H