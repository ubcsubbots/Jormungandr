/*
 * Created By: Joel Ahn
 * Created On: March 5th, 2018
 * Description: World State FSM node. Checks to see if
 *              robot still needs to locate gate.
 */

#ifndef PROJECT_LOCATINGGATE_H
#define PROJECT_LOCATINGGATE_H

#include "State.h"
#include <constants.h>
#include <gate_detect/GateDetectMsg.h>

/*** Communicating Class {alignWithGate, locatingGate, passGate} ***/
class LocatingGate : public State {
  public:
    LocatingGate() : State() {}
    std::vector<ros::Subscriber>
    getNodeSubscriptions(ros::NodeHandle nh) override;

  private:
    /**
     * Decides based on image data whether the robot still needs to
     * align with the gate.
     *
     * @param msg gateDetectMsg data
     */
    void gateDetectCallBack(const gate_detect::GateDetectMsg::ConstPtr& msg);
};

#endif // PROJECT_LOCATINGGATE_H
