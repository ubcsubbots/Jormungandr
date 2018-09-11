/*
 * Created By: Cameron Newton
 * Created On: July 18th, 2018
 * Description: State to define when robot is adjusting depth to desired level
 */

#ifndef PROJECT_ADJUSTDEPTH_H
#define PROJECT_ADJUSTDEPTH_H

#include "State.h"
#include "constants.h"
#include <gate_detect/GateDetectMsg.h>

/*** Communicating Class {alignWithGate, locatingGate, passGate} ***/
class AdjustDepth : public State {
  public:
    AdjustDepth(std::unordered_map<std::string, double>* constants)
      : State(constants) {}
    std::vector<ros::Subscriber>
    getNodeSubscriptions(ros::NodeHandle nh) override;

  private:
    ros::Timer depth_timer_;

    /**
     * Decides based on image data whether the robot still needs to
     * align with the gate.
     *
     * @param msg gateDetectMsg data
     */
    void gateDetectCallBack(const gate_detect::GateDetectMsg::ConstPtr& msg);

    void timerCallback(const ros::TimerEvent& event);
};

#endif // PROJECT_ADJUSTDEPTH_H
