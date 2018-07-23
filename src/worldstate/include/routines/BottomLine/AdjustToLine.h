//
// Created by drdc-s2632 on 17/07/18.
//

#ifndef PROJECT_ADJUSTTOLINE_H
#define PROJECT_ADJUSTTOLINE_H

#include "State.h"
#include <constants.h>
#include <line_detect/LineDetectMsg.h>

/*** Communicating Class {findLine, adjustToLine, followLine} ***/
class AdjustToLine : public State {
  public:
    AdjustToLine(std::unordered_map<std::string, double>* constants)
      : State(constants) {}
    std::vector<ros::Subscriber>
    getNodeSubscriptions(ros::NodeHandle nh) override;

  private:
    /**
     * Decides based on image data whether the robot still needs to
     * align with the gate.
     *
     * @param msg gateDetectMsg data
     */
    void lineDetectCallback(const line_detect::LineDetectMsg::ConstPtr& msg);
};

#endif // PROJECT_ADJUSTTOLINE_H
