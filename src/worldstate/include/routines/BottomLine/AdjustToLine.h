/*
* Created By: Cameron Newton
* Created On: July 15th, 2018
* Description: Subroutine to detect whether we're adjusted to line
*/

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
     * align with the line.
     *
     * @param msg LineDetectMsg data
     */
    void lineDetectCallback(const line_detect::LineDetectMsg::ConstPtr& msg);
};

#endif // PROJECT_ADJUSTTOLINE_H
